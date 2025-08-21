#include "tasks.h"
#include "utilities.h"
#include "audioprocessing.h"
#include "display.h"
#include "menu.h"
#include <esp_timer.h>

// audio capture task (core 0)
void audioTask(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uint8_t coreID = xPortGetCoreID();
    safePrintf("Audio task started on core %d\n", coreID);
    
    TickType_t lastCapture = xTaskGetTickCount();
    TickType_t lastStats = xTaskGetTickCount();
    
    while (true) {
        TickType_t now = xTaskGetTickCount();
        
        // maintain configured capture rate
        if ((now - lastCapture) >= pdMS_TO_TICKS(AUDIO_CAPTURE_INTERVAL_MS)) {
            // allocate new audio buffer
            AudioBuffer* buffer = new AudioBuffer();
            if (!buffer) {
                safePrint("ERROR: Failed to allocate audio buffer\n");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            // initialize buffer with default sample count
            if (!buffer->init(AUDIO_BUFFER_SAMPLES)) {
                safePrint("ERROR: Failed to initialize audio buffer\n");
                delete buffer;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            buffer->bufferID = getNextBufferID();
            
            // capture audio from inmp441 (includes db calculation)
            if (!captureRealAudio(buffer)) {
                safePrint("ERROR: Failed to capture real audio\n");
                buffer->cleanup();
                delete buffer;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            // update power management state based on db level
            updatePowerState(buffer->dbLevel);
            
            // timestamp queue transmission
            buffer->queueSendTime = esp_timer_get_time();
            printTiming("queue send", buffer->bufferID, buffer->queueSendTime, buffer->captureTime);
            
            // send to processing task with queue management
            if (xQueueSend(audioQueue, &buffer, 0) != pdPASS) {
                // drop oldest buffer if queue full
                AudioBuffer* oldBuffer;
                if (xQueueReceive(audioQueue, &oldBuffer, 0) == pdPASS) {
                    oldBuffer->cleanup();
                    delete oldBuffer;
                    __atomic_fetch_add(&droppedCount, 1, __ATOMIC_SEQ_CST);
                }
                
                // retry sending current buffer
                if (xQueueSend(audioQueue, &buffer, 0) != pdPASS) {
                    safePrintf("ERROR: Failed to send buffer %lu\n", buffer->bufferID);
                    buffer->cleanup();
                    delete buffer;
                }
            }
            
            lastCapture = now;
        }
        
        // periodic statistics reporting
        if ((now - lastStats) >= pdMS_TO_TICKS(STATS_REPORT_INTERVAL_MS)) {
            uint32_t processed = __atomic_load_n(&processedCount, __ATOMIC_SEQ_CST);
            uint32_t dropped = __atomic_load_n(&droppedCount, __ATOMIC_SEQ_CST);
            
            safePrintf("=== STATS === Processed: %lu, Dropped: %lu, Free heap: %lu\n",
                      processed, dropped, (uint32_t)esp_get_free_heap_size());
            
            // power state reporting
            const char* stateStr = (currentPowerState == DETECTING) ? "DETECTING" : "ANALYZING";
            uint32_t timeToSleep = 0;
            if (currentPowerState == ANALYZING && silenceTimer > 0) {
                timeToSleep = silenceTimer;
            }
            safePrintf("Power State: %s | CPU: %d MHz | Time to sleep: %lu ms\n",
                      stateStr, getCpuFrequencyMhz(), timeToSleep);
            
            // stack usage monitoring
            UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(audioTaskHandle);
            safePrintf("Audio stack remaining: %lu bytes\n", stackRemaining * 4);
            
            lastStats = now;
        }
        
        // minimal task yield
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// processing and display task (core 1)
void processingAndDisplayTask(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(150));
    
    uint8_t coreID = xPortGetCoreID();
    safePrintf("Processing+Display task started on core %d\n", coreID);
    
    // initialize menu system (gpio setup and menu structure)
    initMenuSystem();
    
    // button polling timing variables
    TickType_t lastButtonCheck = xTaskGetTickCount();
    
    while (true) {
        AudioBuffer* inputBuffer;
        
        // calculate dynamic button check interval based on menu state
        TickType_t buttonCheckInterval;
        if (menuSystem.currentMode == MENU_HIDDEN) {
            // slow polling for menu trigger detection
            buttonCheckInterval = pdMS_TO_TICKS(BUTTON_POLL_INTERVAL_MS);
        } else {
            // fast polling for responsive menu navigation
            buttonCheckInterval = pdMS_TO_TICKS(BUTTON_MENU_POLL_INTERVAL_MS);
        }
        
        // calculate timeout for queue receive (allows button polling)
        TickType_t now = xTaskGetTickCount();
        TickType_t timeSinceLastCheck = now - lastButtonCheck;
        TickType_t queueTimeout;
        
        if (timeSinceLastCheck >= buttonCheckInterval) {
            // time for button check - use minimal timeout
            queueTimeout = 0;
        } else {
            // wait remaining time until next button check
            queueTimeout = buttonCheckInterval - timeSinceLastCheck;
        }
        
        // wait for audio data with calculated timeout
        if (xQueueReceive(audioQueue, &inputBuffer, queueTimeout) == pdPASS) {
            // audio buffer received - process normally
            
            // validate received buffer
            if (!inputBuffer || !inputBuffer->validate()) {
                safePrint("ERROR: Received invalid buffer in processing task\n");
                if (inputBuffer) {
                    inputBuffer->cleanup();
                    delete inputBuffer;
                }
                continue;
            }
            
            // timestamp queue reception
            inputBuffer->queueReceiveTime = esp_timer_get_time();
            printTiming("queue receive", inputBuffer->bufferID, inputBuffer->queueReceiveTime, inputBuffer->captureTime);
            
            bool analysisSuccess = false;
            TuningResult result;
            
            // timestamp processing start
            result.processStartTime = esp_timer_get_time();
            result.bufferID = inputBuffer->bufferID;
            result.captureTime = inputBuffer->captureTime;
            printTiming("process start", result.bufferID, result.processStartTime, result.captureTime);

            // power state decision: only do full yin analysis if in analyzing mode
            if (currentPowerState == ANALYZING) {
                // full two-pass yin analysis
                int coarsePeriod = findCoarsePeriodYIN(inputBuffer, &result);

                // refined analysis with harmonic validation
                if (coarsePeriod > 0) {
                    if (yinAnalysis(inputBuffer, &result, coarsePeriod)) {
                        displayResult(&result, inputBuffer);
                        __atomic_fetch_add(&processedCount, 1, __ATOMIC_SEQ_CST);
                        analysisSuccess = true;
                    }
                }
            } else {
                // detecting mode - minimal processing, just log db level occasionally
                static uint32_t detectingCounter = 0;
                if ((detectingCounter++ % 32) == 0) {
                    safePrintf("DETECTING MODE: db=%.1f (threshold=%.1f)\n", 
                              inputBuffer->dbLevel, DB_ACTIVATION_THRESHOLD);
                }
                
                // fake processing timing for consistency
                result.processStartTime = esp_timer_get_time();
                result.acStartTime = result.processStartTime;
                result.acEndTime = result.processStartTime + 100;
                result.yinStartTime = result.acEndTime;
                result.yinEndTime = result.yinStartTime + 100;
                printTiming("detecting mode skip", result.bufferID, result.yinEndTime, result.captureTime);
            }
            
            // display state management (but not if menu is active)
            if (!analysisSuccess && menuSystem.currentMode == MENU_HIDDEN) {
                if (currentPowerState == DETECTING) {
                    // show detecting mode display
                    displayDetectingMode();
                } else {
                    // show no-signal state in analyzing mode
                    displayResult(nullptr, inputBuffer);
                }
            }
            
            // cleanup buffer memory
            inputBuffer->cleanup();
            delete inputBuffer;
            
        } else {
            // queue timeout occurred - time to check buttons
            now = xTaskGetTickCount();
            
            if ((now - lastButtonCheck) >= buttonCheckInterval) {
                // perform 5hz button polling and menu updates
                checkButtonsAndUpdateMenu();
                lastButtonCheck = now;
                
                // optional: brief yield to ensure smooth operation
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
    }
}