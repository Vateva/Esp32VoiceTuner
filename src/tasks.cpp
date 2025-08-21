#include "tasks.h"
#include "utilities.h"
#include "audioprocessing.h"
#include "display.h"
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
        
        // maintain ~15fps capture rate (64ms intervals)
        if ((now - lastCapture) >= pdMS_TO_TICKS(64)) {
            // allocate new audio buffer
            AudioBuffer* buffer = new AudioBuffer();
            if (!buffer) {
                safePrint("ERROR: Failed to allocate audio buffer\n");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            // initialize buffer with default sample count
            if (!buffer->init(2048)) {
                safePrint("ERROR: Failed to initialize audio buffer\n");
                delete buffer;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            buffer->bufferID = getNextBufferID();
            
            // capture audio from inmp441
            if (!captureRealAudio(buffer)) {
                safePrint("ERROR: Failed to capture real audio\n");
                buffer->cleanup();
                delete buffer;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
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
        
        // periodic statistics reporting (5 second intervals)
        if ((now - lastStats) >= pdMS_TO_TICKS(5000)) {
            uint32_t processed = __atomic_load_n(&processedCount, __ATOMIC_SEQ_CST);
            uint32_t dropped = __atomic_load_n(&droppedCount, __ATOMIC_SEQ_CST);
            
            safePrintf("=== STATS === Processed: %lu, Dropped: %lu, Free heap: %lu\n",
                      processed, dropped, (uint32_t)esp_get_free_heap_size());
            
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
    
    while (true) {
        AudioBuffer* inputBuffer;
        
        // wait for audio data from capture task
        if (xQueueReceive(audioQueue, &inputBuffer, portMAX_DELAY) == pdPASS) {
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

            // two-pass yin analysis
            int coarsePeriod = findCoarsePeriodYIN(inputBuffer, &result);

            // refined analysis with harmonic validation
            if (coarsePeriod > 0) {
                if (yinAnalysis(inputBuffer, &result, coarsePeriod)) {
                    displayResult(&result, inputBuffer);
                    __atomic_fetch_add(&processedCount, 1, __ATOMIC_SEQ_CST);
                    analysisSuccess = true;
                }
            }
            
            // display no-signal state if analysis failed
            if (!analysisSuccess) {
                displayResult(nullptr, inputBuffer);
            }
            
            // cleanup buffer memory
            inputBuffer->cleanup();
            delete inputBuffer;
        }
    }
}