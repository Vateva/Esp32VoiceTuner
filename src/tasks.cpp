#include "tasks.h"
#include "utilities.h"
#include "audioprocessing.h"
#include "display.h"
#include <esp_timer.h>

// Core 0: audio capture task
void audioTask(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uint8_t coreID = xPortGetCoreID();
    safePrintf("Audio task started on core %d\n", coreID);
    
    TickType_t lastCapture = xTaskGetTickCount();
    TickType_t lastStats = xTaskGetTickCount();
    
    while (true) {
        TickType_t now = xTaskGetTickCount();
        
        // Capture audio at ~15fps (64ms intervals)
        if ((now - lastCapture) >= pdMS_TO_TICKS(64)) {
            AudioBuffer* buffer = new AudioBuffer();
            if (!buffer) {
                safePrint("ERROR: Failed to allocate audio buffer\n");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            if (!buffer->init(2048)) {
                safePrint("ERROR: Failed to initialize audio buffer\n");
                delete buffer;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            buffer->bufferID = getNextBufferID();
            
            if (!captureRealAudio(buffer)) {
                safePrint("ERROR: Failed to capture real audio\n");
                buffer->cleanup();
                delete buffer;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            buffer->queueSendTime = esp_timer_get_time();
            printTiming("queue send", buffer->bufferID, buffer->queueSendTime, buffer->captureTime);
            
            if (xQueueSend(audioQueue, &buffer, 0) != pdPASS) {
                AudioBuffer* oldBuffer;
                if (xQueueReceive(audioQueue, &oldBuffer, 0) == pdPASS) {
                    oldBuffer->cleanup();
                    delete oldBuffer;
                    __atomic_fetch_add(&droppedCount, 1, __ATOMIC_SEQ_CST);
                }
                
                if (xQueueSend(audioQueue, &buffer, 0) != pdPASS) {
                    safePrintf("ERROR: Failed to send buffer %lu\n", buffer->bufferID);
                    buffer->cleanup();
                    delete buffer;
                }
            }
            
            lastCapture = now;
        }
        
        // Periodic statistics output (5 seconds)
        if ((now - lastStats) >= pdMS_TO_TICKS(5000)) {
            uint32_t processed = __atomic_load_n(&processedCount, __ATOMIC_SEQ_CST);
            uint32_t dropped = __atomic_load_n(&droppedCount, __ATOMIC_SEQ_CST);
            
            safePrintf("=== STATS === Processed: %lu, Dropped: %lu, Free heap: %lu\n",
                      processed, dropped, (uint32_t)esp_get_free_heap_size());
            
            UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(audioTaskHandle);
            safePrintf("Audio stack remaining: %lu bytes\n", stackRemaining * 4);
            
            lastStats = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Combined processing and display task
void processingAndDisplayTask(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(150));
    
    uint8_t coreID = xPortGetCoreID();
    safePrintf("Processing+Display task started on core %d\n", coreID);
    
    while (true) {
        AudioBuffer* inputBuffer;
        
        if (xQueueReceive(audioQueue, &inputBuffer, portMAX_DELAY) == pdPASS) {
            if (!inputBuffer || !inputBuffer->validate()) {
                safePrint("ERROR: Received invalid buffer in processing task\n");
                if (inputBuffer) {
                    inputBuffer->cleanup();
                    delete inputBuffer;
                }
                continue;
            }
            
            inputBuffer->queueReceiveTime = esp_timer_get_time();
            printTiming("queue receive", inputBuffer->bufferID, inputBuffer->queueReceiveTime, inputBuffer->captureTime);
            
            bool analysisSuccess = false;
            TuningResult result;
            
            result.processStartTime = esp_timer_get_time();
            result.bufferID = inputBuffer->bufferID;
            result.captureTime = inputBuffer->captureTime;
            printTiming("process start", result.bufferID, result.processStartTime, result.captureTime);

            // Pass 1: coarse period estimation
            int coarsePeriod = findCoarsePeriodYIN(inputBuffer, &result);

            // Pass 2: refined analysis if coarse period found
            if (coarsePeriod > 0) {
                if (yinAnalysis(inputBuffer, &result, coarsePeriod)) {
                    displayResult(&result, inputBuffer);
                    __atomic_fetch_add(&processedCount, 1, __ATOMIC_SEQ_CST);
                    analysisSuccess = true;
                }
            }
            
            if (!analysisSuccess) {
                displayResult(nullptr, inputBuffer);
            }
            
            inputBuffer->cleanup();
            delete inputBuffer;
        }
    }
}