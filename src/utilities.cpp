#include "utilities.h"
#include <stdarg.h>

// freertos synchronization primitives
SemaphoreHandle_t serialMutex = nullptr;
SemaphoreHandle_t displayMutex = nullptr;
SemaphoreHandle_t statsMutex = nullptr;
QueueHandle_t audioQueue = nullptr;
TaskHandle_t audioTaskHandle = nullptr;
TaskHandle_t processingTaskHandle = nullptr;

// atomic performance counters
volatile uint32_t bufferCounter = 0;
volatile uint32_t processedCount = 0;
volatile uint32_t droppedCount = 0;

PerformanceStats stats;

// convert microsecond timing to milliseconds
float calculateTimingMs(uint64_t currentTime, uint64_t captureTime) {
    if (currentTime == 0 || captureTime == 0) return 0.0f;
    return (float)(currentTime - captureTime) / 1000.0f;
}

// debug timing with mutex protection
void printTiming(const char* stage, uint32_t bufferID, uint64_t currentTime, uint64_t captureTime) {
    #if ENABLE_TIMING_DEBUG
    if (!stage || currentTime == 0 || captureTime == 0) return;
    float timingMs = calculateTimingMs(currentTime, captureTime);
    safePrintf("timing: buffer %lu - %s = %.2f ms\n", bufferID, stage, timingMs);
    #endif
}

// complete pipeline performance analysis
void printTimingSummary(const TuningResult* result, const AudioBuffer* buffer) {
    #if ENABLE_TIMING_DEBUG
    if (!result || !buffer || !result->validate()) return;
    
    // calculate phase timings
    float captureMs = calculateTimingMs(buffer->captureEndTime, buffer->captureTime);
    float queueMs = calculateTimingMs(buffer->queueReceiveTime, buffer->queueSendTime);
    float yin1Ms = calculateTimingMs(result->acEndTime, result->acStartTime); 
    float yinMs = calculateTimingMs(result->yinEndTime, result->yinStartTime);
    float displayMs = calculateTimingMs(result->displayEndTime, result->displayStartTime);
    float totalMs = calculateTimingMs(result->displayEndTime, buffer->captureTime);
    
    // output timing breakdown
    safePrintf("=== TIMING SUMMARY buffer %lu ===\n", buffer->bufferID);
    safePrintf("capture: %.2f ms | queue: %.2f ms | yin coarse: %.2f ms | yin fine: %.2f ms | display: %.2f ms | total: %.2f ms\n",
              captureMs, queueMs, yin1Ms, yinMs, displayMs, totalMs);
    safePrintf("note: %s | cents: %.1f | freq: %.1f hz\n", 
              result->noteName, result->centsOffset, result->frequency);
    safePrintf("======================\n");
    #endif
}

// thread-safe serial output
bool safePrint(const char* message) {
    if (!serialMutex || !message) return false;
    
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.print(message);
        Serial.flush();
        xSemaphoreGive(serialMutex);
        return true;
    }
    return false;
}

// thread-safe formatted output with overflow protection
bool safePrintf(const char* format, ...) {
    if (!serialMutex || !format) return false;
    
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // validate buffer bounds
    if (len > 0 && len < sizeof(buffer)) {
        return safePrint(buffer);
    }
    return false;
}

// atomic buffer identifier generation
uint32_t getNextBufferID() {
    return __atomic_fetch_add(&bufferCounter, 1, __ATOMIC_SEQ_CST);
}

// thread-safe performance metrics update
void updateStats(uint64_t latency) {
    if (!statsMutex) return;
    
    if (xSemaphoreTake(statsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        stats.totalProcessed++;
        stats.totalLatency += latency;
        if (latency < stats.minLatency) stats.minLatency = latency;
        if (latency > stats.maxLatency) stats.maxLatency = latency;
        xSemaphoreGive(statsMutex);
    }
}