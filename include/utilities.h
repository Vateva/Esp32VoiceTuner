#ifndef UTILITIES_H
#define UTILITIES_H

#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// thread synchronization objects
extern SemaphoreHandle_t serialMutex;
extern SemaphoreHandle_t displayMutex;
extern SemaphoreHandle_t statsMutex;
extern QueueHandle_t audioQueue;
extern TaskHandle_t audioTaskHandle;
extern TaskHandle_t processingTaskHandle;

// atomic counters for performance tracking
extern volatile uint32_t bufferCounter;
extern volatile uint32_t processedCount;
extern volatile uint32_t droppedCount;

// performance statistics
extern PerformanceStats stats;

// utility functions
float calculateTimingMs(uint64_t currentTime, uint64_t captureTime);
void printTiming(const char* stage, uint32_t bufferID, uint64_t currentTime, uint64_t captureTime);
void printTimingSummary(const TuningResult* result, const AudioBuffer* buffer);
bool safePrint(const char* message);
bool safePrintf(const char* format, ...);
uint32_t getNextBufferID();
void updateStats(uint64_t latency);

// confidence calculation and smoothing functions
float calculateYinConfidence(float yinValue);
float calculateSignalConfidence(float rmsLevel);
float calculateFrequencyStability(SmoothingState* state);
float calculateOverallConfidence(const TuningResult* result, const AudioBuffer* buffer, 
                                SmoothingState* state);
float calculateDynamicAlpha(float confidence);
bool applySmoothingFilter(TuningResult* result, const AudioBuffer* buffer, 
                         SmoothingState* state);

#endif // UTILITIES_H