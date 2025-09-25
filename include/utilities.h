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

// power management state
extern PowerState currentPowerState;
extern uint32_t silenceTimer;
extern uint32_t lastSoundTime;

// conditional compilation for all debugging functions
#if ENABLE_SERIAL_MONITOR_PRINT
// when enabled: declare actual function implementations
float calculateTimingMs(uint64_t currentTime, uint64_t captureTime);
void printTiming(const char* stage, uint32_t bufferID, uint64_t currentTime, uint64_t captureTime);
void printTimingSummary(const TuningResult* result, const AudioBuffer* buffer);
bool safePrint(const char* message);
bool safePrintf(const char* format, ...);
#else
// when disabled: replace with inline no-ops that get optimized away completely
inline float calculateTimingMs(uint64_t currentTime, uint64_t captureTime) { return 0.0f; }
inline void printTiming(const char* stage, uint32_t bufferID, uint64_t currentTime, uint64_t captureTime) { }
inline void printTimingSummary(const TuningResult* result, const AudioBuffer* buffer) { }
inline bool safePrint(const char* message) { return true; }
inline bool safePrintf(const char* format, ...) { return true; }
#endif

// utility functions that are always available (not debug-related)
uint32_t getNextBufferID();
void updateStats(uint64_t latency);

// db detection and power management functions (always available)
float calculateDbLevel(float rmsLevel);
void updatePowerState(float dbLevel);
void switchToPowerState(PowerState newState);
void setCpuFrequency(uint32_t freqMhz);

// confidence calculation and smoothing functions (always available)
float calculateYinConfidence(float yinValue);
float calculateSignalConfidence(float rmsLevel);
float calculateFrequencyStability(SmoothingState* state);
float calculateOverallConfidence(const TuningResult* result, const AudioBuffer* buffer, 
                                SmoothingState* state);
float calculateDynamicAlpha(float confidence);

#endif // UTILITIES_H