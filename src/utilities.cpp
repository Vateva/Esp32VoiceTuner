#include "utilities.h"
#include "menu.h"  // add menu.h to access tunerParams
#include <stdarg.h>
#include <cmath>

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

// power management state variables
PowerState currentPowerState = DETECTING;
uint32_t silenceTimer = 0;
uint32_t lastSoundTime = 0;

// ================================================================
// debug/monitoring functions - only compiled when enable_serial_monitor_print is enabled
// ================================================================
#if ENABLE_SERIAL_MONITOR_PRINT

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
    
    // output timing breakdown with confidence metrics
    safePrintf("=== TIMING SUMMARY buffer %lu ===\n", buffer->bufferID);
    safePrintf("capture: %.2f ms | queue: %.2f ms | yin coarse: %.2f ms | yin fine: %.2f ms | display: %.2f ms | total: %.2f ms\n",
              captureMs, queueMs, yin1Ms, yinMs, displayMs, totalMs);
    safePrintf("note: %s | cents: %.1f | freq: %.1f hz\n", 
              result->noteName, result->centsOffset, result->frequency);
    safePrintf("confidence: yin=%.3f, harmonic=%.3f, signal=%.3f, overall=%.3f\n",
              result->yinConfidence, result->harmonicConfidence, result->signalConfidence, result->overallConfidence);
    safePrintf("======================\n");
    #endif
}

// thread-safe serial output
bool safePrint(const char* message) {
    if (!serialMutex || !message) return false;
    
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(PWM_TIMEOUT_MS)) == pdTRUE) {
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

#endif // ENABLE_SERIAL_MONITOR_PRINT

// ================================================================
// core utility functions - always available regardless of debug settings
// ================================================================

// atomic buffer identifier generation
uint32_t getNextBufferID() {
    return __atomic_fetch_add(&bufferCounter, 1, __ATOMIC_SEQ_CST);
}

// calculate db level from rms amplitude
float calculateDbLevel(float rmsLevel) {
    // handle silence floor to prevent log(0)
    if (rmsLevel < 0.0001f) {
        return AUDIO_SILENCE_FLOOR_DB; // silence floor
    }
    
    // convert rms to db: db = 20 * log10(rms / reference)
    float dbLevel = 20.0f * log10f(rmsLevel / DB_REFERENCE_LEVEL);
    
    // clamp to reasonable range
    if (dbLevel < -80.0f) dbLevel = -80.0f;
    if (dbLevel > 0.0f) dbLevel = 0.0f;
    
    return dbLevel;
}

// update power management state with retriggerable timer using runtime parameters
void updatePowerState(float dbLevel) {
    uint32_t currentTime = millis();
    
    // use runtime parameters instead of constants
    float deactivationThreshold = (float)tunerParams.dbDeactivation;
    uint32_t silenceTimeoutMs = tunerParams.silenceTimeout * 1000; // convert seconds to ms
    
    if (dbLevel > deactivationThreshold) {
        // sound detected - reset silence timer
        lastSoundTime = currentTime;
        silenceTimer = silenceTimeoutMs;
        
        // switch to analyzing mode if not already there
        if (currentPowerState == DETECTING) {
            switchToPowerState(ANALYZING);
            safePrintf("POWER: switching to analyzing mode (db=%.1f, threshold=%.1f)\n", 
                      dbLevel, (float)tunerParams.dbActivation);
        }
    } else if (currentPowerState == ANALYZING) {
        // in analyzing mode but no sound - count down silence timer
        uint32_t elapsed = currentTime - lastSoundTime;
        if (elapsed >= silenceTimeoutMs) {
            // silence timeout reached - switch to detecting mode
            switchToPowerState(DETECTING);
            safePrintf("POWER: switching to detecting mode after %.1fs silence (timeout=%ds)\n", 
                      (float)elapsed / 1000.0f, tunerParams.silenceTimeout);
        }
    }
}

// change cpu frequency and update power state
void switchToPowerState(PowerState newState) {
    if (newState == currentPowerState) return;
    
    PowerState oldState = currentPowerState;
    currentPowerState = newState;
    
    // adjust cpu frequency based on state
    if (newState == DETECTING) {
        setCpuFrequency(DETECTING_CPU_FREQ);
        safePrintf("POWER: detecting mode - cpu %d mhz\n", DETECTING_CPU_FREQ);
    } else {
        setCpuFrequency(ANALYZING_CPU_FREQ);
        safePrintf("POWER: analyzing mode - cpu %d mhz\n", ANALYZING_CPU_FREQ);
    }
    
    // reset silence timer when switching to analyzing (use runtime parameter)
    if (newState == ANALYZING) {
        silenceTimer = tunerParams.silenceTimeout * 1000;
        lastSoundTime = millis();
    }
    
    // log power state change
    static uint32_t stateChangeCounter = 0;
    safePrintf("POWER STATE CHANGE #%lu: %s -> %s\n", 
              ++stateChangeCounter, 
              (oldState == DETECTING) ? "DETECTING" : "ANALYZING",
              (newState == DETECTING) ? "DETECTING" : "ANALYZING");
}

// set esp32 cpu frequency with validation
void setCpuFrequency(uint32_t freqMhz) {
    // validate frequency is supported
    if (freqMhz != 80 && freqMhz != 160 && freqMhz != 240) {
        safePrintf("ERROR: unsupported cpu frequency %lu mhz\n", freqMhz);
        return;
    }
    
    uint32_t oldFreq = getCpuFrequencyMhz();
    if (oldFreq == freqMhz) {
        return; // already at target frequency
    }
    
    // change frequency using arduino framework function
    bool success = setCpuFrequencyMhz(freqMhz);
    
    // verify frequency change
    uint32_t newFreq = getCpuFrequencyMhz();
    if (newFreq == freqMhz && success) {
        safePrintf("CPU frequency changed: %lu -> %lu mhz\n", oldFreq, newFreq);
    } else {
        safePrintf("ERROR: cpu frequency change failed %lu -> %lu (got %lu)\n", 
                  oldFreq, freqMhz, newFreq);
    }
}

// convert yin detection value to confidence score
float calculateYinConfidence(float yinValue) {
    // yin values: 0.0 = perfect, 0.15 = threshold, 1.0 = worst
    // confidence: 1.0 = perfect, 0.0 = worst
    if (yinValue <= 0.0f) return 1.0f;
    if (yinValue >= YIN_CONFIDENCE_MAX_VALUE) return 0.0f;
    
    // exponential decay for confidence
    float normalizedYin = yinValue / YIN_CONFIDENCE_MAX_VALUE;
    return expf(-YIN_CONFIDENCE_DECAY_FACTOR * normalizedYin);
}

// calculate signal strength confidence based on rms level
float calculateSignalConfidence(float rmsLevel) {
    // too quiet = low confidence, good level = high confidence
    if (rmsLevel < MIN_RMS_THRESHOLD) return 0.0f;
    if (rmsLevel > SIGNAL_CONFIDENCE_GOOD_LEVEL) return 1.0f;
    
    // sigmoid curve for smooth transition
    float normalizedRms = (rmsLevel - MIN_RMS_THRESHOLD) / (SIGNAL_CONFIDENCE_GOOD_LEVEL - MIN_RMS_THRESHOLD);
    return 1.0f / (1.0f + expf(-SIGNAL_CONFIDENCE_SIGMOID_FACTOR * (normalizedRms - 0.5f)));
}

// measure frequency stability over recent samples
float calculateFrequencyStability(SmoothingState* state) {
    if (!state || state->validSamples < 2) {
        // not enough samples for stability calculation
        return 0.1f; // low stability score for insufficient data
    }
    
    // calculate variance of recent frequency measurements
    float mean = 0.0f;
    int samples = (state->validSamples < FREQUENCY_STABILITY_WINDOW) ? 
                  state->validSamples : FREQUENCY_STABILITY_WINDOW;
    
    // compute mean frequency
    for (int i = 0; i < samples; i++) {
        mean += state->recentFrequencies[i];
    }
    mean /= samples;
    
    // compute variance
    float variance = 0.0f;
    for (int i = 0; i < samples; i++) {
        float diff = state->recentFrequencies[i] - mean;
        variance += diff * diff;
    }
    variance /= samples;
    
    // convert variance to stability score (lower variance = higher stability)
    float standardDeviation = sqrtf(variance);
    float relativeDev = standardDeviation / (mean + 1.0f); // avoid division by zero
    
    // stability score: 1.0 = very stable, 0.0 = very unstable
    float stabilityScore;
    if (relativeDev < STABILITY_MIN_RELATIVE_DEV) {
        stabilityScore = 1.0f;
    } else if (relativeDev > STABILITY_MAX_RELATIVE_DEV) {
        stabilityScore = 0.0f;
    } else {
        stabilityScore = 1.0f - (relativeDev / STABILITY_MAX_RELATIVE_DEV);
    }
    
    // debug stability calculation every 32 samples
    static uint32_t stabilityDebugCounter = 0;
    if (ENABLE_CONFIDENCE_DEBUG && (stabilityDebugCounter++ % 32) == 0 && samples >= 2) {
        safePrintf("STABILITY DEBUG: samples=%d, mean=%.1f, stddev=%.3f, reldev=%.4f, score=%.3f\n",
                  samples, mean, standardDeviation, relativeDev, stabilityScore);
    }
    
    return stabilityScore;
}

// calculate weighted overall confidence score
float calculateOverallConfidence(const TuningResult* result, const AudioBuffer* buffer, 
                                SmoothingState* state) {
    if (!result || !buffer || !state) return 0.0f;
    
    // individual confidence components
    float yinConf = calculateYinConfidence(result->yinConfidence);
    float harmonicConf = result->harmonicConfidence; // already 0-1 from yin analysis
    float signalConf = calculateSignalConfidence(buffer->rmsLevel);
    float stabilityConf = calculateFrequencyStability(state);
    
    // weighted combination (can be tuned based on testing)
    float overallConf = (yinConf * CONFIDENCE_YIN_WEIGHT) +           // 35% yin quality
                       (harmonicConf * CONFIDENCE_HARMONIC_WEIGHT) +  // 25% harmonic content
                       (signalConf * CONFIDENCE_SIGNAL_WEIGHT) +      // 25% signal strength
                       (stabilityConf * CONFIDENCE_STABILITY_WEIGHT); // 15% frequency stability
    
    // detailed confidence debugging output every 16 samples
    static uint32_t debugCounter = 0;
    if (ENABLE_CONFIDENCE_DEBUG && (debugCounter++ % 16) == 0) {
        safePrintf("CONFIDENCE DEBUG:\n");
        safePrintf("  raw yin: %.3f -> conf: %.3f (35%%)\n", result->yinConfidence, yinConf);
        safePrintf("  harmonic: %.3f (25%%)\n", harmonicConf);
        safePrintf("  signal: rms=%.3f -> conf: %.3f (25%%)\n", buffer->rmsLevel, signalConf);
        safePrintf("  stability: %.3f (15%%) [samples=%d]\n", stabilityConf, state->validSamples);
        safePrintf("  OVERALL: %.3f (threshold=%.2f)\n", overallConf, MIN_CONFIDENCE_THRESHOLD);
        safePrintf("  freq: %.1f hz, note: %s, cents: %d\n", 
                  result->frequency, result->noteName, result->centsOffset);
    }
    
    // clamp to valid range
    if (overallConf < 0.0f) overallConf = 0.0f;
    if (overallConf > 1.0f) overallConf = 1.0f;
    
    return overallConf;
}

// map confidence score to ema alpha value based on runtime smoothing level
float calculateDynamicAlpha(float confidence) {
    // use runtime smoothing level parameter to adjust responsiveness
    float smoothingFactor = (float)tunerParams.smoothingLevel / 5.0f; // normalize 1-5 to 0.2-1.0
    
    // calculate base alpha range based on smoothing level
    float minAlpha = EMA_ALPHA_MIN * smoothingFactor;
    float maxAlpha = EMA_ALPHA_MAX * smoothingFactor;
    
    // high confidence = high alpha (responsive)
    // low confidence = low alpha (smooth)
    
    if (confidence <= 0.0f) return minAlpha;
    if (confidence >= 1.0f) return maxAlpha;
    
    // smooth interpolation between min and max alpha
    float alpha = minAlpha + (confidence * (maxAlpha - minAlpha));
    
    // debug alpha calculation every 16 samples
    static uint32_t alphaDebugCounter = 0;
    if (ENABLE_CONFIDENCE_DEBUG && (alphaDebugCounter++ % 16) == 0) {
        safePrintf("SMOOTHING ALPHA: level=%d, factor=%.2f, conf=%.3f -> alpha=%.3f (range %.3f-%.3f)\n",
                  tunerParams.smoothingLevel, smoothingFactor, confidence, alpha, minAlpha, maxAlpha);
    }
    
    return alpha;
}
