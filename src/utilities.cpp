#include "utilities.h"
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

PerformanceStats stats;

// power management state variables
PowerState currentPowerState = DETECTING;
uint32_t silenceTimer = 0;
uint32_t lastSoundTime = 0;

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

// calculate db level from rms amplitude
float calculateDbLevel(float rmsLevel) {
    // handle silence floor to prevent log(0)
    if (rmsLevel < 0.0001f) {
        return -80.0f; // silence floor
    }
    
    // convert rms to db: db = 20 * log10(rms / reference)
    float dbLevel = 20.0f * log10f(rmsLevel / DB_REFERENCE_LEVEL);
    
    // clamp to reasonable range
    if (dbLevel < -80.0f) dbLevel = -80.0f;
    if (dbLevel > 0.0f) dbLevel = 0.0f;
    
    return dbLevel;
}

// determine if sound level warrants full analysis
bool shouldActivateAnalysis(float dbLevel) {
    // use hysteresis to prevent rapid switching
    if (currentPowerState == DETECTING) {
        return dbLevel > DB_ACTIVATION_THRESHOLD;
    } else {
        // already in analyzing mode, check for continued sound
        return dbLevel > DB_DEACTIVATION_THRESHOLD;
    }
}

// update power management state with retriggerable timer
void updatePowerState(float dbLevel) {
    uint32_t currentTime = millis();
    
    if (dbLevel > DB_DEACTIVATION_THRESHOLD) {
        // sound detected - reset silence timer
        lastSoundTime = currentTime;
        silenceTimer = SILENCE_TIMEOUT_MS;
        
        // switch to analyzing mode if not already there
        if (currentPowerState == DETECTING) {
            switchToPowerState(ANALYZING);
            safePrintf("POWER: switching to analyzing mode (db=%.1f)\n", dbLevel);
        }
    } else if (currentPowerState == ANALYZING) {
        // in analyzing mode but no sound - count down silence timer
        uint32_t elapsed = currentTime - lastSoundTime;
        if (elapsed >= SILENCE_TIMEOUT_MS) {
            // 10 seconds of silence - switch to detecting mode
            switchToPowerState(DETECTING);
            safePrintf("POWER: switching to detecting mode after %.1fs silence\n", 
                      (float)elapsed / 1000.0f);
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
    
    // reset silence timer when switching to analyzing
    if (newState == ANALYZING) {
        silenceTimer = SILENCE_TIMEOUT_MS;
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
    if (yinValue >= 0.5f) return 0.0f;
    
    // exponential decay for confidence
    float normalizedYin = yinValue / 0.5f;
    return expf(-3.0f * normalizedYin);
}

// calculate signal strength confidence based on rms level
float calculateSignalConfidence(float rmsLevel) {
    // too quiet = low confidence, good level = high confidence
    if (rmsLevel < MIN_RMS_THRESHOLD) return 0.0f;
    if (rmsLevel > 0.1f) return 1.0f;
    
    // sigmoid curve for smooth transition
    float normalizedRms = (rmsLevel - MIN_RMS_THRESHOLD) / (0.1f - MIN_RMS_THRESHOLD);
    return 1.0f / (1.0f + expf(-8.0f * (normalizedRms - 0.5f)));
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
    if (relativeDev < 0.001f) {
        stabilityScore = 1.0f;
    } else if (relativeDev > 0.02f) {
        stabilityScore = 0.0f;
    } else {
        stabilityScore = 1.0f - (relativeDev / 0.02f);
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
    float overallConf = (yinConf * 0.35f) +           // 35% yin quality
                       (harmonicConf * 0.25f) +       // 25% harmonic content
                       (signalConf * 0.25f) +         // 25% signal strength
                       (stabilityConf * 0.15f);       // 15% frequency stability
    
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

// map confidence score to ema alpha value
float calculateDynamicAlpha(float confidence) {
    // high confidence = high alpha (responsive)
    // low confidence = low alpha (smooth)
    
    if (confidence <= 0.0f) return EMA_ALPHA_MIN;
    if (confidence >= 1.0f) return EMA_ALPHA_MAX;
    
    // smooth interpolation between min and max alpha
    return EMA_ALPHA_MIN + (confidence * (EMA_ALPHA_MAX - EMA_ALPHA_MIN));
}

// apply confidence-based ema smoothing to tuning result
bool applySmoothingFilter(TuningResult* result, const AudioBuffer* buffer, 
                         SmoothingState* state) {
    if (!result || !buffer || !state || !result->validate()) {
        return false;
    }
    
    // update frequency history for stability calculation
    state->recentFrequencies[state->frequencyIndex] = result->frequency;
    state->frequencyIndex = (state->frequencyIndex + 1) % FREQUENCY_STABILITY_WINDOW;
    if (state->validSamples < FREQUENCY_STABILITY_WINDOW) {
        state->validSamples++;
    }
    
    // calculate confidence metrics
    result->yinConfidence = calculateYinConfidence(result->yinConfidence); // convert raw yin value
    result->signalConfidence = calculateSignalConfidence(buffer->rmsLevel);
    float stabilityConf = calculateFrequencyStability(state);
    result->overallConfidence = calculateOverallConfidence(result, buffer, state);
    
    // check minimum confidence threshold
    if (result->overallConfidence < MIN_CONFIDENCE_THRESHOLD) {
        if (ENABLE_CONFIDENCE_DEBUG) {
            safePrintf("REJECT: confidence %.3f < threshold %.2f\n", 
                      result->overallConfidence, MIN_CONFIDENCE_THRESHOLD);
        }
        return false;
    }
    
    // calculate dynamic alpha based on confidence
    float alpha = calculateDynamicAlpha(result->overallConfidence);
    
    // first measurement initialization
    if (!state->initialized) {
        state->smoothedFrequency = result->frequency;
        state->smoothedCents = result->centsOffset;
        state->initialized = true;
        state->lastConfidence = result->overallConfidence;
        state->lastUpdateTime = millis();
        
        if (ENABLE_CONFIDENCE_DEBUG) {
            safePrintf("SMOOTHING INIT: freq=%.1f, cents=%d, conf=%.3f, alpha=%.3f\n",
                      state->smoothedFrequency, (int)state->smoothedCents, 
                      result->overallConfidence, alpha);
        }
        return true;
    }
    
    // store original values for comparison
    float originalFreq = result->frequency;
    float originalCents = result->centsOffset;
    
    // apply exponential moving average
    state->smoothedFrequency = (alpha * result->frequency) + 
                              ((1.0f - alpha) * state->smoothedFrequency);
    state->smoothedCents = (alpha * result->centsOffset) + 
                          ((1.0f - alpha) * state->smoothedCents);
    
    // update result with smoothed values
    result->frequency = state->smoothedFrequency;
    result->centsOffset = (int)roundf(state->smoothedCents);
    
    // recalculate note name based on smoothed frequency
    const char* noteNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    float semitonesFromA4 = 12.0f * log2f(result->frequency / 440.0f);
    int totalSemitones = (int)roundf(semitonesFromA4);
    int octave = 4 + (totalSemitones + 9) / 12;
    if (totalSemitones + 9 < 0) octave--;
    int noteIndex = ((totalSemitones + 9) % 12 + 12) % 12;
    snprintf(result->noteName, sizeof(result->noteName), "%s%d", noteNames[noteIndex], octave);
    
    // update state tracking
    state->lastConfidence = result->overallConfidence;
    state->lastUpdateTime = millis();
    
    // smoothing debug output every 8 samples
    static uint32_t smoothDebugCounter = 0;
    if (ENABLE_CONFIDENCE_DEBUG && (smoothDebugCounter++ % 8) == 0) {
        safePrintf("SMOOTHING APPLIED:\n");
        safePrintf("  conf=%.3f -> alpha=%.3f\n", result->overallConfidence, alpha);
        safePrintf("  freq: %.1f -> %.1f (delta: %.1f)\n", 
                  originalFreq, result->frequency, result->frequency - originalFreq);
        safePrintf("  cents: %.1f -> %d (delta: %.1f)\n", 
                  originalCents, result->centsOffset, result->centsOffset - originalCents);
    }
    
    return true;
}