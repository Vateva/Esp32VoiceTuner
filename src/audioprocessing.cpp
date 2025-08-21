#include "audioprocessing.h"
#include "utilities.h"
#include <esp_timer.h>
#include <cmath>

// yin period candidate with scoring metrics
struct YinCandidate {
    int period;           // samples per cycle
    float yinValue;       // yin difference function result
    float harmonicScore;  // harmonic content validation
    float totalScore;     // weighted composite score
    
    YinCandidate() : period(0), yinValue(1.0f), harmonicScore(0.0f), totalScore(0.0f) {}
};

// configure i2s for inmp441 microphone
bool initI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_SAMPLE_BITS,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = I2S_DMA_BUF_COUNT,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    
    // pin mapping for inmp441
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN
    };
    
    // install and configure i2s driver
    esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (result != ESP_OK) {
        safePrintf("ERROR: i2s_driver_install failed: %s\n", esp_err_to_name(result));
        return false;
    }
    
    result = i2s_set_pin(I2S_PORT, &pin_config);
    if (result != ESP_OK) {
        safePrintf("ERROR: i2s_set_pin failed: %s\n", esp_err_to_name(result));
        i2s_driver_uninstall(I2S_PORT);
        return false;
    }
    
    // clear dma buffers
    i2s_zero_dma_buffer(I2S_PORT);
    safePrint("I2S initialized successfully\n");
    return true;
}

// read audio samples via i2s dma
bool captureRealAudio(AudioBuffer* buffer) {
    if (!buffer || !buffer->validate()) {
        safePrint("ERROR: Invalid buffer in captureRealAudio\n");
        return false;
    }
    
    // timestamp capture start
    buffer->captureTime = esp_timer_get_time();
    printTiming("capture start", buffer->bufferID, buffer->captureTime, buffer->captureTime);
    
    // allocate temp buffer for raw 32-bit samples
    int32_t* rawSamples = (int32_t*)malloc(buffer->sampleCount * sizeof(int32_t));
    if (!rawSamples) {
        safePrint("ERROR: Failed to allocate raw sample buffer\n");
        return false;
    }
    
    // read from i2s with timeout
    size_t bytesRead = 0;
    esp_err_t result = i2s_read(I2S_PORT, rawSamples, 
                               buffer->sampleCount * sizeof(int32_t), 
                               &bytesRead, pdMS_TO_TICKS(100));
    
    if (result != ESP_OK) {
        safePrintf("ERROR: i2s_read failed: %s\n", esp_err_to_name(result));
        free(rawSamples);
        return false;
    }
    
    // validate sample count
    uint16_t samplesReceived = bytesRead / sizeof(int32_t);
    if (samplesReceived != buffer->sampleCount) {
        safePrintf("WARNING: Expected %d samples, got %d\n", 
                  buffer->sampleCount, samplesReceived);
        buffer->sampleCount = samplesReceived;
    }
    
    float maxAmplitude = 0.0f;
    float sumSquares = 0.0f;
    
    // convert 24-bit to normalized float with gain
    for (uint16_t i = 0; i < buffer->sampleCount; i++) {
        int32_t sample24 = rawSamples[i] >> 8;
        float gain = 3.0f;
        buffer->samples[i] = (float)sample24 / 8388608.0f * gain;
        
        // track amplitude statistics
        float absValue = fabsf(buffer->samples[i]);
        if (absValue > maxAmplitude) {
            maxAmplitude = absValue;
        }
        
        sumSquares += buffer->samples[i] * buffer->samples[i];
    }
    
    // calculate audio metrics
    buffer->amplitude = maxAmplitude;
    buffer->rmsLevel = sqrtf(sumSquares / buffer->sampleCount);
    
    buffer->captureEndTime = esp_timer_get_time();
    printTiming("capture end", buffer->bufferID, buffer->captureEndTime, buffer->captureTime);
    
    free(rawSamples);
    
    // periodic debug output
    static uint32_t debugCounter = 0;
    if ((debugCounter++ % 16) == 0) {
        safePrintf("Audio: Buffer %lu, RMS=%.3f\n", buffer->bufferID, buffer->rmsLevel);
    }
    
    return true;
}

// fast coarse period estimation using limited samples
int findCoarsePeriodYIN(const AudioBuffer* input, TuningResult* result) {
    if (!input || !result || !input->validate() || input->rmsLevel < 0.01f) {
        return 0;
    }

    result->acStartTime = esp_timer_get_time();
    printTiming("coarse yin start", result->bufferID, result->acStartTime, result->captureTime);

    // coarse analysis parameters
    int MIN_YIN_PERIOD = 32;
    int MAX_YIN_PERIOD = 600;
    
    const float COARSE_THRESHOLD = 0.25f;
    const int COARSE_SAMPLES = 512;
    
    int samplesToUse = (input->sampleCount < COARSE_SAMPLES) ? input->sampleCount : COARSE_SAMPLES;
    
    float yinBuffer[MAX_YIN_PERIOD + 1];
    memset(yinBuffer, 0, sizeof(yinBuffer));
    
    // step 1: difference function
    for (int tau = MIN_YIN_PERIOD; tau <= MAX_YIN_PERIOD; tau++) {
        float diff = 0.0f;
        int validSamples = samplesToUse - tau;
        
        if (validSamples < 64) continue;
        
        // sum squared differences
        for (int i = 0; i < validSamples; i++) {
            float delta = input->samples[i] - input->samples[i + tau];
            diff += delta * delta;
        }
        yinBuffer[tau] = diff;
    }
    
    // step 2: cumulative mean normalization
    float runningSum = 0.0f;
    yinBuffer[MIN_YIN_PERIOD] = 1.0f;
    
    for (int tau = MIN_YIN_PERIOD + 1; tau <= MAX_YIN_PERIOD; tau++) {
        runningSum += yinBuffer[tau];
        if (runningSum > 0.0f) {
            yinBuffer[tau] *= tau / runningSum;
        } else {
            yinBuffer[tau] = 1.0f;
        }
    }
    
    // step 3: threshold search and fallback minimum
    int bestPeriod = 0;
    float bestValue = 1.0f;
    
    for (int tau = MIN_YIN_PERIOD; tau <= MAX_YIN_PERIOD; tau++) {
        if (yinBuffer[tau] < COARSE_THRESHOLD) {
            bestPeriod = tau;
            break;
        }
        
        if (yinBuffer[tau] < bestValue) {
            bestValue = yinBuffer[tau];
            bestPeriod = tau;
        }
    }
    
    result->acEndTime = esp_timer_get_time();
    printTiming("coarse yin end", result->bufferID, result->acEndTime, result->captureTime);
    
    // quality gate
    if (bestPeriod == 0 || bestValue > 0.5f) {
        return 0;
    }
    
    return bestPeriod;
}

// extract period candidates from yin buffer
int findAllYinCandidates(const float* yinBuffer, int searchMin, int searchMax, 
                         YinCandidate* candidates, int maxCandidates) {
    int candidateCount = 0;
    
    // scan for local minima below threshold
    for (int tau = searchMin + 1; tau < searchMax - 1 && candidateCount < maxCandidates; tau++) {
        // local minimum test
        if (yinBuffer[tau] < YIN_THRESHOLD &&
            yinBuffer[tau] < yinBuffer[tau - 1] &&
            yinBuffer[tau] < yinBuffer[tau + 1]) {
            
            candidates[candidateCount].period = tau;
            candidates[candidateCount].yinValue = yinBuffer[tau];
            candidateCount++;
        }
    }
    
    // fallback to global minimum if no candidates
    if (candidateCount == 0) {
        float minValue = 1.0f;
        int minPeriod = 0;
        
        for (int tau = searchMin; tau <= searchMax; tau++) {
            if (yinBuffer[tau] < minValue) {
                minValue = yinBuffer[tau];
                minPeriod = tau;
            }
        }
        
        if (minPeriod > 0) {
            candidates[0].period = minPeriod;
            candidates[0].yinValue = minValue;
            candidateCount = 1;
        }
    }
    
    return candidateCount;
}

// validate fundamental by checking harmonic relationships
float scoreHarmonicContent(int candidatePeriod, const AudioBuffer* input) {
    if (!input || candidatePeriod <= 0) return 0.0f;
    
    float harmonicScore = 0.0f;
    int validHarmonics = 0;
    
    // harmonic analysis configuration
    int harmonics[] = {2, 3, 4};
    float harmonicWeights[] = {0.8f, 0.6f, 0.4f};
    
    for (int h = 0; h < 3; h++) {
        int harmonicPeriod = candidatePeriod / harmonics[h];
        
        // skip harmonics too small to analyze reliably
        if (harmonicPeriod < 16) continue;
        
        // autocorrelation at harmonic period
        float harmonicCorrelation = 0.0f;
        float totalEnergy = 0.0f;
        int validSamples = input->sampleCount - harmonicPeriod;
        
        if (validSamples < 32) continue;
        
        // calculate correlation and energy
        for (int i = 0; i < validSamples; i++) {
            float sample1 = input->samples[i];
            float sample2 = input->samples[i + harmonicPeriod];
            
            harmonicCorrelation += sample1 * sample2;
            totalEnergy += sample1 * sample1;
        }
        
        // normalize and weight harmonic strength
        if (totalEnergy > 0.0f) {
            float normalizedCorr = harmonicCorrelation / totalEnergy;
            if (normalizedCorr > 0.1f) {
                harmonicScore += normalizedCorr * harmonicWeights[h];
                validHarmonics++;
            }
        }
    }
    
    // bonus for multiple harmonics present
    if (validHarmonics >= 2) {
        harmonicScore *= 1.5f;
    }
    
    return harmonicScore;
}

// frequency preference bias for vocal range
float calculateFrequencyBias(int period) {
    float frequency = (float)I2S_SAMPLE_RATE / period;
    
    // vocal range gets full score (80-600 hz)
    if (frequency >= 80.0f && frequency <= 600.0f) {
        return 1.0f;
    }
    // gradual penalty outside vocal range
    else if (frequency > 600.0f && frequency <= 1000.0f) {
        return 1.0f - (frequency - 600.0f) / 400.0f * 0.3f;
    }
    else if (frequency >= 40.0f && frequency < 80.0f) {
        return 1.0f - (80.0f - frequency) / 40.0f * 0.2f;
    }
    else {
        return 0.5f; // heavy penalty for extreme frequencies
    }
}

// rank candidates using weighted scoring
int selectBestCandidate(YinCandidate* candidates, int candidateCount, const AudioBuffer* input) {
    if (candidateCount == 0) return -1;
    
    // score each candidate with weighted criteria
    for (int i = 0; i < candidateCount; i++) {
        // invert yin value (lower is better)
        float yinScore = (1.0f - candidates[i].yinValue) * 2.0f;
        
        // harmonic validation score
        candidates[i].harmonicScore = scoreHarmonicContent(candidates[i].period, input);
        
        // frequency range preference
        float frequencyBias = calculateFrequencyBias(candidates[i].period);
        
        // weighted composite score
        candidates[i].totalScore = (yinScore * 0.4f) +           // 40% yin quality
                                  (candidates[i].harmonicScore * 0.5f) +  // 50% harmonic validation  
                                  (frequencyBias * 0.1f);        // 10% frequency preference
    }
    
    // find highest scoring candidate
    int bestIndex = 0;
    float bestScore = candidates[0].totalScore;
    
    for (int i = 1; i < candidateCount; i++) {
        if (candidates[i].totalScore > bestScore) {
            bestScore = candidates[i].totalScore;
            bestIndex = i;
        }
    }
    
    return bestIndex;
}

// multi-candidate yin with harmonic validation
bool yinAnalysis(const AudioBuffer* input, TuningResult* output, int hintedPeriod) {
    if (!input || !output || !input->validate() || hintedPeriod == 0) {
        return false;
    }

    output->yinStartTime = esp_timer_get_time();
    printTiming("yin start", output->bufferID, output->yinStartTime, output->captureTime);

    // copy timing and id info
    output->bufferID = input->bufferID;
    output->captureTime = input->captureTime;
    
    int MIN_YIN_PERIOD = 32;
    int MAX_YIN_PERIOD = 600;
    
    float yinBuffer[MAX_YIN_PERIOD + 1];

    // focused search window around coarse estimate
    int searchMin = hintedPeriod * (1.0f - YIN_SEARCH_WINDOW);
    int searchMax = hintedPeriod * (1.0f + YIN_SEARCH_WINDOW);

    if (searchMin < MIN_YIN_PERIOD) searchMin = MIN_YIN_PERIOD;
    if (searchMax > MAX_YIN_PERIOD) searchMax = MAX_YIN_PERIOD;

    // step 1: difference function
    for (int tau = searchMin; tau <= searchMax; tau++) {
        float diff = 0.0f;
        for (int i = 0; i < input->sampleCount - tau; i++) {
            float delta = input->samples[i] - input->samples[i + tau];
            diff += delta * delta;
        }
        yinBuffer[tau] = diff;
    }

    // step 2: cumulative mean normalization
    float runningSum = 0.0f;
    yinBuffer[searchMin] = 1.0f;

    for (int tau = searchMin + 1; tau <= searchMax; tau++) {
        runningSum += yinBuffer[tau];
        if (runningSum > 0.0f) {
            yinBuffer[tau] *= tau / runningSum;
        } else {
            yinBuffer[tau] = 1.0f;
        }
    }

    // step 3: extract multiple candidates
    const int MAX_CANDIDATES = 8;
    YinCandidate candidates[MAX_CANDIDATES];
    
    int candidateCount = findAllYinCandidates(yinBuffer, searchMin, searchMax, 
                                             candidates, MAX_CANDIDATES);
    
    if (candidateCount == 0) {
        return false;
    }
    
    // step 4: harmonic validation and ranking
    int bestCandidateIndex = selectBestCandidate(candidates, candidateCount, input);
    
    if (bestCandidateIndex < 0) {
        return false;
    }
    
    int bestPeriod = candidates[bestCandidateIndex].period;
    float bestYinValue = candidates[bestCandidateIndex].yinValue;
    float bestHarmonicScore = candidates[bestCandidateIndex].harmonicScore;

    // step 5: parabolic interpolation for sub-sample precision
    float betterPeriod;
    if (bestPeriod > MIN_YIN_PERIOD && bestPeriod < MAX_YIN_PERIOD) {
        float y_minus = yinBuffer[bestPeriod - 1];
        float y_center = yinBuffer[bestPeriod];
        float y_plus = yinBuffer[bestPeriod + 1];
        
        // parabolic fit to find sub-sample peak
        float p = (y_plus - y_minus) / (2.0f * (2.0f * y_center - y_plus - y_minus));
        betterPeriod = (float)bestPeriod + p;
    } else {
        betterPeriod = (float)bestPeriod;
    }
    
    // convert to musical notation
    output->frequency = ((float)I2S_SAMPLE_RATE / betterPeriod) * AUDIO_CALIBRATION_FACTOR;
    convertFrequencyToNote(output->frequency, output->noteName, sizeof(output->noteName));
    output->centsOffset = calculateCentsOffset(output->frequency);
    output->isValid = true;

    // store confidence metrics for smoothing (raw values, will be converted in utilities)
    output->yinConfidence = bestYinValue;           // raw yin value (lower = better)
    output->harmonicConfidence = bestHarmonicScore; // harmonic score (higher = better, 0-1)
    // signalConfidence and overallConfidence calculated in utilities

    output->yinEndTime = esp_timer_get_time();
    printTiming("yin end", output->bufferID, output->yinEndTime, output->captureTime);

    // periodic debug output
    static uint32_t debugCounter = 0;
    if (ENABLE_TIMING_DEBUG && (debugCounter++ % 32) == 0) {
        safePrintf("candidates found: %d, best period: %d, yin: %.3f, harmonic: %.3f\n", 
                  candidateCount, bestPeriod, bestYinValue, bestHarmonicScore);
    }

    return output->validate();
}

// frequency to musical note conversion
void convertFrequencyToNote(float frequency, char* noteName, size_t nameSize) {
    if (!noteName || nameSize < 4) return;
    
    const char* noteNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    
    // semitones from A4 (440 hz)
    float semitonesFromA4 = 12.0f * log2f(frequency / 440.0f);
    int totalSemitones = (int)roundf(semitonesFromA4);
    
    // calculate octave and note index
    int octave = 4 + (totalSemitones + 9) / 12;
    if (totalSemitones + 9 < 0) octave--;
    
    int noteIndex = ((totalSemitones + 9) % 12 + 12) % 12;
    
    snprintf(noteName, nameSize, "%s%d", noteNames[noteIndex], octave);
}

// cents deviation from equal temperament
int calculateCentsOffset(float frequency) {
    float semitonesFromA4 = 12.0f * log2f(frequency / 440.0f);
    float nearestSemitone = roundf(semitonesFromA4);
    float perfectFrequency = 440.0f * powf(2.0f, nearestSemitone / 12.0f);
    int centsOffset = 1200.0f * log2f(frequency / perfectFrequency);
    
    return centsOffset;
}