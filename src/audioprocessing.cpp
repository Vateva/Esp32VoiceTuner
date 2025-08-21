#include "audioprocessing.h"
#include "utilities.h"
#include <esp_timer.h>
#include <cmath>

// global audio filter state
AudioFilters audioFilters;

// yin period candidate with scoring metrics
struct YinCandidate {
    int period;           // samples per cycle
    float yinValue;       // yin difference function result
    float harmonicScore;  // harmonic content validation
    float totalScore;     // weighted composite score
    
    YinCandidate() : period(0), yinValue(1.0f), harmonicScore(0.0f), totalScore(0.0f) {}
};

// calculate 2nd order butterworth filter coefficients
void calculateButterworthCoefficients(float cutoffHz, float sampleRate, bool isHighpass, IIRFilter* filter) {
    if (!filter || cutoffHz <= 0.0f || sampleRate <= 0.0f) return;
    
    // butterworth filter design parameters
    float omega = 2.0f * M_PI * cutoffHz / sampleRate;
    float k = tanf(omega / 2.0f);
    float k2 = k * k;
    float sqrt2 = sqrtf(2.0f);
    float norm = 1.0f / (1.0f + sqrt2 * k + k2);
    
    if (isHighpass) {
        // highpass coefficients
        filter->b0 = 1.0f * norm;
        filter->b1 = -2.0f * filter->b0;
        filter->b2 = filter->b0;
    } else {
        // lowpass coefficients  
        filter->b0 = k2 * norm;
        filter->b1 = 2.0f * filter->b0;
        filter->b2 = filter->b0;
    }
    
    // feedback coefficients (same for both types)
    filter->a1 = 2.0f * (k2 - 1.0f) * norm;
    filter->a2 = (1.0f - sqrt2 * k + k2) * norm;
    
    // reset filter state
    filter->reset();
}

// initialize audio filtering subsystem
void initAudioFilters() {
    // calculate highpass filter coefficients (removes dc and low frequency noise)
    calculateButterworthCoefficients(HIGHPASS_CUTOFF_HZ, I2S_SAMPLE_RATE, true, &audioFilters.highpass);
    
    // calculate lowpass filter coefficients (removes high frequency noise)
    calculateButterworthCoefficients(LOWPASS_CUTOFF_HZ, I2S_SAMPLE_RATE, false, &audioFilters.lowpass);
    
    audioFilters.filtersReady = true;
    
    safePrintf("Audio filters initialized: HP=%.0f Hz, LP=%.0f Hz\n", 
              HIGHPASS_CUTOFF_HZ, LOWPASS_CUTOFF_HZ);
}

// apply 2nd order iir filter to single sample
float applyIIRFilter(float input, IIRFilter* filter) {
    if (!filter || !filter->initialized) return input;
    
    // direct form i implementation: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float output = filter->b0 * input + 
                  filter->b1 * filter->x1 + 
                  filter->b2 * filter->x2 - 
                  filter->a1 * filter->y1 - 
                  filter->a2 * filter->y2;
    
    // update filter memory (shift delay line)
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    
    return output;
}

// apply cascaded prefiltering to audio buffer
void applyPrefiltering(float* samples, uint16_t sampleCount) {
    if (!samples || sampleCount == 0 || !audioFilters.filtersReady) return;
    
    // process each sample through cascaded filters
    for (uint16_t i = 0; i < sampleCount; i++) {
        // stage 1: highpass filter (remove dc offset and low frequency noise)
        float filtered = applyIIRFilter(samples[i], &audioFilters.highpass);
        
        // stage 2: lowpass filter (remove high frequency noise above harmonics)
        samples[i] = applyIIRFilter(filtered, &audioFilters.lowpass);
    }
}

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
    
    // initialize audio filtering subsystem
    initAudioFilters();
    
    safePrint("I2S initialized successfully\n");
    return true;
}

// read audio samples via i2s dma with prefiltering and db calculation
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
    
    // convert 24-bit to normalized float with gain
    for (uint16_t i = 0; i < buffer->sampleCount; i++) {
        int32_t sample24 = rawSamples[i] >> 8;
        float gain = 3.0f;
        buffer->samples[i] = (float)sample24 / 8388608.0f * gain;
    }
    
    free(rawSamples);
    
    // apply cascaded audio prefiltering (highpass then lowpass)
    applyPrefiltering(buffer->samples, buffer->sampleCount);
    
    // calculate audio metrics on filtered signal
    float maxAmplitude = 0.0f;
    float sumSquares = 0.0f;
    
    for (uint16_t i = 0; i < buffer->sampleCount; i++) {
        // track amplitude statistics
        float absValue = fabsf(buffer->samples[i]);
        if (absValue > maxAmplitude) {
            maxAmplitude = absValue;
        }
        
        sumSquares += buffer->samples[i] * buffer->samples[i];
    }
    
    // store audio metrics
    buffer->amplitude = maxAmplitude;
    buffer->rmsLevel = sqrtf(sumSquares / buffer->sampleCount);
    
    // calculate db level for power management
    buffer->dbLevel = calculateDbLevel(buffer->rmsLevel);
    
    buffer->captureEndTime = esp_timer_get_time();
    printTiming("capture end", buffer->bufferID, buffer->captureEndTime, buffer->captureTime);
    
    // periodic debug output including db level
    static uint32_t debugCounter = 0;
    if ((debugCounter++ % 16) == 0) {
        safePrintf("Audio: Buffer %lu, RMS=%.3f, dB=%.1f (filtered)\n", 
                  buffer->bufferID, buffer->rmsLevel, buffer->dbLevel);
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

    int samplesToUse = (input->sampleCount < YIN_COARSE_SAMPLES) ? input->sampleCount : YIN_COARSE_SAMPLES;
    
    float yinBuffer[YIN_MAX_PERIOD + 1];
    memset(yinBuffer, 0, sizeof(yinBuffer));
    
    // step 1: difference function
    for (int tau = YIN_MIN_PERIOD; tau <= YIN_MAX_PERIOD; tau++) {
        float diff = 0.0f;
        int validSamples = samplesToUse - tau;
        
        if (validSamples < YIN_COARSE_MIN_VALID_SAMPLES) continue;
        
        // sum squared differences
        for (int i = 0; i < validSamples; i++) {
            float delta = input->samples[i] - input->samples[i + tau];
            diff += delta * delta;
        }
        yinBuffer[tau] = diff;
    }
    
    // step 2: cumulative mean normalization
    float runningSum = 0.0f;
    yinBuffer[YIN_MIN_PERIOD] = 1.0f;
    
    for (int tau = YIN_MIN_PERIOD + 1; tau <= YIN_MAX_PERIOD; tau++) {
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
    
    for (int tau = YIN_MIN_PERIOD; tau <= YIN_MAX_PERIOD; tau++) {
        if (yinBuffer[tau] < YIN_COARSE_THRESHOLD) {
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
    if (bestPeriod == 0 || bestValue > YIN_COARSE_QUALITY_GATE) {
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
    const int harmonics[] = YIN_HARMONICS_TO_CHECK;
    const float harmonicWeights[] = YIN_HARMONIC_WEIGHTS;
    const int numHarmonics = sizeof(harmonics) / sizeof(int);
    
    for (int h = 0; h < numHarmonics; h++) {
        int harmonicPeriod = candidatePeriod / harmonics[h];
        
        // skip harmonics too small to analyze reliably
        if (harmonicPeriod < YIN_MIN_HARMONIC_PERIOD) continue;
        
        // autocorrelation at harmonic period
        float harmonicCorrelation = 0.0f;
        float totalEnergy = 0.0f;
        int validSamples = input->sampleCount - harmonicPeriod;
        
        if (validSamples < YIN_MIN_HARMONIC_VALID_SAMPLES) continue;
        
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
            if (normalizedCorr > YIN_HARMONIC_CORRELATION_THRESHOLD) {
                harmonicScore += normalizedCorr * harmonicWeights[h];
                validHarmonics++;
            }
        }
    }
    
    // bonus for multiple harmonics present
    if (validHarmonics >= 2) {
        harmonicScore *= YIN_HARMONIC_BONUS;
    }
    
    return harmonicScore;
}

// frequency preference bias for vocal range
float calculateFrequencyBias(int period) {
    if (period <= 0) return 0.0f;
    float frequency = (float)I2S_SAMPLE_RATE / period;
    
    // vocal range gets full score
    if (frequency >= VOCAL_RANGE_MIN && frequency <= VOCAL_RANGE_MAX) {
        return 1.0f;
    }
    // gradual penalty outside vocal range
    else if (frequency > VOCAL_RANGE_MAX && frequency <= VOCAL_RANGE_HIGH_MAX) {
        return 1.0f - (frequency - VOCAL_RANGE_MAX) / (VOCAL_RANGE_HIGH_MAX - VOCAL_RANGE_MAX) * VOCAL_PENALTY_HIGH;
    }
    else if (frequency >= VOCAL_RANGE_LOW_MIN && frequency < VOCAL_RANGE_MIN) {
        return 1.0f - (VOCAL_RANGE_MIN - frequency) / (VOCAL_RANGE_MIN - VOCAL_RANGE_LOW_MIN) * VOCAL_PENALTY_LOW;
    }
    else {
        return VOCAL_PENALTY_EXTREME; // heavy penalty for extreme frequencies
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
        candidates[i].totalScore = (yinScore * YIN_SCORE_WEIGHT) +
                                  (candidates[i].harmonicScore * HARMONIC_SCORE_WEIGHT) +  
                                  (frequencyBias * FREQUENCY_BIAS_WEIGHT);
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
    
    float yinBuffer[YIN_MAX_PERIOD + 1];

    // focused search window around coarse estimate
    int searchMin = hintedPeriod * (1.0f - YIN_SEARCH_WINDOW);
    int searchMax = hintedPeriod * (1.0f + YIN_SEARCH_WINDOW);

    if (searchMin < YIN_MIN_PERIOD) searchMin = YIN_MIN_PERIOD;
    if (searchMax > YIN_MAX_PERIOD) searchMax = YIN_MAX_PERIOD;

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
    if (searchMin <= YIN_MAX_PERIOD) {
        yinBuffer[searchMin] = 1.0f;
    }

    for (int tau = searchMin + 1; tau <= searchMax; tau++) {
        runningSum += yinBuffer[tau];
        if (runningSum > 0.0f) {
            yinBuffer[tau] *= tau / runningSum;
        } else {
            yinBuffer[tau] = 1.0f;
        }
    }

    // step 3: extract multiple candidates
    YinCandidate candidates[YIN_MAX_CANDIDATES];
    
    int candidateCount = findAllYinCandidates(yinBuffer, searchMin, searchMax, 
                                             candidates, YIN_MAX_CANDIDATES);
    
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
    if (bestPeriod > YIN_MIN_PERIOD && bestPeriod < YIN_MAX_PERIOD) {
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
    if (!noteName || nameSize < 4 || frequency <= 0) return;
    
    const char* noteNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    
    // semitones from A4
    float semitonesFromA4 = 12.0f * log2f(frequency / A4_REFERENCE_PITCH);
    int totalSemitones = (int)roundf(semitonesFromA4);
    
    // calculate octave and note index
    int octave = 4 + (totalSemitones + 9) / 12;
    if (totalSemitones + 9 < 0) octave--;
    
    int noteIndex = ((totalSemitones + 9) % 12 + 12) % 12;
    
    snprintf(noteName, nameSize, "%s%d", noteNames[noteIndex], octave);
}

// cents deviation from equal temperament
int calculateCentsOffset(float frequency) {
    if (frequency <= 0) return 0;

    float semitonesFromA4 = 12.0f * log2f(frequency / A4_REFERENCE_PITCH);
    float nearestSemitone = roundf(semitonesFromA4);
    float perfectFrequency = A4_REFERENCE_PITCH * powf(2.0f, nearestSemitone / 12.0f);
    int centsOffset = 1200.0f * log2f(frequency / perfectFrequency);
    
    return centsOffset;
}