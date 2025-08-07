#include "audioprocessing.h"
#include "utilities.h"
#include <esp_timer.h>
#include <cmath>

// Initialize I2S peripheral for INMP441 microphone
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
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN
    };
    
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
    
    i2s_zero_dma_buffer(I2S_PORT);
    safePrint("I2S initialized successfully\n");
    return true;
}

// Capture audio from INMP441 via I2S DMA
bool captureRealAudio(AudioBuffer* buffer) {
    if (!buffer || !buffer->validate()) {
        safePrint("ERROR: Invalid buffer in captureRealAudio\n");
        return false;
    }
    
    buffer->captureTime = esp_timer_get_time();
    printTiming("capture start", buffer->bufferID, buffer->captureTime, buffer->captureTime);
    
    int32_t* rawSamples = (int32_t*)malloc(buffer->sampleCount * sizeof(int32_t));
    if (!rawSamples) {
        safePrint("ERROR: Failed to allocate raw sample buffer\n");
        return false;
    }
    
    size_t bytesRead = 0;
    esp_err_t result = i2s_read(I2S_PORT, rawSamples, 
                               buffer->sampleCount * sizeof(int32_t), 
                               &bytesRead, pdMS_TO_TICKS(100));
    
    if (result != ESP_OK) {
        safePrintf("ERROR: i2s_read failed: %s\n", esp_err_to_name(result));
        free(rawSamples);
        return false;
    }
    
    uint16_t samplesReceived = bytesRead / sizeof(int32_t);
    if (samplesReceived != buffer->sampleCount) {
        safePrintf("WARNING: Expected %d samples, got %d\n", 
                  buffer->sampleCount, samplesReceived);
        buffer->sampleCount = samplesReceived;
    }
    
    float maxAmplitude = 0.0f;
    float sumSquares = 0.0f;
    
    for (uint16_t i = 0; i < buffer->sampleCount; i++) {
        int32_t sample24 = rawSamples[i] >> 8;
        float gain = 3.0f;
        buffer->samples[i] = (float)sample24 / 8388608.0f * gain;
        
        float absValue = fabsf(buffer->samples[i]);
        if (absValue > maxAmplitude) {
            maxAmplitude = absValue;
        }
        
        sumSquares += buffer->samples[i] * buffer->samples[i];
    }
    
    buffer->amplitude = maxAmplitude;
    buffer->rmsLevel = sqrtf(sumSquares / buffer->sampleCount);
    
    buffer->captureEndTime = esp_timer_get_time();
    printTiming("capture end", buffer->bufferID, buffer->captureEndTime, buffer->captureTime);
    
    free(rawSamples);
    
    static uint32_t debugCounter = 0;
    if ((debugCounter++ % 16) == 0) {
        safePrintf("Audio: Buffer %lu, RMS=%.3f\n", buffer->bufferID, buffer->rmsLevel);
    }
    
    return true;
}

// First pass: coarse YIN for rapid period estimation
int findCoarsePeriodYIN(const AudioBuffer* input, TuningResult* result) {
    if (!input || !result || !input->validate() || input->rmsLevel < 0.01f) {
        return 0;
    }

    result->acStartTime = esp_timer_get_time();
    printTiming("coarse yin start", result->bufferID, result->acStartTime, result->captureTime);

    int MIN_YIN_PERIOD = 32;
    int MAX_YIN_PERIOD = 600;
    
    const float COARSE_THRESHOLD = 0.25f;
    const int COARSE_SAMPLES = 512;
    
    int samplesToUse = (input->sampleCount < COARSE_SAMPLES) ? input->sampleCount : COARSE_SAMPLES;
    
    float yinBuffer[MAX_YIN_PERIOD + 1];
    memset(yinBuffer, 0, sizeof(yinBuffer));
    
    // Step 1: calculate difference function
    for (int tau = MIN_YIN_PERIOD; tau <= MAX_YIN_PERIOD; tau++) {
        float diff = 0.0f;
        int validSamples = samplesToUse - tau;
        
        if (validSamples < 64) continue;
        
        for (int i = 0; i < validSamples; i++) {
            float delta = input->samples[i] - input->samples[i + tau];
            diff += delta * delta;
        }
        yinBuffer[tau] = diff;
    }
    
    // Step 2: cumulative mean normalized difference
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
    
    // Step 3: find first dip below threshold
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
    
    if (bestPeriod == 0 || bestValue > 0.5f) {
        return 0;
    }
    
    return bestPeriod;
}

// Second pass: refined YIN pitch detection
bool yinAnalysis(const AudioBuffer* input, TuningResult* output, int hintedPeriod) {
    if (!input || !output || !input->validate() || hintedPeriod == 0) {
        return false;
    }

    output->yinStartTime = esp_timer_get_time();
    printTiming("yin start", output->bufferID, output->yinStartTime, output->captureTime);

    output->bufferID = input->bufferID;
    output->captureTime = input->captureTime;
    
    int MIN_YIN_PERIOD = 32;
    int MAX_YIN_PERIOD = 600;
    
    float yinBuffer[MAX_YIN_PERIOD + 1];

    int searchMin = hintedPeriod * (1.0f - YIN_SEARCH_WINDOW);
    int searchMax = hintedPeriod * (1.0f + YIN_SEARCH_WINDOW);

    if (searchMin < MIN_YIN_PERIOD) searchMin = MIN_YIN_PERIOD;
    if (searchMax > MAX_YIN_PERIOD) searchMax = MAX_YIN_PERIOD;

    // Step 2: calculate difference function
    for (int tau = searchMin; tau <= searchMax; tau++) {
        float diff = 0.0f;
        for (int i = 0; i < input->sampleCount - tau; i++) {
            float delta = input->samples[i] - input->samples[i + tau];
            diff += delta * delta;
        }
        yinBuffer[tau] = diff;
    }

    // Step 3: cumulative mean normalized difference
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

    // Step 4: find first dip below threshold
    int period = 0;
    for (int tau = searchMin; tau <= searchMax; tau++) {
        if (yinBuffer[tau] < YIN_THRESHOLD) {
            period = tau;
            break;
        }
    }
    
    if (period == 0) {
        return false;
    }

    // Step 5: parabolic interpolation
    float betterPeriod;
    if (period > MIN_YIN_PERIOD && period < MAX_YIN_PERIOD) {
        float y_minus = yinBuffer[period - 1];
        float y_center = yinBuffer[period];
        float y_plus = yinBuffer[period + 1];
        
        float p = (y_plus - y_minus) / (2.0f * (2.0f * y_center - y_plus - y_minus));
        betterPeriod = (float)period + p;
    } else {
        betterPeriod = (float)period;
    }
    
    output->frequency = ((float)I2S_SAMPLE_RATE / betterPeriod) * AUDIO_CALIBRATION_FACTOR;
    convertFrequencyToNote(output->frequency, output->noteName, sizeof(output->noteName));
    output->centsOffset = calculateCentsOffset(output->frequency);
    output->isValid = true;

    output->yinEndTime = esp_timer_get_time();
    printTiming("yin end", output->bufferID, output->yinEndTime, output->captureTime);

    return output->validate();
}

// Convert frequency to musical note name
void convertFrequencyToNote(float frequency, char* noteName, size_t nameSize) {
    if (!noteName || nameSize < 4) return;
    
    const char* noteNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    
    float semitonesFromA4 = 12.0f * log2f(frequency / 440.0f);
    int totalSemitones = (int)roundf(semitonesFromA4);
    
    int octave = 4 + (totalSemitones + 9) / 12;
    if (totalSemitones + 9 < 0) octave--;
    
    int noteIndex = ((totalSemitones + 9) % 12 + 12) % 12;
    
    snprintf(noteName, nameSize, "%s%d", noteNames[noteIndex], octave);
}

// Calculate cents offset from perfect pitch
float calculateCentsOffset(float frequency) {
    float semitonesFromA4 = 12.0f * log2f(frequency / 440.0f);
    float nearestSemitone = roundf(semitonesFromA4);
    float perfectFrequency = 440.0f * powf(2.0f, nearestSemitone / 12.0f);
    float centsOffset = 1200.0f * log2f(frequency / perfectFrequency);
    
    return centsOffset;
}