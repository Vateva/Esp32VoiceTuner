#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <driver/i2s.h>

// debug output control
#define ENABLE_TIMING_DEBUG 0
#define ENABLE_CONFIDENCE_DEBUG 1

// gc9a01 round display pins
#define TFT_SCK     1   // spi clock
#define TFT_MOSI    2   // spi data out
#define TFT_CS      5   // chip select
#define TFT_DC      4   // data/command
#define TFT_RST     3   // reset
#define TFT_BLK     6   // backlight

// i2s audio configuration
#define I2S_PORT          I2S_NUM_0
#define I2S_SAMPLE_RATE   48000
#define I2S_SAMPLE_BITS   I2S_BITS_PER_SAMPLE_32BIT
#define I2S_CHANNELS      I2S_CHANNEL_MONO
#define I2S_DMA_BUF_COUNT 32
#define I2S_DMA_BUF_LEN   64
#define AUDIO_CALIBRATION_FACTOR 1.0000f

// inmp441 microphone pins
#define I2S_SCK_PIN       13   // bit clock
#define I2S_WS_PIN        12   // word select
#define I2S_SD_PIN        11   // serial data

// yin algorithm tuning
#define YIN_THRESHOLD         0.15f    // detection confidence
#define YIN_SEARCH_WINDOW     0.40f    // ±40% search range

// smoothing configuration
#define MIN_CONFIDENCE_THRESHOLD  0.15f  // minimum confidence for display update
#define MIN_RMS_THRESHOLD        0.008f  // minimum signal level
#define EMA_ALPHA_MIN            0.08f   // heavy smoothing for low confidence
#define EMA_ALPHA_MAX            0.65f   // light smoothing for high confidence
#define FREQUENCY_STABILITY_WINDOW 5     // samples for stability calculation

// audio buffer with heap allocation
struct AudioBuffer {
    float* samples;                // audio data pointer
    uint64_t captureTime;          // capture start timestamp
    uint64_t captureEndTime;       // capture end timestamp
    uint64_t queueSendTime;        // queue send timestamp
    uint64_t queueReceiveTime;     // queue receive timestamp
    uint32_t bufferID;             // unique identifier
    uint16_t sampleCount;          // sample count
    float amplitude;               // peak amplitude
    float rmsLevel;                // rms level
    bool isValid;                  // validity flag
    
    AudioBuffer() : samples(nullptr), captureTime(0), captureEndTime(0), 
                   queueSendTime(0), queueReceiveTime(0), bufferID(0), 
                   sampleCount(0), amplitude(0.0f), rmsLevel(0.0f), isValid(false) {}
    
    // heap allocation for samples
    bool init(uint16_t size = 2048) {
        if (samples) free(samples);
        samples = (float*)malloc(size * sizeof(float));
        if (samples) {
            sampleCount = size;
            isValid = true;
            return true;
        }
        return false;
    }
    
    // memory cleanup
    void cleanup() {
        if (samples) {
            free(samples);
            samples = nullptr;
        }
        isValid = false;
    }
    
    // data integrity check
    bool validate() const {
        return isValid && samples != nullptr && sampleCount > 0 && sampleCount <= 2048;
    }
};

// pitch detection result with confidence metrics
struct TuningResult {
    float frequency;               // detected frequency
    char noteName[8];             // musical note name
    int centsOffset;              // cents deviation
    uint64_t captureTime;         // capture timestamp
    uint64_t processStartTime;    // processing start
    uint64_t acStartTime;         // coarse analysis start
    uint64_t acEndTime;           // coarse analysis end
    uint64_t yinStartTime;        // yin analysis start
    uint64_t yinEndTime;          // yin analysis end
    uint64_t displayStartTime;    // display start
    uint64_t displayEndTime;      // display end
    uint32_t bufferID;            // source buffer id
    bool isValid;                 // result validity
    
    // confidence metrics for smoothing
    float yinConfidence;          // yin detection quality (0-1, higher better)
    float harmonicConfidence;     // harmonic content score (0-1, higher better)
    float signalConfidence;       // signal strength score (0-1, higher better)
    float overallConfidence;      // weighted composite confidence (0-1)
    
    TuningResult() : frequency(0), centsOffset(0), captureTime(0), 
                    processStartTime(0), acStartTime(0), acEndTime(0),
                    yinStartTime(0), yinEndTime(0), displayStartTime(0),
                    displayEndTime(0), bufferID(0), isValid(false),
                    yinConfidence(0), harmonicConfidence(0), signalConfidence(0),
                    overallConfidence(0) {
        memset(noteName, 0, sizeof(noteName));
    }
    
    // frequency range validation
    bool validate() const {
        return isValid && frequency >= 20.0f && frequency <= 20000.0f;
    }
};

// smoothing state for temporal filtering
struct SmoothingState {
    float smoothedFrequency;        // ema smoothed frequency
    float smoothedCents;            // ema smoothed cents deviation
    float recentFrequencies[FREQUENCY_STABILITY_WINDOW];  // frequency history
    int frequencyIndex;             // circular buffer index
    int validSamples;               // number of valid samples in history
    bool initialized;               // first measurement flag
    uint32_t lastUpdateTime;        // timestamp of last update
    float lastConfidence;           // previous confidence score
    
    SmoothingState() : smoothedFrequency(0), smoothedCents(0), frequencyIndex(0),
                      validSamples(0), initialized(false), lastUpdateTime(0),
                      lastConfidence(0) {
        memset(recentFrequencies, 0, sizeof(recentFrequencies));
    }
    
    // reset smoothing state
    void reset() {
        smoothedFrequency = 0;
        smoothedCents = 0;
        frequencyIndex = 0;
        validSamples = 0;
        initialized = false;
        lastUpdateTime = 0;
        lastConfidence = 0;
        memset(recentFrequencies, 0, sizeof(recentFrequencies));
    }
};

// display state management
struct DisplayState {
    char lastNoteName[8];
    float lastCentsOffset;
    uint32_t lastUpdateTime;
    bool needsFullRedraw;
    int lastDynamicRadius;
    uint32_t lastDynamicColor;
    bool staticCirclesDrawn;
    SmoothingState smoothing;       // temporal smoothing state
};

// performance monitoring
struct PerformanceStats {
    uint32_t totalProcessed;
    uint32_t totalDropped;
    uint64_t totalLatency;
    uint64_t minLatency;
    uint64_t maxLatency;
    bool initialized;
    
    PerformanceStats() : totalProcessed(0), totalDropped(0), totalLatency(0),
                        minLatency(UINT64_MAX), maxLatency(0), initialized(true) {}
};

// tuner display layout
#define CENTS_UNDER_THRESHOLD -15 // flat threshold
#define CENTS_OVER_THRESHOLD   15 // sharp threshold
#define DISPLAY_CENTER_X       120
#define DISPLAY_CENTER_Y       120
#define DISPLAY_INNER_RADIUS   67   // flat circle
#define DISPLAY_MIDDLE_RADIUS  80   // perfect pitch
#define DISPLAY_OUTER_RADIUS   93   // sharp circle

#endif // CONFIG_H