#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <driver/i2s.h>

// ==================== DEBUG CONFIGURATION ====================
#define ENABLE_TIMING_DEBUG 0

// ==================== DISPLAY PINS (GC9A01) ====================
#define TFT_SCK     1   // SPI clock signal
#define TFT_MOSI    2   // SPI data output (MOSI)
#define TFT_CS      5   // Chip select (active low)
#define TFT_DC      4   // Data/command control
#define TFT_RST     3   // Hardware reset
#define TFT_BLK     6   // Backlight control

// ==================== I2S AUDIO CONFIGURATION ====================
#define I2S_PORT          I2S_NUM_0
#define I2S_SAMPLE_RATE   48000
#define I2S_SAMPLE_BITS   I2S_BITS_PER_SAMPLE_32BIT
#define I2S_CHANNELS      I2S_CHANNEL_MONO
#define I2S_DMA_BUF_COUNT 32
#define I2S_DMA_BUF_LEN   64
#define AUDIO_CALIBRATION_FACTOR 1.0073f

// ==================== INMP441 PINS ====================
#define I2S_SCK_PIN       13   // Serial clock (bit clock)
#define I2S_WS_PIN        12   // Word select (left/right clock)
#define I2S_SD_PIN        11   // Serial data input

// ==================== YIN ALGORITHM PARAMETERS ====================
#define YIN_THRESHOLD         0.15f    // Confidence threshold
#define YIN_SEARCH_WINDOW     0.40f    // ±20% search window around hint

// ==================== DATA STRUCTURES ====================

// Audio buffer structure with heap-allocated sample storage
struct AudioBuffer {
    float* samples;                // heap-allocated audio data pointer
    uint64_t captureTime;          // audio capture timestamp (microseconds)
    uint64_t captureEndTime;       // capture completion timestamp (microseconds)
    uint64_t queueSendTime;        // queue transmission timestamp (microseconds)
    uint64_t queueReceiveTime;     // queue reception timestamp (microseconds)
    uint32_t bufferID;             // unique buffer identifier
    uint16_t sampleCount;          // sample count (max 2048)
    float amplitude;               // peak amplitude in buffer
    float rmsLevel;                // rms level for monitoring
    bool isValid;                  // data integrity flag
    
    // constructor initializes safe defaults
    AudioBuffer() : samples(nullptr), captureTime(0), captureEndTime(0), 
                   queueSendTime(0), queueReceiveTime(0), bufferID(0), 
                   sampleCount(0), amplitude(0.0f), rmsLevel(0.0f), isValid(false) {}
    
    // allocate heap memory for audio samples
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
    
    // release allocated memory
    void cleanup() {
        if (samples) {
            free(samples);
            samples = nullptr;
        }
        isValid = false;
    }
    
    // validate buffer integrity
    bool validate() const {
        return isValid && samples != nullptr && sampleCount > 0 && sampleCount <= 2048;
    }
};

// Frequency analysis result structure
struct TuningResult {
    float frequency;               // detected frequency (hz)
    char noteName[8];             // musical note (e.g., "a4", "c#3")
    int centsOffset;              // cents deviation (-50 to +50)
    uint64_t captureTime;         // original capture timestamp
    uint64_t processStartTime;    // processing start timestamp
    uint64_t acStartTime;         // first pass start timestamp
    uint64_t acEndTime;           // first pass end timestamp
    uint64_t yinStartTime;        // yin analysis start timestamp
    uint64_t yinEndTime;          // yin analysis end timestamp
    uint64_t displayStartTime;    // display update start timestamp
    uint64_t displayEndTime;      // display update end timestamp
    uint32_t bufferID;            // source buffer identifier
    bool isValid;                 // data integrity flag
    
    TuningResult() : frequency(0), centsOffset(0), captureTime(0), 
                    processStartTime(0), acStartTime(0), acEndTime(0),
                    yinStartTime(0), yinEndTime(0), displayStartTime(0),
                    displayEndTime(0), bufferID(0), isValid(false) {
        memset(noteName, 0, sizeof(noteName));
    }
    
    bool validate() const {
        return isValid && frequency >= 20.0f && frequency <= 20000.0f;
    }
};

// Display state tracking
struct DisplayState {
    char lastNoteName[8];
    float lastCentsOffset;
    uint32_t lastUpdateTime;
    bool needsFullRedraw;
    int lastDynamicRadius;
    uint32_t lastDynamicColor;
    bool staticCirclesDrawn;
};

// Performance statistics
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

// ==================== DISPLAY LAYOUT ====================
#define DISPLAY_CENTER_X     120
#define DISPLAY_CENTER_Y     120
#define DISPLAY_INNER_RADIUS  67   // -10 cents circle
#define DISPLAY_MIDDLE_RADIUS 80  // 0 cents circle  
#define DISPLAY_OUTER_RADIUS  93  // +10 cents circle

#endif // CONFIG_H