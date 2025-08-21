#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <driver/i2s.h>

// =================================================================
// DEBUGGING
// =================================================================
#define ENABLE_TIMING_DEBUG 0
#define ENABLE_CONFIDENCE_DEBUG 1
#define SERIAL_BAUD_RATE 115200

// =================================================================
// HARDWARE PINS
// =================================================================
// GC9A01 Round Display
#define TFT_SCK     1   // SPI clock
#define TFT_MOSI    2   // SPI data out
#define TFT_CS      5   // Chip select
#define TFT_DC      4   // Data/command
#define TFT_RST     3   // Reset
#define TFT_BLK     6   // Backlight

// INMP441 Microphone
#define I2S_SCK_PIN       13   // Bit clock
#define I2S_WS_PIN        12   // Word select
#define I2S_SD_PIN        11   // Serial data

// =================================================================
// I2S & AUDIO PROCESSING
// =================================================================
#define I2S_PORT          I2S_NUM_0
#define I2S_SAMPLE_RATE   48000
#define I2S_SAMPLE_BITS   I2S_BITS_PER_SAMPLE_32BIT
#define I2S_CHANNELS      I2S_CHANNEL_MONO
#define I2S_DMA_BUF_COUNT 32
#define I2S_DMA_BUF_LEN   64
#define AUDIO_BUFFER_SAMPLES 2048
#define AUDIO_CALIBRATION_FACTOR 1.0000f

// Audio Prefiltering
#define HIGHPASS_CUTOFF_HZ    60.0f    // Remove dc offset and low frequency noise
#define LOWPASS_CUTOFF_HZ   6000.0f    // Remove high frequency noise above harmonics
#define FILTER_ORDER             2     // 2nd order butterworth filters

// =================================================================
// YIN PITCH DETECTION ALGORITHM
// =================================================================
#define YIN_MIN_PERIOD 32
#define YIN_MAX_PERIOD 600

// Coarse Pass
#define YIN_COARSE_THRESHOLD 0.25f
#define YIN_COARSE_SAMPLES 512
#define YIN_COARSE_MIN_VALID_SAMPLES 64
#define YIN_COARSE_QUALITY_GATE 0.5f

// Refined Pass
#define YIN_THRESHOLD         0.15f    // Detection confidence
#define YIN_SEARCH_WINDOW     0.40f    // ±40% search range
#define YIN_MAX_CANDIDATES    8        // Max candidates for refined analysis

// Harmonic Scoring
#define YIN_HARMONICS_TO_CHECK {2, 3, 4}
#define YIN_HARMONIC_WEIGHTS {0.8f, 0.6f, 0.4f}
#define YIN_MIN_HARMONIC_PERIOD 16
#define YIN_MIN_HARMONIC_VALID_SAMPLES 32
#define YIN_HARMONIC_CORRELATION_THRESHOLD 0.1f
#define YIN_HARMONIC_BONUS 1.5f

// Candidate Selection Weights
#define YIN_SCORE_WEIGHT 0.4f
#define HARMONIC_SCORE_WEIGHT 0.5f
#define FREQUENCY_BIAS_WEIGHT 0.1f

// Vocal Range Frequency Bias
#define VOCAL_RANGE_MIN 80.0f
#define VOCAL_RANGE_MAX 600.0f
#define VOCAL_RANGE_HIGH_MAX 1000.0f
#define VOCAL_RANGE_LOW_MIN 40.0f
#define VOCAL_PENALTY_HIGH 0.3f
#define VOCAL_PENALTY_LOW 0.2f
#define VOCAL_PENALTY_EXTREME 0.5f

// =================================================================
// SMOOTHING & CONFIDENCE
// =================================================================
#define MIN_CONFIDENCE_THRESHOLD  0.15f  // Minimum confidence for display update
#define MIN_RMS_THRESHOLD        0.008f  // Minimum signal level
#define EMA_ALPHA_MIN            0.08f   // Heavy smoothing for low confidence
#define EMA_ALPHA_MAX            0.65f   // Light smoothing for high confidence
#define FREQUENCY_STABILITY_WINDOW 5     // Samples for stability calculation
#define A4_REFERENCE_PITCH       440.0f  // A4 = 440 Hz

// =================================================================
// POWER MANAGEMENT
// =================================================================
#define DB_ACTIVATION_THRESHOLD   -15.0f  // dB level to enter analyzing mode
#define DB_DEACTIVATION_THRESHOLD -25.0f  // dB level threshold for silence detection
#define DB_REFERENCE_LEVEL        0.1f    // Reference rms level for dB calculation
#define SILENCE_TIMEOUT_MS        5000    // Time to return to detecting mode
#define DETECTING_CPU_FREQ        80      // MHz in detecting mode
#define ANALYZING_CPU_FREQ        240     // MHz in analyzing mode

// =================================================================
// FREERTOS TASK CONFIGURATION
// =================================================================
#define AUDIO_TASK_STACK_SIZE 16384
#define AUDIO_TASK_PRIORITY 3
#define AUDIO_TASK_CORE 0
#define PROCESSING_TASK_STACK_SIZE 16384
#define PROCESSING_TASK_PRIORITY 2
#define PROCESSING_TASK_CORE 1
#define AUDIO_QUEUE_LENGTH 4
#define AUDIO_CAPTURE_INTERVAL_MS 64
#define STATS_REPORT_INTERVAL_MS 5000
#define QUEUE_FULL_SKIP_THRESHOLD 2

// =================================================================
// DISPLAY LAYOUT & CONFIG
// =================================================================
// SPI Bus
#define TFT_SPI_WRITE_FREQ 40000000
#define TFT_SPI_READ_FREQ  16000000

// Panel Dimensions
#define TFT_PANEL_WIDTH 240
#define TFT_PANEL_HEIGHT 240

// Tuner Interface
#define CENTS_UNDER_THRESHOLD -15 // Flat threshold
#define CENTS_OVER_THRESHOLD   15 // Sharp threshold
#define DISPLAY_CENTER_X       120
#define DISPLAY_CENTER_Y       120
#define DISPLAY_INNER_RADIUS   67   // Flat circle
#define DISPLAY_MIDDLE_RADIUS  80   // Perfect pitch
#define DISPLAY_OUTER_RADIUS   93   // Sharp circle

// Dynamic Circle Drawing
#define DYNAMIC_CIRCLE_EXTREME_SCALE 1.0f
#define DYNAMIC_CIRCLE_MAX_RADIUS_OFFSET 26
#define DYNAMIC_CIRCLE_MIN_RADIUS_OFFSET 27
#define DYNAMIC_CIRCLE_COLLISION_BUFFER 1

// Text & UI Elements
#define NOTE_TEXT_POS_X 120
#define NOTE_TEXT_POS_Y 120
#define CENTS_TEXT_POS_X 120
#define CENTS_TEXT_POS_Y 97
#define TEXT_CLEAR_AREA_X 93
#define TEXT_CLEAR_AREA_Y 97
#define TEXT_CLEAR_AREA_WIDTH 51
#define TEXT_CLEAR_AREA_HEIGHT 44
#define CENTS_UPDATE_THRESHOLD 2 // Cents change needed to trigger redraw

// Color Thresholds
#define CENTS_COLOR_THRESHOLD_GREENYELLOW 3
#define CENTS_COLOR_THRESHOLD_YELLOW 5
#define CENTS_COLOR_THRESHOLD_ORANGE 7

// button pins (gpio numbers)
#define BUTTON_DOWN_PIN   10   // button for menu down navigation (loops around)
#define BUTTON_SELECT_PIN 9    // button for menu entry and item selection

// button timing and polling
#define BUTTON_POLL_INTERVAL_MS        200   // 5hz button polling rate for menu trigger
#define BUTTON_MENU_POLL_INTERVAL_MS   50    // 20hz fast polling rate inside menu  
#define BUTTON_DEBOUNCE_MS             50    // debounce time for clean detection
#define BUTTON_ACTIVE_LOW              true  // buttons are active low (pulled up)

// menu structure configuration  
#define MENU_MAX_ITEMS             5     // maximum items per menu level
#define MENU_MAX_LEVELS            2     // two level deep menu (main + sub)
#define MENU_TIMEOUT_MS            10000 // auto-exit menu after 10 seconds inactive

// menu display layout
#define MENU_TITLE_Y               40    // y position for menu title
#define MENU_ITEM_START_Y          70    // y position for first menu item
#define MENU_ITEM_HEIGHT           25    // spacing between menu items
#define MENU_SELECTION_INDENT      10    // indent for selected item indicator
#define MENU_TEXT_SIZE             1     // text size for menu items

// menu colors
#define MENU_BACKGROUND_COLOR      TFT_BLACK      // menu background
#define MENU_TITLE_COLOR           TFT_CYAN       // menu title text
#define MENU_ITEM_COLOR            TFT_WHITE      // normal menu item text
#define MENU_SELECTED_COLOR        TFT_GREEN      // selected menu item text
#define MENU_INDICATOR_COLOR       TFT_YELLOW     // selection indicator color

// Power management states
enum PowerState {
    DETECTING,    // Low power mode, simple dB monitoring only
    ANALYZING     // Full power mode, complete yin analysis
};

// 2nd order IIR filter state for real-time processing
struct IIRFilter {
    float b0, b1, b2;        // Feedforward coefficients
    float a1, a2;            // Feedback coefficients (a0 normalized to 1)
    float x1, x2;            // Previous input samples
    float y1, y2;            // Previous output samples
    bool initialized;        // Initialization flag
    
    IIRFilter() : b0(1.0f), b1(0.0f), b2(0.0f), a1(0.0f), a2(0.0f),
                  x1(0.0f), x2(0.0f), y1(0.0f), y2(0.0f), initialized(false) {}
    
    // Reset filter state for new audio session
    void reset() {
        x1 = x2 = y1 = y2 = 0.0f;
        initialized = true;
    }
};

// Audio filter state management
struct AudioFilters {
    IIRFilter highpass;      // DC removal and low frequency noise
    IIRFilter lowpass;       // High frequency noise removal
    bool filtersReady;       // Initialization status
    
    AudioFilters() : filtersReady(false) {}
    
    void reset() {
        highpass.reset();
        lowpass.reset();
        filtersReady = true;
    }
};

// Audio buffer with heap allocation
struct AudioBuffer {
    float* samples;                // Audio data pointer
    uint64_t captureTime;          // Capture start timestamp
    uint64_t captureEndTime;       // Capture end timestamp
    uint64_t queueSendTime;        // Queue send timestamp
    uint64_t queueReceiveTime;     // Queue receive timestamp
    uint32_t bufferID;             // Unique identifier
    uint16_t sampleCount;          // Sample count
    float amplitude;               // Peak amplitude
    float rmsLevel;                // RMS level
    float dbLevel;                 // dB level (calculated from RMS)
    bool isValid;                  // Validity flag
    
    AudioBuffer() : samples(nullptr), captureTime(0), captureEndTime(0), 
                   queueSendTime(0), queueReceiveTime(0), bufferID(0), 
                   sampleCount(0), amplitude(0.0f), rmsLevel(0.0f), dbLevel(-80.0f), isValid(false) {}
    
    // Heap allocation for samples
    bool init(uint16_t size = AUDIO_BUFFER_SAMPLES) {
        if (samples) free(samples);
        samples = (float*)malloc(size * sizeof(float));
        if (samples) {
            sampleCount = size;
            isValid = true;
            return true;
        }
        return false;
    }
    
    // Memory cleanup
    void cleanup() {
        if (samples) {
            free(samples);
            samples = nullptr;
        }
        isValid = false;
    }
    
    // Data integrity check
    bool validate() const {
        return isValid && samples != nullptr && sampleCount > 0 && sampleCount <= AUDIO_BUFFER_SAMPLES;
    }
};

// Pitch detection result with confidence metrics
struct TuningResult {
    float frequency;               // Detected frequency
    char noteName[8];             // Musical note name
    int centsOffset;              // Cents deviation
    uint64_t captureTime;         // Capture timestamp
    uint64_t processStartTime;    // Processing start
    uint64_t acStartTime;         // Coarse analysis start
    uint64_t acEndTime;           // Coarse analysis end
    uint64_t yinStartTime;        // Yin analysis start
    uint64_t yinEndTime;          // Yin analysis end
    uint64_t displayStartTime;    // Display start
    uint64_t displayEndTime;      // Display end
    uint32_t bufferID;            // Source buffer id
    bool isValid;                 // Result validity
    
    // Confidence metrics for smoothing
    float yinConfidence;          // Yin detection quality (0-1, higher better)
    float harmonicConfidence;     // Harmonic content score (0-1, higher better)
    float signalConfidence;       // Signal strength score (0-1, higher better)
    float overallConfidence;      // Weighted composite confidence (0-1)
    
    TuningResult() : frequency(0), centsOffset(0), captureTime(0), 
                    processStartTime(0), acStartTime(0), acEndTime(0),
                    yinStartTime(0), yinEndTime(0), displayStartTime(0),
                    displayEndTime(0), bufferID(0), isValid(false),
                    yinConfidence(0), harmonicConfidence(0), signalConfidence(0),
                    overallConfidence(0) {
        memset(noteName, 0, sizeof(noteName));
    }
    
    // Frequency range validation
    bool validate() const {
        return isValid && frequency >= 20.0f && frequency <= 20000.0f;
    }
};

// Smoothing state for temporal filtering
struct SmoothingState {
    float smoothedFrequency;        // EMA smoothed frequency
    float smoothedCents;            // EMA smoothed cents deviation
    float recentFrequencies[FREQUENCY_STABILITY_WINDOW];  // Frequency history
    int frequencyIndex;             // Circular buffer index
    int validSamples;               // Number of valid samples in history
    bool initialized;               // First measurement flag
    uint32_t lastUpdateTime;        // Timestamp of last update
    float lastConfidence;           // Previous confidence score
    
    SmoothingState() : smoothedFrequency(0), smoothedCents(0), frequencyIndex(0),
                      validSamples(0), initialized(false), lastUpdateTime(0),
                      lastConfidence(0) {
        memset(recentFrequencies, 0, sizeof(recentFrequencies));
    }
    
    // Reset smoothing state
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

// Display state management
struct DisplayState {
    char lastNoteName[8];
    float lastCentsOffset;
    uint32_t lastUpdateTime;
    bool needsFullRedraw;
    int lastDynamicRadius;
    uint32_t lastDynamicColor;
    bool staticCirclesDrawn;
    SmoothingState smoothing;       // Temporal smoothing state
    PowerState powerState;          // Current power management state
    bool showingDetectingMode;      // Display state tracking
};

// Performance monitoring
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

#endif // CONFIG_H