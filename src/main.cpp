#include <Arduino.h>
#include <cmath>
#include <driver/i2s.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// timing debug control - enable for detailed timing output
#define ENABLE_TIMING_DEBUG 1

// gc9a01 display spi pin definitions
// esp32-s3 to round display connection mapping
#define TFT_SCK     1   // spi clock signal
#define TFT_MOSI    2   // spi data output (mosi)
#define TFT_CS      5   // chip select (active low)
#define TFT_DC      4   // data/command control
#define TFT_RST     3   // hardware reset
#define TFT_BLK     6   // backlight control

// audio buffer structure with heap-allocated sample storage
// heap allocation required for large audio buffers (2048+ samples)
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
    // heap preferred over stack for large arrays due to stack limitations
    bool init(uint16_t size = 2048) {
        if (samples) free(samples); // cleanup existing allocation
        samples = (float*)malloc(size * sizeof(float));
        if (samples) {
            sampleCount = size;
            isValid = true;
            return true;
        }
        return false;
    }
    
    // release allocated memory
    // prevents memory leaks
    void cleanup() {
        if (samples) {
            free(samples);
            samples = nullptr;
        }
        isValid = false;
    }
    
    // validate buffer integrity
    // prevents crashes from corrupted data
    bool validate() const {
        return isValid && samples != nullptr && sampleCount > 0 && sampleCount <= 2048;
    }
};

// frequency analysis result structure
// compact structure for efficient copying
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
    
    // constructor with safe defaults
    TuningResult() : frequency(0), centsOffset(0), captureTime(0), 
                    processStartTime(0), acStartTime(0), acEndTime(0),
                    yinStartTime(0), yinEndTime(0), displayStartTime(0),
                    displayEndTime(0), bufferID(0), isValid(false) {
        memset(noteName, 0, sizeof(noteName));
    }
    
    // validate result integrity
    // ensures frequency within audible range
    bool validate() const {
        return isValid && frequency >= 20.0f && frequency <= 20000.0f;
    }
};

// audio processing and analysis function declarations
bool captureRealAudio(AudioBuffer* buffer);
int findCoarsePeriodYIN(const AudioBuffer* input, TuningResult* result);
bool yinAnalysis(const AudioBuffer* input, TuningResult* output, int hintedPeriod);
void convertFrequencyToNote(float frequency, char* noteName, size_t nameSize);
float calculateCentsOffset(float frequency);
void displayResult(const TuningResult* result, const AudioBuffer* buffer);

// display function declarations
void initDisplay();
void drawTunerInterface();
void drawStaticTunerElements();
void updateTunerDisplay(const char* note, float cents, TuningResult* result);
void drawOptimizedCentsCircle(float cents, uint32_t circleColor);

// task function declarations
void audioTask(void* parameter);
void processingAndDisplayTask(void* parameter);

// utility function declarations
float calculateTimingMs(uint64_t currentTime, uint64_t captureTime);
void printTiming(const char* stage, uint32_t bufferID, uint64_t currentTime, uint64_t captureTime);
void printTimingSummary(const TuningResult* result, const AudioBuffer* buffer);
bool safePrint(const char* message);
bool safePrintf(const char* format, ...);
uint32_t getNextBufferID();
void updateStats(uint64_t latency);
bool initI2S();

// calculate milliseconds elapsed from capture start
// converts microsecond timestamps to relative milliseconds
float calculateTimingMs(uint64_t currentTime, uint64_t captureTime) {
    if (currentTime == 0 || captureTime == 0) return 0.0f;
    return (float)(currentTime - captureTime) / 1000.0f;
}

// thread-safe timing output
// tracks specific buffers through processing pipeline
void printTiming(const char* stage, uint32_t bufferID, uint64_t currentTime, uint64_t captureTime) {
    #if ENABLE_TIMING_DEBUG
    if (!stage || currentTime == 0 || captureTime == 0) return;
    float timingMs = calculateTimingMs(currentTime, captureTime);
    safePrintf("timing: buffer %lu - %s = %.2f ms\n", bufferID, stage, timingMs);
    #endif
}

// comprehensive pipeline timing analysis
// displays total latency and stage breakdown
void printTimingSummary(const TuningResult* result, const AudioBuffer* buffer) {
    #if ENABLE_TIMING_DEBUG
    if (!result || !buffer || !result->validate()) return;
    
    float captureMs = calculateTimingMs(buffer->captureEndTime, buffer->captureTime);
    float queueMs = calculateTimingMs(buffer->queueReceiveTime, buffer->queueSendTime);
    float yin1Ms = calculateTimingMs(result->acEndTime, result->acStartTime); 
    float yinMs = calculateTimingMs(result->yinEndTime, result->yinStartTime);
    float displayMs = calculateTimingMs(result->displayEndTime, result->displayStartTime);
    float totalMs = calculateTimingMs(result->displayEndTime, buffer->captureTime);
    
    safePrintf("=== TIMING SUMMARY buffer %lu ===\n", buffer->bufferID);
    safePrintf("capture: %.2f ms | queue: %.2f ms | yin coarse: %.2f ms | yin fine: %.2f ms | display: %.2f ms | total: %.2f ms\n",
              captureMs, queueMs, yin1Ms, yinMs, displayMs, totalMs);
    safePrintf("note: %s | cents: %.1f | freq: %.1f hz\n", 
              result->noteName, result->centsOffset, result->frequency);
    safePrintf("======================\n");
    #endif
}

// lovyangfx configuration for gc9a01 display
// configures spi communication and panel parameters
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Bus_SPI _bus_instance;

public:
  LGFX(void)
  {
    // spi bus configuration for display communication
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;              // spi2 peripheral
      cfg.spi_mode = 0;                      // mode 0 (cpol=0, cpha=0)
      cfg.freq_write = 40000000;             // 40mhz write speed
      cfg.freq_read = 16000000;              // 16mhz read speed
      cfg.spi_3wire = true;                  // 3-wire spi mode
      cfg.use_lock = true;                   // thread-safe access
      cfg.dma_channel = SPI_DMA_CH_AUTO;     // automatic dma channel
      cfg.pin_sclk = TFT_SCK;                // clock pin
      cfg.pin_mosi = TFT_MOSI;               // data output pin
      cfg.pin_miso = -1;                     // not used
      cfg.pin_dc = TFT_DC;                   // data/command pin
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    // gc9a01 panel configuration
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = TFT_CS;                   // chip select
      cfg.pin_rst = TFT_RST;                 // reset pin
      cfg.pin_busy = -1;                     // not used
      cfg.panel_width = 240;                 // 240x240 pixel display
      cfg.panel_height = 240;
      cfg.offset_x = 0;                      // no x offset
      cfg.offset_y = 0;                      // no y offset
      cfg.offset_rotation = 0;               // no rotation offset
      cfg.dummy_read_pixel = 8;              // spi timing
      cfg.dummy_read_bits = 1;               // spi timing
      cfg.readable = false;                  // write-only display
      cfg.invert = true;                     // gc9a01 requires inversion
      cfg.rgb_order = false;                 // bgr color order
      cfg.dlen_16bit = false;                // 8-bit data length
      cfg.bus_shared = false;                // dedicated spi bus
      _panel_instance.config(cfg);
    }

    setPanel(&_panel_instance);
  }
};

// global display instance
LGFX tft;

// display state tracking for efficient updates
// prevents flickering and tracks current display content
struct DisplayState {
    char lastNoteName[8];        // previously displayed note
    float lastCentsOffset;       // previously displayed cents
    uint32_t lastUpdateTime;     // last update timestamp
    bool needsFullRedraw;        // full refresh flag
    int lastDynamicRadius;       // previous circle radius for efficient clearing
    uint32_t lastDynamicColor;   // previous circle color
    bool staticCirclesDrawn;     // static elements drawn flag
} displayState = {"", 0.0f, 0, true, 0, 0, false};

// i2s configuration for inmp441 microphone
// esp32-s3 i2s_num_0 peripheral
#define I2S_PORT          I2S_NUM_0
#define I2S_SAMPLE_RATE   48000
#define I2S_SAMPLE_BITS   I2S_BITS_PER_SAMPLE_32BIT
#define I2S_CHANNELS      I2S_CHANNEL_MONO
#define I2S_DMA_BUF_COUNT 32        // dma buffer count
#define I2S_DMA_BUF_LEN   64        // samples per dma buffer
// calibration: 415.305 hz reads as -7 cents
// detected frequency = 415.305 * 2^(-7/1200) = 413.88 hz
// correction factor = 415.305 / 413.88 = 1.0034

#define AUDIO_CALIBRATION_FACTOR 1.0000f  // empirically determined

// inmp441 gpio pin assignments
#define I2S_SCK_PIN       13       // serial clock (bit clock)
#define I2S_WS_PIN        12       // word select (left/right clock)
#define I2S_SD_PIN        11       // serial data input

// yin algorithm parameters for second pass
#define YIN_THRESHOLD         0.15f    // confidence threshold
#define YIN_SEARCH_WINDOW     0.40f    // ±20% search window around hint

// thread-safe global variables for multi-core communication
// freertos handles for queues, tasks, and mutexes

// serial output mutex prevents interleaved output from multiple cores
SemaphoreHandle_t serialMutex = nullptr;

// display mutex prevents corruption during concurrent updates
SemaphoreHandle_t displayMutex = nullptr;

// inter-core communication queue
// transfers audiobuffer pointers from core 0 to core 1
QueueHandle_t audioQueue = nullptr;      // queue of audiobuffer pointers

// task handles for monitoring and control
TaskHandle_t audioTaskHandle = nullptr;
TaskHandle_t processingTaskHandle = nullptr;

// atomic counters for thread-safe statistics
// hardware-guaranteed atomic operations without mutex overhead
volatile uint32_t bufferCounter = 0;  // total buffers created
volatile uint32_t processedCount = 0; // successfully processed buffers
volatile uint32_t droppedCount = 0;   // dropped buffers (queue overflow)

// performance statistics with mutex protection
// mutex required for multi-field updates
struct PerformanceStats {
    uint32_t totalProcessed;       // successful operations
    uint32_t totalDropped;         // failed/dropped operations
    uint64_t totalLatency;         // cumulative latency for averaging
    uint64_t minLatency;           // minimum processing time
    uint64_t maxLatency;           // maximum processing time
    bool initialized;              // initialization flag
    
    // constructor with safe defaults
    PerformanceStats() : totalProcessed(0), totalDropped(0), totalLatency(0),
                        minLatency(UINT64_MAX), maxLatency(0), initialized(true) {}
} stats;

SemaphoreHandle_t statsMutex = nullptr;

// thread-safe utility functions
// prevent concurrent serial access conflicts

// safely print string with mutex protection
// timeout prevents deadlock
bool safePrint(const char* message) {
    if (!serialMutex || !message) return false;
    
    // acquire exclusive serial access
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.print(message);
        Serial.flush(); // ensure uart transmission
        xSemaphoreGive(serialMutex);
        return true;
    }
    return false;
}

// thread-safe printf with buffer overflow protection
// vsnprintf prevents buffer overrun
bool safePrintf(const char* format, ...) {
    if (!serialMutex || !format) return false;
    
    char buffer[256]; // fixed size prevents heap fragmentation
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // verify formatting success and buffer fit
    if (len > 0 && len < sizeof(buffer)) {
        return safePrint(buffer);
    }
    return false;
}

// atomic buffer id increment
// prevents race conditions between cores
uint32_t getNextBufferID() {
    return __atomic_fetch_add(&bufferCounter, 1, __ATOMIC_SEQ_CST);
}

// thread-safe statistics update
// mutex protects multi-field modifications
void updateStats(uint64_t latency) {
    if (!statsMutex) return;
    
    // short timeout prevents audio blocking
    if (xSemaphoreTake(statsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        stats.totalProcessed++;
        stats.totalLatency += latency;
        if (latency < stats.minLatency) stats.minLatency = latency;
        if (latency > stats.maxLatency) stats.maxLatency = latency;
        xSemaphoreGive(statsMutex);
    }
}

// initialize display hardware and interface
// configures lovyangfx, enables backlight, draws ui
void initDisplay() {
    // enable backlight for immediate visual feedback
    pinMode(TFT_BLK, OUTPUT);
    digitalWrite(TFT_BLK, HIGH);
    safePrint("display backlight enabled\n");
    
    // initialize lovyangfx with gc9a01 configuration
    tft.init();
    tft.setRotation(0);  // default orientation
    
    // display test sequence for verification
    tft.fillScreen(TFT_RED);
    delay(200);
    tft.fillScreen(TFT_GREEN);
    delay(200);
    tft.fillScreen(TFT_BLUE);
    delay(200);
    tft.fillScreen(TFT_BLACK);
    
    safePrint("display initialized successfully\n");
    
    // render initial tuner interface
    drawTunerInterface();
}

// draw static tuner interface elements
// creates three concentric reference circles for ±10 cents
void drawStaticTunerElements() {
    if (displayState.staticCirclesDrawn) return; // skip if already drawn
    
    // display center coordinates
    int centerX = 120;
    int centerY = 120;
    
    // reference radii for ±10 cents system
    const int INNER_RADIUS = 80;   // -10 cents
    const int MIDDLE_RADIUS = 100; // 0 cents (perfect pitch)  
    const int OUTER_RADIUS = 120;  // +10 cents
    
    // draw three static reference circles
    
    // outer circle (+10 cents)
    tft.drawCircle(centerX, centerY, OUTER_RADIUS, TFT_DARKGREY);
    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREY);
    tft.drawCenterString("+10c", centerX, centerY - OUTER_RADIUS + 10);
    
    // middle circle (perfect pitch)
    tft.drawCircle(centerX, centerY, MIDDLE_RADIUS, TFT_WHITE);
    tft.setTextColor(TFT_WHITE);
    tft.drawCenterString("0c", centerX, centerY - MIDDLE_RADIUS + 10);
    
    // inner circle (-10 cents)
    tft.drawCircle(centerX, centerY, INNER_RADIUS, TFT_DARKGREY);
    tft.setTextColor(TFT_DARKGREY);
    tft.drawCenterString("-10c", centerX, centerY - INNER_RADIUS + 10);
    
    displayState.staticCirclesDrawn = true;
}

// efficiently draw dynamic 2-pixel thick circle based on cents
// linear mapping: -10c=80px, 0c=100px, +10c=120px
void drawOptimizedCentsCircle(float cents, uint32_t circleColor) {
    // display center coordinates
    int centerX = 120;
    int centerY = 120;
    
    // reference radii matching static circles
    const int INNER_RADIUS = 80;   // -10 cents
    const int MIDDLE_RADIUS = 100; // 0 cents
    const int OUTER_RADIUS = 120;  // +10 cents
    
    // calculate radius using linear mapping
    // formula: radius = middle_radius + (cents * 2.0f)
    // yields: -10c→80px, 0c→100px, +10c→120px
    int newRadius = MIDDLE_RADIUS + (int)(cents * 2.0f);
    
    // handle extreme values beyond ±10 cents
    if (cents < -10.0f) {
        // very flat notes: shrink below inner circle
        newRadius = INNER_RADIUS - (int)((fabs(cents) - 10.0f) * 1.0f);
        if (newRadius < 60) newRadius = 60; // minimum radius
    } else if (cents > 10.0f) {
        // very sharp notes: expand beyond outer circle
        newRadius = OUTER_RADIUS + (int)((cents - 10.0f) * 1.0f);
        if (newRadius > 140) newRadius = 140; // maximum radius
    }
    
    // erase previous dynamic circle (2-pixel thickness)
    if (displayState.lastDynamicRadius > 0 && 
        displayState.lastDynamicRadius != newRadius) {
        // clear with black
        tft.drawCircle(centerX, centerY, displayState.lastDynamicRadius, TFT_BLACK);
        tft.drawCircle(centerX, centerY, displayState.lastDynamicRadius + 1, TFT_BLACK);
        
        // redraw static elements to prevent text corruption
        // black circle may overlap text labels at various radii
        
        // redraw all three static circles
        tft.drawCircle(centerX, centerY, OUTER_RADIUS, TFT_DARKGREY);
        tft.drawCircle(centerX, centerY, MIDDLE_RADIUS, TFT_WHITE);
        tft.drawCircle(centerX, centerY, INNER_RADIUS, TFT_DARKGREY);
        
        // redraw text labels with appropriate colors
        tft.setTextSize(1);
        
        tft.setTextColor(TFT_DARKGREY);
        tft.drawCenterString("+10c", centerX, centerY - OUTER_RADIUS + 10);
        
        tft.setTextColor(TFT_WHITE);
        tft.drawCenterString("0c", centerX, centerY - MIDDLE_RADIUS + 10);
        
        tft.setTextColor(TFT_DARKGREY);
        tft.drawCenterString("-10c", centerX, centerY - INNER_RADIUS + 10);
    }
    
    // draw new dynamic circle (2-pixel thickness)
    if (newRadius != displayState.lastDynamicRadius || 
        circleColor != displayState.lastDynamicColor) {
        
        // create 2-pixel thickness with two circles
        tft.drawCircle(centerX, centerY, newRadius, circleColor);
        tft.drawCircle(centerX, centerY, newRadius + 1, circleColor);
        
        // update state tracking
        displayState.lastDynamicRadius = newRadius;
        displayState.lastDynamicColor = circleColor;
    }
}

// draw complete interface with static and dynamic elements
// use only for full redraws (startup, major changes)
void drawTunerInterface() {
    // clear display
    tft.fillScreen(TFT_BLACK);
    
    // reset state for full redraw
    displayState.staticCirclesDrawn = false;
    displayState.lastDynamicRadius = 0;
    displayState.needsFullRedraw = false;
    
    // render static elements
    drawStaticTunerElements();
}

// update dynamic tuner display elements efficiently
// redraws only changed elements to minimize latency
void updateTunerDisplay(const char* note, float cents, TuningResult* result) {
    if (!displayMutex) return;

    // record display timing start
    if (result) {
        result->displayStartTime = esp_timer_get_time();
        printTiming("display start", result->bufferID, result->displayStartTime, result->captureTime);
    }

    // acquire display mutex
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        
        // ensure static elements exist
        drawStaticTunerElements();
        
        // determine color based on pitch accuracy
        uint32_t noteColor;
        if (fabs(cents) < 10.0f) {
            noteColor = TFT_GREEN;       // excellent pitch
        } else if (fabs(cents) > 10.0f && fabs(cents) < 13.0f) {
            noteColor = TFT_GREENYELLOW; // good pitch
        } else if (fabs(cents) > 13.0f && fabs(cents) < 15.0f) {
            noteColor = TFT_YELLOW;      // marginal pitch
        } else if (fabs(cents) > 15.0f && fabs(cents) < 18.0f) {
            noteColor = TFT_ORANGE;      // poor pitch
        } else {
            noteColor = TFT_RED;         // very poor pitch
        }

        // update text only if significantly changed
        if (strcmp(note, displayState.lastNoteName) != 0 || 
            fabs(cents - displayState.lastCentsOffset) > 1.0f) {
            
            // clear text area efficiently
            tft.fillRect(60, 40, 120, 100, TFT_BLACK);
            
            // draw cents deviation (smaller for speed)
            tft.setTextColor(noteColor);
            tft.setTextSize(2);
            char centsStr[8];
            snprintf(centsStr, sizeof(centsStr), "%.0f", cents); // integer for speed
            tft.drawCenterString(centsStr, 120, 60);

            // draw note name 
            tft.setTextSize(4); // optimized size
            tft.drawCenterString(note, 120, 90);
            
            // update state tracking
            strcpy(displayState.lastNoteName, note);
            displayState.lastCentsOffset = cents;
        }

        // update dynamic circle
        drawOptimizedCentsCircle(cents, noteColor);

        xSemaphoreGive(displayMutex);
        
        // record display timing end
        if (result) {
            result->displayEndTime = esp_timer_get_time();
            printTiming("display end", result->bufferID, result->displayEndTime, result->captureTime);
        }
    }
}

// initialize i2s peripheral for inmp441 microphone
// configures esp32-s3 i2s0 for audio input
bool initI2S() {
    // i2s configuration structure
    // defines communication parameters for inmp441
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),  // master receive mode
        .sample_rate = I2S_SAMPLE_RATE,                       // 48khz sampling
        .bits_per_sample = I2S_SAMPLE_BITS,                   // 32-bit samples
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,          // mono (left channel)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,    // standard i2s protocol
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,            // interrupt priority
        .dma_buf_count = I2S_DMA_BUF_COUNT,                  // dma buffer count
        .dma_buf_len = I2S_DMA_BUF_LEN,                      // dma buffer length
        .use_apll = false,                                    // pll clock (stable for 48khz)
        .tx_desc_auto_clear = false,                          // not applicable for rx
        .fixed_mclk = 0                                       // auto-calculate mclk
    };
    
    // gpio pin configuration
    // maps esp32 pins to inmp441
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,     // bit clock (gpio 13)
        .ws_io_num = I2S_WS_PIN,       // word select (gpio 12)  
        .data_out_num = I2S_PIN_NO_CHANGE,  // not used (rx only)
        .data_in_num = I2S_SD_PIN      // data input (gpio 11)
    };
    
    // install i2s driver
    esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (result != ESP_OK) {
        safePrintf("ERROR: i2s_driver_install failed: %s\n", esp_err_to_name(result));
        return false;
    }
    
    // configure gpio pins
    result = i2s_set_pin(I2S_PORT, &pin_config);
    if (result != ESP_OK) {
        safePrintf("ERROR: i2s_set_pin failed: %s\n", esp_err_to_name(result));
        i2s_driver_uninstall(I2S_PORT);
        return false;
    }
    
    // clear dma buffers
    // prevents stale data in first capture
    i2s_zero_dma_buffer(I2S_PORT);
    
    safePrint("I2S initialized successfully\n");
    return true;
}

// capture audio from inmp441 via i2s dma
// fills buffer with normalized float samples
bool captureRealAudio(AudioBuffer* buffer) {
    if (!buffer || !buffer->validate()) {
        safePrint("ERROR: Invalid buffer in captureRealAudio\n");
        return false;
    }
    
    // record capture start time
    buffer->captureTime = esp_timer_get_time();
    printTiming("capture start", buffer->bufferID, buffer->captureTime, buffer->captureTime);
    
    // allocate temporary buffer for raw i2s data
    // inmp441 outputs 24-bit data in 32-bit containers
    int32_t* rawSamples = (int32_t*)malloc(buffer->sampleCount * sizeof(int32_t));
    if (!rawSamples) {
        safePrint("ERROR: Failed to allocate raw sample buffer\n");
        return false;
    }
    
    // read from i2s dma buffers
    size_t bytesRead = 0;
    esp_err_t result = i2s_read(I2S_PORT, rawSamples, 
                               buffer->sampleCount * sizeof(int32_t), 
                               &bytesRead, pdMS_TO_TICKS(100));
    
    if (result != ESP_OK) {
        safePrintf("ERROR: i2s_read failed: %s\n", esp_err_to_name(result));
        free(rawSamples);
        return false;
    }
    
    // verify expected data received
    uint16_t samplesReceived = bytesRead / sizeof(int32_t);
    if (samplesReceived != buffer->sampleCount) {
        safePrintf("WARNING: Expected %d samples, got %d\n", 
                  buffer->sampleCount, samplesReceived);
        // continue with available data
        buffer->sampleCount = samplesReceived;
    }
    
    // convert 32-bit integers to normalized floats
    // inmp441 provides 24-bit data left-aligned in 32-bit words
    float maxAmplitude = 0.0f;
    float sumSquares = 0.0f;
    
    for (uint16_t i = 0; i < buffer->sampleCount; i++) {
        // extract 24-bit value from 32-bit container
        // right shift by 8 for actual 24-bit data
        int32_t sample24 = rawSamples[i] >> 8;
        
        // normalize to [-1.0, +1.0] range
        // divide by 24-bit maximum (8388608 = 2^23)
        float gain = 3.0f;
        buffer->samples[i] = (float)sample24 / 8388608.0f * gain;
        
        // track peak amplitude
        float absValue = fabsf(buffer->samples[i]);
        if (absValue > maxAmplitude) {
            maxAmplitude = absValue;
        }
        
        // accumulate for rms
        sumSquares += buffer->samples[i] * buffer->samples[i];
    }
    
    // store audio metrics
    buffer->amplitude = maxAmplitude;
    buffer->rmsLevel = sqrtf(sumSquares / buffer->sampleCount);
    
    // record capture completion
    buffer->captureEndTime = esp_timer_get_time();
    printTiming("capture end", buffer->bufferID, buffer->captureEndTime, buffer->captureTime);
    
    // cleanup temporary buffer
    free(rawSamples);
    
    // periodic debug output to minimize serial overhead
    static uint32_t debugCounter = 0;
    if ((debugCounter++ % 16) == 0) {  // every 16th buffer (~1 second)
        safePrintf("Audio: Buffer %lu, RMS=%.3f\n", buffer->bufferID, buffer->rmsLevel);
    }
    
    return true;
}

// first pass: coarse yin for rapid period estimation
int findCoarsePeriodYIN(const AudioBuffer* input, TuningResult* result) {
    if (!input || !result || !input->validate() || input->rmsLevel < 0.01f) {
        return 0; // invalid or too quiet
    }

    // record coarse yin timing start
    result->acStartTime = esp_timer_get_time(); // reusing ac timing fields
    printTiming("coarse yin start", result->bufferID, result->acStartTime, result->captureTime);

    // search parameters matching fine yin range
    int MIN_YIN_PERIOD = 32;   // ~1500hz at 48khz
    int MAX_YIN_PERIOD = 600;  // ~80hz at 48khz
    
    // speed optimizations for first pass
    const float COARSE_THRESHOLD = 0.25f;  // relaxed threshold
    const int COARSE_SAMPLES = 512;        // limited samples for speed
    
    // determine sample count (limited for performance)
    int samplesToUse = (input->sampleCount < COARSE_SAMPLES) ? input->sampleCount : COARSE_SAMPLES;
    
    // allocate difference buffer on stack
    float yinBuffer[MAX_YIN_PERIOD + 1];
    memset(yinBuffer, 0, sizeof(yinBuffer));
    
    // step 1: calculate difference function d(tau)
    for (int tau = MIN_YIN_PERIOD; tau <= MAX_YIN_PERIOD; tau++) {
        float diff = 0.0f;
        int validSamples = samplesToUse - tau;
        
        // skip insufficient samples
        if (validSamples < 64) continue;
        
        // calculate squared difference for lag
        for (int i = 0; i < validSamples; i++) {
            float delta = input->samples[i] - input->samples[i + tau];
            diff += delta * delta;
        }
        yinBuffer[tau] = diff;
    }
    
    // step 2: cumulative mean normalized difference d'(tau)
    float runningSum = 0.0f;
    yinBuffer[MIN_YIN_PERIOD] = 1.0f; // d'(0) equivalent
    
    for (int tau = MIN_YIN_PERIOD + 1; tau <= MAX_YIN_PERIOD; tau++) {
        runningSum += yinBuffer[tau];
        if (runningSum > 0.0f) {
            yinBuffer[tau] *= tau / runningSum;
        } else {
            yinBuffer[tau] = 1.0f;
        }
    }
    
    // step 3: find first dip below threshold
    int bestPeriod = 0;
    float bestValue = 1.0f;
    
    for (int tau = MIN_YIN_PERIOD; tau <= MAX_YIN_PERIOD; tau++) {
        if (yinBuffer[tau] < COARSE_THRESHOLD) {
            bestPeriod = tau;
            break; // early termination for speed
        }
        
        // track overall minimum if no threshold crossing
        if (yinBuffer[tau] < bestValue) {
            bestValue = yinBuffer[tau];
            bestPeriod = tau;
        }
    }
    
    // record coarse yin timing end
    result->acEndTime = esp_timer_get_time();
    printTiming("coarse yin end", result->bufferID, result->acEndTime, result->captureTime);
    
    // validate result
    if (bestPeriod == 0 || bestValue > 0.5f) {
        return 0;
    }
    
    // return coarse period (no interpolation for speed)
    return bestPeriod;
}

// second pass: refined yin pitch detection
// uses hinted period to constrain search
bool yinAnalysis(const AudioBuffer* input, TuningResult* output, int hintedPeriod) {
    if (!input || !output || !input->validate() || hintedPeriod == 0) {
        return false;
    }

    // record yin timing start
    output->yinStartTime = esp_timer_get_time();
    printTiming("yin start", output->bufferID, output->yinStartTime, output->captureTime);

    output->bufferID = input->bufferID;
    output->captureTime = input->captureTime;
    
    // define search range from hint
    int MIN_YIN_PERIOD = 32;   // ~1500hz at 48khz
    int MAX_YIN_PERIOD = 600;  // ~80hz at 48khz
    
    // allocate difference buffer on stack
    float yinBuffer[MAX_YIN_PERIOD + 1];

    // calculate search window around hint
    int searchMin = hintedPeriod * (1.0f - YIN_SEARCH_WINDOW);
    int searchMax = hintedPeriod * (1.0f + YIN_SEARCH_WINDOW);

    // clamp to valid range
    if (searchMin < MIN_YIN_PERIOD) searchMin = MIN_YIN_PERIOD;
    if (searchMax > MAX_YIN_PERIOD) searchMax = MAX_YIN_PERIOD;

    // step 2: calculate difference function d(tau)
    for (int tau = searchMin; tau <= searchMax; tau++) {
        float diff = 0.0f;
        for (int i = 0; i < input->sampleCount - tau; i++) {
            float delta = input->samples[i] - input->samples[i + tau];
            diff += delta * delta;
        }
        yinBuffer[tau] = diff;
    }

    // step 3: cumulative mean normalized difference d'(tau)
    float runningSum = 0.0f;
    yinBuffer[searchMin] = 1.0f; // d'(0) = 1

    for (int tau = searchMin + 1; tau <= searchMax; tau++) {
        runningSum += yinBuffer[tau];
        if (runningSum > 0.0f) {
            yinBuffer[tau] *= tau / runningSum;
        } else {
            yinBuffer[tau] = 1.0f;
        }
    }

    // step 4: find first dip below threshold
    int period = 0;
    for (int tau = searchMin; tau <= searchMax; tau++) {
        if (yinBuffer[tau] < YIN_THRESHOLD) {
            period = tau;
            break;
        }
    }
    
    // fail if no dip found
    if (period == 0) {
        return false;
    }

    // step 5: parabolic interpolation for accuracy
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
    
    // calculate final results
    output->frequency = ((float)I2S_SAMPLE_RATE / betterPeriod) * AUDIO_CALIBRATION_FACTOR;
    convertFrequencyToNote(output->frequency, output->noteName, sizeof(output->noteName));
    output->centsOffset = calculateCentsOffset(output->frequency);
    output->isValid = true;

    // record yin timing end
    output->yinEndTime = esp_timer_get_time();
    printTiming("yin end", output->bufferID, output->yinEndTime, output->captureTime);

    return output->validate();
}

// convert frequency to musical note name
// maps frequency to closest semitone
void convertFrequencyToNote(float frequency, char* noteName, size_t nameSize) {
    if (!noteName || nameSize < 4) return;
    
    // chromatic scale note names
    const char* noteNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    
    // calculate semitones from a4 (440hz)
    // formula: semitones = 12 * log2(freq / 440)
    float semitonesFromA4 = 12.0f * log2f(frequency / 440.0f);
    int totalSemitones = (int)roundf(semitonesFromA4);
    
    // determine octave (a4 = octave 4)
    int octave = 4 + (totalSemitones + 9) / 12;  // +9: a is 9 semitones from c
    if (totalSemitones + 9 < 0) octave--;  // handle negative correctly
    
    // determine note within octave
    int noteIndex = ((totalSemitones + 9) % 12 + 12) % 12;  // ensure positive
    
    // format note with octave
    snprintf(noteName, nameSize, "%s%d", noteNames[noteIndex], octave);
}

// calculate cents offset from perfect pitch
// returns deviation in cents (±50)
float calculateCentsOffset(float frequency) {
    // find nearest semitone
    float semitonesFromA4 = 12.0f * log2f(frequency / 440.0f);
    float nearestSemitone = roundf(semitonesFromA4);
    
    // calculate perfect frequency
    float perfectFrequency = 440.0f * powf(2.0f, nearestSemitone / 12.0f);
    
    // calculate cents offset
    // 100 cents = 1 semitone
    float centsOffset = 1200.0f * log2f(frequency / perfectFrequency);
    
    return centsOffset;
}

// display tuning results on gc9a01
// shows frequency, note, cents offset
void displayResult(const TuningResult* result, const AudioBuffer* buffer) {
    if (!result || !result->validate()) {
        // invalid result - clear display
        if (displayState.lastNoteName[0] != '\0') {
            updateTunerDisplay("--", 0.0f, nullptr);
            displayState.lastNoteName[0] = '\0';
        }
        return;
    }
    
    // check queue depth - skip display if backlogged
    UBaseType_t queueDepth = uxQueueMessagesWaiting(audioQueue);
    bool skipDisplayUpdate = (queueDepth > 2); // skip if >2 buffers waiting
    
    // create mutable copy for timing updates
    TuningResult mutableResult = *result;
    
    // check if update needed
    bool needsUpdate = false;
    
    if (strcmp(result->noteName, displayState.lastNoteName) != 0) {
        needsUpdate = true;
    }
    
    if (fabs(result->centsOffset - displayState.lastCentsOffset) > 2.0f) { // hysteresis threshold
        needsUpdate = true;
    }
    
    // update display if needed and not skipping
    if ((needsUpdate || displayState.needsFullRedraw) && !skipDisplayUpdate) {
        updateTunerDisplay(result->noteName, result->centsOffset, &mutableResult);
        displayState.lastUpdateTime = millis();
        
        // update tracking after successful display
        strcpy(displayState.lastNoteName, result->noteName);
        displayState.lastCentsOffset = result->centsOffset;
    } else if (skipDisplayUpdate) {
        // record timing even when skipping
        mutableResult.displayStartTime = esp_timer_get_time();
        mutableResult.displayEndTime = mutableResult.displayStartTime + 1000; // 1ms placeholder
        printTiming("display skipped", mutableResult.bufferID, mutableResult.displayEndTime, mutableResult.captureTime);
    }
    
    // print timing summary
    if (buffer) {
        printTimingSummary(&mutableResult, buffer);
    }
    
    // calculate total latency
    uint64_t totalLatency = mutableResult.displayEndTime - result->captureTime;
    
    // update statistics
    updateStats(totalLatency);
}

// core 0: audio capture task
// captures audio every 64ms from inmp441 via i2s dma
void audioTask(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(100)); // system stabilization
    
    uint8_t coreID = xPortGetCoreID();
    safePrintf("Audio task started on core %d\n", coreID);
    
    TickType_t lastCapture = xTaskGetTickCount();
    TickType_t lastStats = xTaskGetTickCount();
    
    while (true) {
        TickType_t now = xTaskGetTickCount();
        
        // capture audio at ~15fps (64ms intervals)
        // balanced responsiveness and cpu usage
        if ((now - lastCapture) >= pdMS_TO_TICKS(64)) {
            // allocate buffer on heap (stack overflow prevention)
            AudioBuffer* buffer = new AudioBuffer();
            if (!buffer) {
                safePrint("ERROR: Failed to allocate audio buffer\n");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            // initialize sample memory
            if (!buffer->init(2048)) {
                safePrint("ERROR: Failed to initialize audio buffer\n");
                delete buffer;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            // assign unique tracking id
            buffer->bufferID = getNextBufferID();
            
            // capture audio from microphone
            if (!captureRealAudio(buffer)) {
                safePrint("ERROR: Failed to capture real audio\n");
                buffer->cleanup();
                delete buffer;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            // record queue send timing
            buffer->queueSendTime = esp_timer_get_time();
            printTiming("queue send", buffer->bufferID, buffer->queueSendTime, buffer->captureTime);
            
            // send buffer pointer to processing task
            // pointer transfer avoids copying 2048 floats
            if (xQueueSend(audioQueue, &buffer, 0) != pdPASS) {
                // queue full - drop oldest buffer
                AudioBuffer* oldBuffer;
                if (xQueueReceive(audioQueue, &oldBuffer, 0) == pdPASS) {
                    oldBuffer->cleanup();
                    delete oldBuffer;
                    __atomic_fetch_add(&droppedCount, 1, __ATOMIC_SEQ_CST);
                }
                
                // retry with new buffer
                if (xQueueSend(audioQueue, &buffer, 0) != pdPASS) {
                    safePrintf("ERROR: Failed to send buffer %lu\n", buffer->bufferID);
                    buffer->cleanup();
                    delete buffer;
                }
            }
            
            lastCapture = now;
        }
        
        // periodic statistics output (5 seconds)
        if ((now - lastStats) >= pdMS_TO_TICKS(5000)) {
            uint32_t processed = __atomic_load_n(&processedCount, __ATOMIC_SEQ_CST);
            uint32_t dropped = __atomic_load_n(&droppedCount, __ATOMIC_SEQ_CST);
            
            safePrintf("=== STATS === Processed: %lu, Dropped: %lu, Free heap: %lu\n",
                      processed, dropped, (uint32_t)esp_get_free_heap_size());
            
            // stack watermark for overflow detection
            UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(audioTaskHandle);
            safePrintf("Audio stack remaining: %lu bytes\n", stackRemaining * 4);
            
            lastStats = now;
        }
        
        // minimal delay prevents cpu monopolization
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// combined processing and display task
void processingAndDisplayTask(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(150)); // startup delay
    
    uint8_t coreID = xPortGetCoreID();
    safePrintf("Processing+Display task started on core %d\n", coreID);
    
    while (true) {
        AudioBuffer* inputBuffer;
        
        if (xQueueReceive(audioQueue, &inputBuffer, portMAX_DELAY) == pdPASS) {
            if (!inputBuffer || !inputBuffer->validate()) {
                safePrint("ERROR: Received invalid buffer in processing task\n");
                if (inputBuffer) {
                    inputBuffer->cleanup();
                    delete inputBuffer;
                }
                continue;
            }
            
            // record queue receive timing
            inputBuffer->queueReceiveTime = esp_timer_get_time();
            printTiming("queue receive", inputBuffer->bufferID, inputBuffer->queueReceiveTime, inputBuffer->captureTime);
            
            bool analysisSuccess = false;
            TuningResult result;
            
            // record processing start timing
            result.processStartTime = esp_timer_get_time();
            result.bufferID = inputBuffer->bufferID;
            result.captureTime = inputBuffer->captureTime;
            printTiming("process start", result.bufferID, result.processStartTime, result.captureTime);

            // pass 1: coarse period estimation with yin
            int coarsePeriod = findCoarsePeriodYIN(inputBuffer, &result);

            // pass 2: refined analysis if coarse period found
            if (coarsePeriod > 0) {
                if (yinAnalysis(inputBuffer, &result, coarsePeriod)) {
                    displayResult(&result, inputBuffer);
                    __atomic_fetch_add(&processedCount, 1, __ATOMIC_SEQ_CST);
                    analysisSuccess = true;
                }
            }
            
            // clear display on analysis failure
            if (!analysisSuccess) {
                displayResult(nullptr, inputBuffer);
            }
            
            // cleanup buffer memory
            inputBuffer->cleanup();
            delete inputBuffer;
        }
    }
}

// setup: runs once at startup
// initializes freertos objects and starts tasks
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=== ESP32-S3 Multi-Core Voice Tuner with GC9A01 Display ===");
    Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("Timing Debug: %s\n", ENABLE_TIMING_DEBUG ? "ENABLED" : "DISABLED");
    
    // initialize display for visual feedback
    initDisplay();
    
    // display startup message
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.drawCenterString("VOICE TUNER", 120, 80);
    tft.setTextSize(1);
    tft.drawCenterString("Initializing...", 120, 120);
    
    // initialize i2s audio
    if (!initI2S()) {
        Serial.println("FATAL: I2S initialization failed");
        tft.setTextColor(TFT_RED);
        tft.drawCenterString("I2S FAILED!", 120, 140);
        return;
    }
    
    // display i2s success
    tft.setTextColor(TFT_GREEN);
    tft.drawCenterString("I2S OK", 120, 140);
    delay(500);
    
    // create synchronization mutexes
    // prevent inter-core conflicts
    serialMutex = xSemaphoreCreateMutex();
    statsMutex = xSemaphoreCreateMutex();
    displayMutex = xSemaphoreCreateMutex();
    
    if (!serialMutex || !statsMutex || !displayMutex) {
        Serial.println("FATAL: Failed to create mutexes");
        tft.setTextColor(TFT_RED);
        tft.drawCenterString("MUTEX FAILED!", 120, 160);
        return;
    }
    
    Serial.println("Mutexes created successfully");
    
    // create inter-task communication queue
    // size 4 allows buffering up to 4 items
    audioQueue = xQueueCreate(4, sizeof(AudioBuffer*));
    
    if (!audioQueue) {
        Serial.println("FATAL: Failed to create audio queue");
        tft.setTextColor(TFT_RED);
        tft.drawCenterString("QUEUE FAILED!", 120, 160);
        return;
    }
    
    Serial.println("Audio queue created successfully");
    
    // create tasks with core affinity
    // core 0: audio, core 1: processing+display
    BaseType_t result1 = xTaskCreatePinnedToCore(
        audioTask, "AudioTask", 16384, NULL, 3, &audioTaskHandle, 0);
    
    BaseType_t result2 = xTaskCreatePinnedToCore(
        processingAndDisplayTask, "ProcessingTask", 16384, NULL, 2, &processingTaskHandle, 1);
    
    if (result1 != pdPASS || result2 != pdPASS) {
        Serial.println("FATAL: Failed to create tasks");
        tft.setTextColor(TFT_RED);
        tft.drawCenterString("TASK FAILED!", 120, 160);
        return;
    }
    
    Serial.println("All tasks created successfully");
    Serial.println("System starting with two-pass (YINCoarse + YINFine) analysis...\n");
    
    // final startup delay
    delay(1000);
    
    // render main tuner interface
    drawTunerInterface();
}

// main loop: minimal activity
// real work occurs in freertos tasks
void loop() {
    // prevent watchdog timeout
    vTaskDelay(pdMS_TO_TICKS(1000));
}