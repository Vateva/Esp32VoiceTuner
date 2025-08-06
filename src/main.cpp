#include <Arduino.h>
#include <cmath>
#include <driver/i2s.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// timing debug control - set to 1 to enable detailed timing output
#define ENABLE_TIMING_DEBUG 1

// display pin definitions for gc9a01
// these connect the esp32-s3 to the round display via spi
#define TFT_SCK     1   // spi clock signal
#define TFT_MOSI    2   // spi data output to display
#define TFT_CS      5   // chip select (active low)
#define TFT_DC      4   // data/command control
#define TFT_RST     3   // hardware reset
#define TFT_BLK     6   // backlight control

// audio buffer structure for heap-allocated sample data
// uses heap allocation because audio buffers are large (2048+ samples)
struct AudioBuffer {
    float* samples;                // pointer to actual audio data stored in heap memory
    uint64_t captureTime;          // timestamp when audio was recorded (in microseconds)
    uint64_t captureEndTime;       // timestamp when capture completed (in microseconds)
    uint64_t queueSendTime;        // timestamp when sent to queue (in microseconds)
    uint64_t queueReceiveTime;     // timestamp when received from queue (in microseconds)
    uint32_t bufferID;             // unique number to track this specific buffer
    uint16_t sampleCount;          // how many audio samples we have (max 2048)
    float amplitude;               // maximum amplitude detected in this buffer
    float rmsLevel;                // rms level for audio monitoring
    bool isValid;                  // flag to check if this buffer got corrupted
    
    // constructor initializes all values to safe defaults
    AudioBuffer() : samples(nullptr), captureTime(0), captureEndTime(0), 
                   queueSendTime(0), queueReceiveTime(0), bufferID(0), 
                   sampleCount(0), amplitude(0.0f), rmsLevel(0.0f), isValid(false) {}
    
    // allocate memory for audio samples on the heap
    // heap is better than stack for large arrays because stack is limited
    bool init(uint16_t size = 2048) {
        if (samples) free(samples); // clean up any existing memory first
        samples = (float*)malloc(size * sizeof(float));
        if (samples) {
            sampleCount = size;
            isValid = true;
            return true;
        }
        return false;
    }
    
    // free the memory when we're done with this buffer
    // this prevents memory leaks
    void cleanup() {
        if (samples) {
            free(samples);
            samples = nullptr;
        }
        isValid = false;
    }
    
    // check if this buffer is safe to use
    // prevents crashes from corrupted data
    bool validate() const {
        return isValid && samples != nullptr && sampleCount > 0 && sampleCount <= 2048;
    }
};

// this structure holds the results of frequency analysis
// much smaller than audiobuffer so we can copy it around safely
struct TuningResult {
    float frequency;               // the detected frequency in hertz
    char noteName[8];             // musical note name like "A4" or "C#3"
    float centsOffset;            // how many cents sharp or flat (-50 to +50)
    uint64_t captureTime;         // when the original audio was captured
    uint64_t processStartTime;    // when processing started
    uint64_t acStartTime;         // when first pass analysis started
    uint64_t acEndTime;           // when first pass analysis finished
    uint64_t yinStartTime;        // when yin analysis started
    uint64_t yinEndTime;          // when yin analysis finished
    uint64_t displayStartTime;    // when display update started
    uint64_t displayEndTime;      // when display update finished
    uint32_t bufferID;            // links back to the original audio buffer
    bool isValid;                 // corruption check flag
    
    // constructor sets everything to safe default values
    TuningResult() : frequency(0), centsOffset(0), captureTime(0), 
                    processStartTime(0), acStartTime(0), acEndTime(0),
                    yinStartTime(0), yinEndTime(0), displayStartTime(0),
                    displayEndTime(0), bufferID(0), isValid(false) {
        memset(noteName, 0, sizeof(noteName));
    }
    
    // check if the results make sense
    // frequency should be in audible range
    bool validate() const {
        return isValid && frequency >= 20.0f && frequency <= 20000.0f;
    }
};

// function declarations for audio processing and analysis
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
void drawCentsCircleMeter(float cents); // legacy function

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



// timing helper function to calculate milliseconds from capture start
// converts microsecond timestamps to milliseconds relative to capture time
float calculateTimingMs(uint64_t currentTime, uint64_t captureTime) {
    if (currentTime == 0 || captureTime == 0) return 0.0f;
    return (float)(currentTime - captureTime) / 1000.0f;
}

// thread-safe timing output function
// prints timing information with buffer id for tracking specific buffers through pipeline
void printTiming(const char* stage, uint32_t bufferID, uint64_t currentTime, uint64_t captureTime) {
    #if ENABLE_TIMING_DEBUG
    if (!stage || currentTime == 0 || captureTime == 0) return;
    float timingMs = calculateTimingMs(currentTime, captureTime);
    safePrintf("timing: buffer %lu - %s = %.2f ms\n", bufferID, stage, timingMs);
    #endif
}

// comprehensive timing summary for complete pipeline analysis
// shows total time and breakdown of each major stage
void printTimingSummary(const TuningResult* result, const AudioBuffer* buffer) {
    #if ENABLE_TIMING_DEBUG
    if (!result || !buffer || !result->validate()) return;
    
    float captureMs = calculateTimingMs(buffer->captureEndTime, buffer->captureTime);
    float queueMs = calculateTimingMs(buffer->queueReceiveTime, buffer->queueSendTime);
    float hpsMs = calculateTimingMs(result->acEndTime, result->acStartTime); // reusing ac timing fields for hps
    float yinMs = calculateTimingMs(result->yinEndTime, result->yinStartTime);
    float displayMs = calculateTimingMs(result->displayEndTime, result->displayStartTime);
    float totalMs = calculateTimingMs(result->displayEndTime, buffer->captureTime);
    
    safePrintf("=== TIMING SUMMARY buffer %lu ===\n", buffer->bufferID);
    safePrintf("capture: %.2f ms | queue: %.2f ms | yin coarse: %.2f ms | yin fine: %.2f ms | display: %.2f ms | total: %.2f ms\n",
              captureMs, queueMs, hpsMs, yinMs, displayMs, totalMs);
    safePrintf("note: %s | cents: %.1f | freq: %.1f hz\n", 
              result->noteName, result->centsOffset, result->frequency);
    safePrintf("======================\n");
    #endif
}

// lovyangfx configuration class for gc9a01 display
// this configures the spi communication and panel settings
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Bus_SPI _bus_instance;

public:
  LGFX(void)
  {
    // configure spi bus for display communication
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;              // use spi2 peripheral
      cfg.spi_mode = 0;                      // spi mode 0 (cpol=0, cpha=0)
      cfg.freq_write = 40000000;             // 40mhz write speed for fast updates
      cfg.freq_read = 16000000;              // 16mhz read speed (not used much)
      cfg.spi_3wire = true;                  // 3-wire spi mode
      cfg.use_lock = true;                   // thread-safe spi access
      cfg.dma_channel = SPI_DMA_CH_AUTO;     // automatic dma channel selection
      cfg.pin_sclk = TFT_SCK;                // clock pin
      cfg.pin_mosi = TFT_MOSI;               // data out pin
      cfg.pin_miso = -1;                     // not used for display
      cfg.pin_dc = TFT_DC;                   // data/command pin
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    // configure gc9a01 panel specifics
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = TFT_CS;                   // chip select pin
      cfg.pin_rst = TFT_RST;                 // reset pin
      cfg.pin_busy = -1;                     // not used
      cfg.panel_width = 240;                 // display is 240x240 pixels
      cfg.panel_height = 240;
      cfg.offset_x = 0;                      // no x offset needed
      cfg.offset_y = 0;                      // no y offset needed
      cfg.offset_rotation = 0;               // no rotation offset
      cfg.dummy_read_pixel = 8;              // spi timing parameter
      cfg.dummy_read_bits = 1;               // spi timing parameter
      cfg.readable = false;                  // can't read from display
      cfg.invert = true;                     // gc9a01 needs color inversion
      cfg.rgb_order = false;                 // bgr color order
      cfg.dlen_16bit = false;                // 8-bit data length
      cfg.bus_shared = false;                // dedicated spi bus
      _panel_instance.config(cfg);
    }

    setPanel(&_panel_instance);
  }
};

// create global display instance
LGFX tft;

// display state variables for smooth updates
// these prevent flickering and track what's currently shown
struct DisplayState {
    char lastNoteName[8];        // previous note to detect changes
    float lastCentsOffset;       // previous cents to detect changes
    uint32_t lastUpdateTime;     // timestamp of last display update
    bool needsFullRedraw;        // flag to force complete screen refresh
    int lastDynamicRadius;       // previous dynamic circle radius for efficient erasing
    uint32_t lastDynamicColor;   // previous dynamic circle color
    bool staticCirclesDrawn;     // flag to track if static circles are already drawn
} displayState = {"", 0.0f, 0, true, 0, 0, false};

// i2s configuration for inmp441 microphone
// esp32-s3 supports multiple i2s ports, we use i2s_num_0
#define I2S_PORT          I2S_NUM_0
#define I2S_SAMPLE_RATE   48000
#define I2S_SAMPLE_BITS   I2S_BITS_PER_SAMPLE_32BIT
#define I2S_CHANNELS      I2S_CHANNEL_MONO
#define I2S_DMA_BUF_COUNT 32        // 16 buffers
#define I2S_DMA_BUF_LEN   64     // samples per dma buffer
// 415.305 Hz reads as -7 cents
// Detected frequency = 415.305 * 2^(-7/1200) = 413.88 Hz
// Correction factor = 415.305 / 413.88 = 1.0034

#define AUDIO_CALIBRATION_FACTOR 1.0000f  // based on real audio test

// gpio pin assignments for inmp441
#define I2S_SCK_PIN       13       // serial clock (bit clock)
#define I2S_WS_PIN        12       // word select (left/right clock)
#define I2S_SD_PIN        11       // serial data (audio input)


// yin algorithm parameters for second pass analysis
#define YIN_THRESHOLD         0.15f    // yin confidence threshold
#define YIN_SEARCH_WINDOW     0.40f    // search +/- 20% around the hinted period

// thread-safe global variables for multi-core communication
// freertos uses handles to reference queues, tasks, and mutexes

// mutex prevents multiple cores from writing to serial at the same time
// without this, serial output gets scrambled when cores try to print simultaneously
SemaphoreHandle_t serialMutex = nullptr;

// display mutex prevents display corruption during updates
// lovyangfx has some thread safety but we add extra protection
SemaphoreHandle_t displayMutex = nullptr;

// queues let cores send data to each other safely
// audioqueue sends audiobuffer pointers from core 0 to core 1
QueueHandle_t audioQueue = nullptr;      // holds AudioBuffer* (just pointers, not full structs)

// task handles let us monitor and control the running tasks
TaskHandle_t audioTaskHandle = nullptr;
TaskHandle_t processingTaskHandle = nullptr;

// atomic counters are thread-safe without needing mutexes
// the cpu hardware guarantees these operations won't be interrupted
volatile uint32_t bufferCounter = 0;  // total buffers created
volatile uint32_t processedCount = 0; // total buffers processed successfully
volatile uint32_t droppedCount = 0;   // total buffers dropped due to queue full

// performance statistics protected by mutex
// we need mutex here because we're updating multiple values together
struct PerformanceStats {
    uint32_t totalProcessed;       // total successful operations
    uint32_t totalDropped;         // total failed/dropped operations
    uint64_t totalLatency;         // sum of all latencies for averaging
    uint64_t minLatency;           // fastest processing time seen
    uint64_t maxLatency;           // slowest processing time seen
    bool initialized;              // sanity check flag
    
    // initialize with safe values
    PerformanceStats() : totalProcessed(0), totalDropped(0), totalLatency(0),
                        minLatency(UINT64_MAX), maxLatency(0), initialized(true) {}
} stats;

SemaphoreHandle_t statsMutex = nullptr;

// thread-safe utility functions
// these prevent crashes when multiple cores try to use serial simultaneously

// safely print a string without other cores interfering
// timeout prevents hanging if mutex is stuck
bool safePrint(const char* message) {
    if (!serialMutex || !message) return false;
    
    // try to get exclusive access to serial port
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.print(message);
        Serial.flush(); // make sure data actually goes out the uart
        xSemaphoreGive(serialMutex);
        return true;
    }
    return false;
}

// thread-safe printf with buffer overflow protection
// vsnprintf prevents writing past end of buffer even with bad format strings
bool safePrintf(const char* format, ...) {
    if (!serialMutex || !format) return false;
    
    char buffer[256]; // fixed size prevents heap fragmentation
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // only print if formatting succeeded and fits in buffer
    if (len > 0 && len < sizeof(buffer)) {
        return safePrint(buffer);
    }
    return false;
}

// atomically increment buffer counter and return new value
// atomic operations prevent race conditions between cores
uint32_t getNextBufferID() {
    return __atomic_fetch_add(&bufferCounter, 1, __ATOMIC_SEQ_CST);
}

// safely update performance statistics
// mutex protects against partial updates when multiple variables change
void updateStats(uint64_t latency) {
    if (!statsMutex) return;
    
    // short timeout prevents blocking audio processing
    if (xSemaphoreTake(statsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        stats.totalProcessed++;
        stats.totalLatency += latency;
        if (latency < stats.minLatency) stats.minLatency = latency;
        if (latency > stats.maxLatency) stats.maxLatency = latency;
        xSemaphoreGive(statsMutex);
    }
}

// initialize display hardware and draw initial interface
// sets up lovyangfx, enables backlight, and draws the tuner ui
void initDisplay() {
    // enable backlight first so user sees something happening
    pinMode(TFT_BLK, OUTPUT);
    digitalWrite(TFT_BLK, HIGH);
    safePrint("display backlight enabled\n");
    
    // initialize lovyangfx with our gc9a01 configuration
    tft.init();
    tft.setRotation(0);  // no rotation, use default orientation
    
    // test display with color flashes to verify it's working
    tft.fillScreen(TFT_RED);
    delay(200);
    tft.fillScreen(TFT_GREEN);
    delay(200);
    tft.fillScreen(TFT_BLUE);
    delay(200);
    tft.fillScreen(TFT_BLACK);
    
    safePrint("display initialized successfully\n");
    
    // draw the initial tuner interface
    drawTunerInterface();
}

// draw the static parts of the tuner interface that never change
// this creates the background layout once and marks it as drawn
void drawStaticTunerElements() {
    if (displayState.staticCirclesDrawn) return; // already drawn, skip
    
    // draw the three concentric circles for the cents meter
    int centerX = 120;
    int centerY = 120;

    // outer circle for +25 cents range
    tft.drawCircle(centerX, centerY, 110, TFT_DARKGREY);
    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREY);
    tft.drawCenterString("+25", centerX + 85, centerY - 85);

    // inner circle for -25 cents range
    tft.drawCircle(centerX, centerY, 70, TFT_DARKGREY);
    tft.drawCenterString("-25", centerX + 50, centerY - 50);
    
    // middle circle for perfect pitch
    tft.drawCircle(centerX, centerY, 90, TFT_WHITE);
    tft.setTextColor(TFT_WHITE);
    tft.drawCenterString("0", centerX + 65, centerY - 65);
    
    displayState.staticCirclesDrawn = true;
}

// draw the complete interface including static and dynamic elements
// only call this for full redraws (startup, major changes)
void drawTunerInterface() {
    // clear screen to black background
    tft.fillScreen(TFT_BLACK);
    
    // reset static elements flag to force redraw
    displayState.staticCirclesDrawn = false;
    displayState.lastDynamicRadius = 0;
    displayState.needsFullRedraw = false;
    
    // draw static elements
    drawStaticTunerElements();
}

// update the dynamic parts of the tuner display efficiently
// only redraws changed elements to minimize display time
void updateTunerDisplay(const char* note, float cents, TuningResult* result) {
    if (!displayMutex) return;

    // record timing for display start
    if (result) {
        result->displayStartTime = esp_timer_get_time();
        printTiming("display start", result->bufferID, result->displayStartTime, result->captureTime);
    }

    // try to get exclusive access to display
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        
        // ensure static elements are drawn (only draws once)
        drawStaticTunerElements();
        
        // determine color based on how close to perfect pitch
        uint32_t noteColor;
        if (fabs(cents) < 5.0f) {
            noteColor = TFT_GREEN;       // very close to perfect pitch
        } else if (fabs(cents) > 5.0f && fabs(cents) < 10.0f) {
            noteColor = TFT_GREENYELLOW; // pretty close
        } else if (fabs(cents) > 10.0f && fabs(cents) < 15.0f) {
            noteColor = TFT_YELLOW;      // getting off pitch
        } else if (fabs(cents) > 15.0f && fabs(cents) < 20.0f) {
            noteColor = TFT_ORANGE;      // noticeably off
        } else {
            noteColor = TFT_RED;         // way off pitch
        }

        // only update note/cents text if it changed significantly
        if (strcmp(note, displayState.lastNoteName) != 0 || 
            fabs(cents - displayState.lastCentsOffset) > 1.0f) {
            
            // clear only the text area efficiently
            tft.fillRect(60, 40, 120, 100, TFT_BLACK);
            
            // draw the cents deviation (smaller, less text operations)
            tft.setTextColor(noteColor);
            tft.setTextSize(2);
            char centsStr[8];
            snprintf(centsStr, sizeof(centsStr), "%.0f", cents); // no decimal for speed
            tft.drawCenterString(centsStr, 120, 60);

            // draw the note name 
            tft.setTextSize(4); // slightly smaller for speed
            tft.drawCenterString(note, 120, 90);
            
            // update tracking variables
            strcpy(displayState.lastNoteName, note);
            displayState.lastCentsOffset = cents;
        }

        // update the dynamic circle efficiently
        drawOptimizedCentsCircle(cents, noteColor);

        xSemaphoreGive(displayMutex);
        
        // record timing for display end
        if (result) {
            result->displayEndTime = esp_timer_get_time();
            printTiming("display end", result->bufferID, result->displayEndTime, result->captureTime);
        }
    }
}

// efficiently draw only the dynamic circle without touching static elements
// this is the key optimization that reduces display time dramatically
void drawOptimizedCentsCircle(float cents, uint32_t circleColor) {
    // constrain cents to reasonable range
    if (cents < -50.0f) cents = -50.0f;
    if (cents > 50.0f) cents = 50.0f;

    int centerX = 120;
    int centerY = 120;
    
    // calculate new radius based on cents value
    int newRadius;
    if (fabs(cents) <= 25.0f) {
        // map cents from -25 to +25 to radius from 70 to 110
        newRadius = 90 + (int)(cents * 0.8f);
    } else if (cents > 25.0f) {
        // high pitch, radius grows bigger than +25 circle
        newRadius = 110 + (int)((cents - 25.0f) * 0.4f);
        if (newRadius > 120) newRadius = 120;
    } else { // cents < -25.0f
        // low pitch, radius shrinks smaller than -25 circle
        newRadius = 70 + (int)((cents + 25.0f) * 0.4f);
        if (newRadius < 60) newRadius = 60;
    }

    // erase previous dynamic circle by drawing it in black (only if different)
    if (displayState.lastDynamicRadius > 0 && 
        displayState.lastDynamicRadius != newRadius) {
        tft.drawCircle(centerX, centerY, displayState.lastDynamicRadius, TFT_BLACK);
    }

    // draw new dynamic circle (only if radius changed or color changed)
    if (newRadius != displayState.lastDynamicRadius || 
        circleColor != displayState.lastDynamicColor) {
        tft.drawCircle(centerX, centerY, newRadius, circleColor);
        
        // update state tracking
        displayState.lastDynamicRadius = newRadius;
        displayState.lastDynamicColor = circleColor;
    }
}

// legacy function - now unused but kept for compatibility
// the new optimized version replaces this completely
void drawCentsCircleMeter(float cents) {
    // this function is no longer used - replaced by drawOptimizedCentsCircle()
    // kept here to avoid compilation errors, but should not be called
}

// initialize i2s peripheral for inmp441 microphone
// configures esp32-s3 i2s0 port for audio input
bool initI2S() {
    // i2s configuration structure
    // this tells the esp32 how to communicate with the inmp441
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),  // master mode, receive only
        .sample_rate = I2S_SAMPLE_RATE,                       // 16khz sample rate
        .bits_per_sample = I2S_SAMPLE_BITS,                   // 32 bits per sample
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,          // mono audio (left channel only)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,    // standard i2s protocol
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,            // interrupt priority
        .dma_buf_count = I2S_DMA_BUF_COUNT,                  // number of dma buffers (2 for double buffering)
        .dma_buf_len = I2S_DMA_BUF_LEN,                      // samples per dma buffer
        .use_apll = false,                                    // use pll clock (more stable than apll for this sample rate)
        .tx_desc_auto_clear = false,                          // not used for rx only
        .fixed_mclk = 0                                       // auto calculate master clock
    };
    
    // gpio pin configuration for i2s signals
    // connects esp32 pins to inmp441 pins
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,     // bit clock pin (gpio 13)
        .ws_io_num = I2S_WS_PIN,       // word select pin (gpio 12)  
        .data_out_num = I2S_PIN_NO_CHANGE,  // not used for input only
        .data_in_num = I2S_SD_PIN      // data input pin (gpio 11)
    };
    
    // install and start i2s driver
    esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (result != ESP_OK) {
        safePrintf("ERROR: i2s_driver_install failed: %s\n", esp_err_to_name(result));
        return false;
    }
    
    // configure gpio pins for i2s
    result = i2s_set_pin(I2S_PORT, &pin_config);
    if (result != ESP_OK) {
        safePrintf("ERROR: i2s_set_pin failed: %s\n", esp_err_to_name(result));
        i2s_driver_uninstall(I2S_PORT);
        return false;
    }
    
    // clear any existing data in dma buffers
    // prevents old data from appearing in first capture
    i2s_zero_dma_buffer(I2S_PORT);
    
    safePrint("I2S initialized successfully\n");
    return true;
}

// capture real audio from inmp441 microphone via i2s dma
// fills audiobuffer with normalized float samples from microphone
bool captureRealAudio(AudioBuffer* buffer) {
    if (!buffer || !buffer->validate()) {
        safePrint("ERROR: Invalid buffer in captureRealAudio\n");
        return false;
    }
    
    // record when we started capturing this audio
    buffer->captureTime = esp_timer_get_time();
    printTiming("capture start", buffer->bufferID, buffer->captureTime, buffer->captureTime);
    
    // temporary buffer for raw i2s data (32-bit integers)
    // inmp441 outputs 24-bit data in 32-bit containers
    int32_t* rawSamples = (int32_t*)malloc(buffer->sampleCount * sizeof(int32_t));
    if (!rawSamples) {
        safePrint("ERROR: Failed to allocate raw sample buffer\n");
        return false;
    }
    
    // read audio data from i2s dma buffers
    size_t bytesRead = 0;
    esp_err_t result = i2s_read(I2S_PORT, rawSamples, 
                               buffer->sampleCount * sizeof(int32_t), 
                               &bytesRead, pdMS_TO_TICKS(100));
    
    if (result != ESP_OK) {
        safePrintf("ERROR: i2s_read failed: %s\n", esp_err_to_name(result));
        free(rawSamples);
        return false;
    }
    
    // check if we got the expected amount of data
    uint16_t samplesReceived = bytesRead / sizeof(int32_t);
    if (samplesReceived != buffer->sampleCount) {
        safePrintf("WARNING: Expected %d samples, got %d\n", 
                  buffer->sampleCount, samplesReceived);
        // continue processing with whatever we got
        buffer->sampleCount = samplesReceived;
    }
    
    // convert raw 32-bit integers to normalized float values
    // inmp441 outputs 24-bit data left-aligned in 32-bit words
    float maxAmplitude = 0.0f;
    float sumSquares = 0.0f;
    
    for (uint16_t i = 0; i < buffer->sampleCount; i++) {
        // extract 24-bit audio data from 32-bit container
        // shift right by 8 to get the actual 24-bit value
        int32_t sample24 = rawSamples[i] >> 8;
        
        // normalize to float range [-1.0, +1.0]
        // divide by maximum 24-bit value (8388608 = 2^23)
        float gain = 3.0f;
        buffer->samples[i] = (float)sample24 / 8388608.0f * gain;
        
        // track maximum amplitude for monitoring
        float absValue = fabsf(buffer->samples[i]);
        if (absValue > maxAmplitude) {
            maxAmplitude = absValue;
        }
        
        // accumulate for rms calculation
        sumSquares += buffer->samples[i] * buffer->samples[i];
    }
    
    // store audio level information for monitoring
    buffer->amplitude = maxAmplitude;
    buffer->rmsLevel = sqrtf(sumSquares / buffer->sampleCount);
    
    // record when capture completed
    buffer->captureEndTime = esp_timer_get_time();
    printTiming("capture end", buffer->bufferID, buffer->captureEndTime, buffer->captureTime);
    
    // clean up temporary buffer
    free(rawSamples);
    
    // only print debug info occasionally to reduce serial overhead
    static uint32_t debugCounter = 0;
    if ((debugCounter++ % 16) == 0) {  // print every 16th buffer (about once per second)
        safePrintf("Audio: Buffer %lu, RMS=%.3f\n", buffer->bufferID, buffer->rmsLevel);
    }
    
    return true;
}

// first pass: coarse yin for fast period estimation
// optimized for speed rather than precision to replace hps functionality
int findCoarsePeriodYIN(const AudioBuffer* input, TuningResult* result) {
    if (!input || !result || !input->validate() || input->rmsLevel < 0.01f) {
        return 0; // return 0 if signal is invalid or too quiet
    }

    // record timing for coarse yin start
    result->acStartTime = esp_timer_get_time(); // reusing ac timing fields
    printTiming("coarse yin start", result->bufferID, result->acStartTime, result->captureTime);

    // define search parameters - same range as fine yin but optimized for speed
    int MIN_YIN_PERIOD = 13;   // ~1230hz at 16khz sample rate
    int MAX_YIN_PERIOD = 400;  // ~40hz at 16khz sample rate
    
    // speed optimizations for first pass
    const float COARSE_THRESHOLD = 0.25f;  // more relaxed threshold than fine pass
    const int COARSE_SAMPLES = 512;
    
    // determine actual sample count to use (limited for speed)
    int samplesToUse = (input->sampleCount < COARSE_SAMPLES) ? input->sampleCount : COARSE_SAMPLES;
    
    // allocate difference buffer on stack (much smaller than hps fft buffer)
    float yinBuffer[MAX_YIN_PERIOD + 1];
    memset(yinBuffer, 0, sizeof(yinBuffer));
    
    // step 1: calculate difference function d(tau) - check every tau value for accuracy
    for (int tau = MIN_YIN_PERIOD; tau <= MAX_YIN_PERIOD; tau++) {
        float diff = 0.0f;
        int validSamples = samplesToUse - tau;
        
        // skip if we don't have enough samples for this tau
        if (validSamples < 64) continue;
        
        // calculate squared difference for this lag
        for (int i = 0; i < validSamples; i++) {
            float delta = input->samples[i] - input->samples[i + tau];
            diff += delta * delta;
        }
        yinBuffer[tau] = diff;
    }
    
    // step 2: calculate cumulative mean normalized difference d'(tau)
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
    
    // step 3: find first dip below threshold (early termination for speed)
    int bestPeriod = 0;
    float bestValue = 1.0f;
    
    for (int tau = MIN_YIN_PERIOD; tau <= MAX_YIN_PERIOD; tau++) {
        if (yinBuffer[tau] < COARSE_THRESHOLD) {
            bestPeriod = tau;
            break; // early termination - take first good result for speed
        }
        
        // also track the overall best value in case we don't find any below threshold
        if (yinBuffer[tau] < bestValue) {
            bestValue = yinBuffer[tau];
            bestPeriod = tau;
        }
    }
    
    // record timing for coarse yin end
    result->acEndTime = esp_timer_get_time();
    printTiming("coarse yin end", result->bufferID, result->acEndTime, result->captureTime);
    
    // validate result - if no period found or value too high, return 0
    if (bestPeriod == 0 || bestValue > 0.5f) {
        return 0;
    }
    
    // return the coarse period estimate (no interpolation for speed)
    return bestPeriod;
}



// second pass: yin algorithm for refined pitch detection
// uses the hintedPeriod from the first pass to constrain its search
bool yinAnalysis(const AudioBuffer* input, TuningResult* output, int hintedPeriod) {
    if (!input || !output || !input->validate() || hintedPeriod == 0) {
        return false;
    }

    // record timing for yin start
    output->yinStartTime = esp_timer_get_time();
    printTiming("yin start", output->bufferID, output->yinStartTime, output->captureTime);

    output->bufferID = input->bufferID;
    output->captureTime = input->captureTime;
    
    // define yin search range based on hps hint
    int MIN_YIN_PERIOD = 13;   // ~1230hz at 16khz sample rate
    int MAX_YIN_PERIOD = 400;  // ~40hz at 16khz sample rate
    
    // yin works on a buffer of difference values. allocate on stack since it's small.
    float yinBuffer[MAX_YIN_PERIOD + 1];

    // define the search window around the hinted period
    int searchMin = hintedPeriod * (1.0f - YIN_SEARCH_WINDOW);
    int searchMax = hintedPeriod * (1.0f + YIN_SEARCH_WINDOW);

    // clamp search window to valid period range
    if (searchMin < MIN_YIN_PERIOD) searchMin = MIN_YIN_PERIOD;
    if (searchMax > MAX_YIN_PERIOD) searchMax = MAX_YIN_PERIOD;

    // step 2: calculate the difference function d(tau)
    for (int tau = searchMin; tau <= searchMax; tau++) {
        float diff = 0.0f;
        for (int i = 0; i < input->sampleCount - tau; i++) {
            float delta = input->samples[i] - input->samples[i + tau];
            diff += delta * delta;
        }
        yinBuffer[tau] = diff;
    }

    // step 3: calculate the cumulative mean normalized difference function d'(tau)
    float runningSum = 0.0f;
    yinBuffer[searchMin] = 1.0f; // d'(0) is always 1

    for (int tau = searchMin + 1; tau <= searchMax; tau++) {
        runningSum += yinBuffer[tau];
        if (runningSum > 0.0f) {
            yinBuffer[tau] *= tau / runningSum;
        } else {
            yinBuffer[tau] = 1.0f;
        }
    }

    // step 4: find the first dip below the absolute threshold
    int period = 0;
    for (int tau = searchMin; tau <= searchMax; tau++) {
        if (yinBuffer[tau] < YIN_THRESHOLD) {
            period = tau;
            break;
        }
    }
    
    // if no dip is found, analysis fails
    if (period == 0) {
        return false;
    }

    // step 5: parabolic interpolation for higher accuracy
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
    
    // final result calculation
    output->frequency = ((float)I2S_SAMPLE_RATE / betterPeriod) * AUDIO_CALIBRATION_FACTOR;
    convertFrequencyToNote(output->frequency, output->noteName, sizeof(output->noteName));
    output->centsOffset = calculateCentsOffset(output->frequency);
    output->isValid = true;

    // record timing for yin end
    output->yinEndTime = esp_timer_get_time();
    printTiming("yin end", output->bufferID, output->yinEndTime, output->captureTime);

    return output->validate();
}

// convert frequency to musical note name
// determines the closest musical note and stores it as a string
void convertFrequencyToNote(float frequency, char* noteName, size_t nameSize) {
    if (!noteName || nameSize < 4) return;
    
    // note names in chromatic scale
    const char* noteNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    
    // calculate semitones from A4 (440Hz)
    // formula: semitones = 12 * log2(freq / 440)
    float semitonesFromA4 = 12.0f * log2f(frequency / 440.0f);
    int totalSemitones = (int)roundf(semitonesFromA4);
    
    // determine octave (A4 is octave 4)
    int octave = 4 + (totalSemitones + 9) / 12;  // +9 because A is 9 semitones from C
    if (totalSemitones + 9 < 0) octave--;  // handle negative case correctly
    
    // determine note within the octave
    int noteIndex = ((totalSemitones + 9) % 12 + 12) % 12;  // ensure positive result
    
    // format note name with octave
    snprintf(noteName, nameSize, "%s%d", noteNames[noteIndex], octave);
}

// calculate cents offset from perfect pitch
// returns how many cents sharp (+) or flat (-) the frequency is
float calculateCentsOffset(float frequency) {
    // find the nearest semitone frequency
    float semitonesFromA4 = 12.0f * log2f(frequency / 440.0f);
    float nearestSemitone = roundf(semitonesFromA4);
    
    // calculate the frequency of the nearest perfect semitone
    float perfectFrequency = 440.0f * powf(2.0f, nearestSemitone / 12.0f);
    
    // calculate cents offset from perfect pitch
    // 100 cents = 1 semitone
    float centsOffset = 1200.0f * log2f(frequency / perfectFrequency);
    
    return centsOffset;
}

// display tuning results on the gc9a01 round display
// shows frequency, note name, cents offset
void displayResult(const TuningResult* result, const AudioBuffer* buffer) {
    if (!result || !result->validate()) {
        // invalid result - clear display to show no signal
        if (displayState.lastNoteName[0] != '\0') {
            updateTunerDisplay("--", 0.0f, nullptr);
            displayState.lastNoteName[0] = '\0';
        }
        return;
    }
    
    // check queue depth - skip expensive display updates if falling behind
    UBaseType_t queueDepth = uxQueueMessagesWaiting(audioQueue);
    bool skipDisplayUpdate = (queueDepth > 2); // skip if more than 2 buffers waiting
    
    // make a copy of result so we can modify timing fields
    TuningResult mutableResult = *result;
    
    // check if display needs updating
    // only update if something changed to reduce flicker
    bool needsUpdate = false;
    
    if (strcmp(result->noteName, displayState.lastNoteName) != 0) {
        needsUpdate = true;
    }
    
    if (fabs(result->centsOffset - displayState.lastCentsOffset) > 2.0f) { // increased threshold
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
        // still record timing even if we skip display for accurate measurements
        mutableResult.displayStartTime = esp_timer_get_time();
        mutableResult.displayEndTime = mutableResult.displayStartTime + 1000; // 1ms fake time
        printTiming("display skipped", mutableResult.bufferID, mutableResult.displayEndTime, mutableResult.captureTime);
    }
    
    // print comprehensive timing summary for this buffer
    if (buffer) {
        printTimingSummary(&mutableResult, buffer);
    }
    
    // calculate total latency from capture to display end
    uint64_t totalLatency = mutableResult.displayEndTime - result->captureTime;
    
    // update performance statistics
    updateStats(totalLatency);
}

// core 0: audio capture task
// this runs on core 0 and captures audio every 64ms from inmp441
// uses i2s dma for continuous audio input
void audioTask(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(100)); // wait for system to stabilize
    
    uint8_t coreID = xPortGetCoreID();
    safePrintf("Audio task started on core %d\n", coreID);
    
    TickType_t lastCapture = xTaskGetTickCount();
    TickType_t lastStats = xTaskGetTickCount();
    
    while (true) {
        TickType_t now = xTaskGetTickCount();
        
        // capture audio every 64ms (about 15fps)
        // this gives good responsiveness without overwhelming the processor
        if ((now - lastCapture) >= pdMS_TO_TICKS(64)) {
            // create new buffer on heap (stack would overflow with 1024 floats)
            AudioBuffer* buffer = new AudioBuffer();
            if (!buffer) {
                safePrint("ERROR: Failed to allocate audio buffer\n");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            // allocate memory for audio samples
            if (!buffer->init(2048)) {
                safePrint("ERROR: Failed to initialize audio buffer\n");
                delete buffer;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            // assign unique id for tracking this buffer through the system
            buffer->bufferID = getNextBufferID();
            
            // capture real audio from inmp441 microphone
            if (!captureRealAudio(buffer)) {
                safePrint("ERROR: Failed to capture real audio\n");
                buffer->cleanup();
                delete buffer;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            // record timing for queue send
            buffer->queueSendTime = esp_timer_get_time();
            printTiming("queue send", buffer->bufferID, buffer->queueSendTime, buffer->captureTime);
            
            // send buffer pointer to processing task
            // we send pointer not full struct because copying 2048 floats is slow
            if (xQueueSend(audioQueue, &buffer, 0) != pdPASS) {
                // queue is full, drop oldest buffer to make room
                AudioBuffer* oldBuffer;
                if (xQueueReceive(audioQueue, &oldBuffer, 0) == pdPASS) {
                    oldBuffer->cleanup();
                    delete oldBuffer;
                    __atomic_fetch_add(&droppedCount, 1, __ATOMIC_SEQ_CST);
                }
                
                // try to send new buffer again
                if (xQueueSend(audioQueue, &buffer, 0) != pdPASS) {
                    safePrintf("ERROR: Failed to send buffer %lu\n", buffer->bufferID);
                    buffer->cleanup();
                    delete buffer;
                }
            }
            
            lastCapture = now;
        }
        
        // print system stats every 5 seconds for monitoring
        if ((now - lastStats) >= pdMS_TO_TICKS(5000)) {
            uint32_t processed = __atomic_load_n(&processedCount, __ATOMIC_SEQ_CST);
            uint32_t dropped = __atomic_load_n(&droppedCount, __ATOMIC_SEQ_CST);
            
            safePrintf("=== STATS === Processed: %lu, Dropped: %lu, Free heap: %lu\n",
                      processed, dropped, (uint32_t)esp_get_free_heap_size());
            
            // check how much stack space is left (for debugging stack overflows)
            UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(audioTaskHandle);
            safePrintf("Audio stack remaining: %lu bytes\n", stackRemaining * 4);
            
            lastStats = now;
        }
        
        // small delay prevents task from hogging cpu completely
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// combined processing and display task with hps + yin analysis
// this task orchestrates the two-pass pitch detection using hps and yin
void processingAndDisplayTask(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(150)); // wait for system startup
    
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
            
            // record timing for queue receive
            inputBuffer->queueReceiveTime = esp_timer_get_time();
            printTiming("queue receive", inputBuffer->bufferID, inputBuffer->queueReceiveTime, inputBuffer->captureTime);
            
            bool analysisSuccess = false;
            TuningResult result;
            
            // record timing for processing start
            result.processStartTime = esp_timer_get_time();
            result.bufferID = inputBuffer->bufferID;
            result.captureTime = inputBuffer->captureTime;
            printTiming("process start", result.bufferID, result.processStartTime, result.captureTime);

            // pass 1: get a coarse period estimate using hps
            int coarsePeriod = findCoarsePeriodYIN(inputBuffer, &result);

            // if the first pass found a potential period, proceed to the second pass
            if (coarsePeriod > 0) {
                // pass 2: use yin for a refined analysis
                if (yinAnalysis(inputBuffer, &result, coarsePeriod)) {
                    displayResult(&result, inputBuffer);
                    __atomic_fetch_add(&processedCount, 1, __ATOMIC_SEQ_CST);
                    analysisSuccess = true;
                }
            }
            
            // if any part of the analysis failed, clear the display
            if (!analysisSuccess) {
                displayResult(nullptr, inputBuffer);
            }
            
            // always clean up input buffer memory
            inputBuffer->cleanup();
            delete inputBuffer;
        }
    }
}

// setup function runs once at startup
// initializes all the freertos objects and starts tasks
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=== ESP32-S3 Multi-Core Voice Tuner with GC9A01 Display ===");
    Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("Timing Debug: %s\n", ENABLE_TIMING_DEBUG ? "ENABLED" : "DISABLED");
    
    // initialize display first so user sees startup progress
    initDisplay();
    
    // show startup message on display
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.drawCenterString("VOICE TUNER", 120, 80);
    tft.setTextSize(1);
    tft.drawCenterString("Initializing...", 120, 120);
    
    // initialize i2s for audio capture
    if (!initI2S()) {
        Serial.println("FATAL: I2S initialization failed");
        tft.setTextColor(TFT_RED);
        tft.drawCenterString("I2S FAILED!", 120, 140);
        return;
    }
    
    // update display to show i2s success
    tft.setTextColor(TFT_GREEN);
    tft.drawCenterString("I2S OK", 120, 140);
    delay(500);
    
    // create mutexes before anything else
    // these prevent cores from interfering with each other
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
    // create queues for inter-task communication
    // size 4 means we can buffer up to 4 items before blocking
    audioQueue = xQueueCreate(4, sizeof(AudioBuffer*));
    
    if (!audioQueue) {
        Serial.println("FATAL: Failed to create audio queue");
        tft.setTextColor(TFT_RED);
        tft.drawCenterString("QUEUE FAILED!", 120, 160);
        return;
    }
    
    Serial.println("Audio queue created successfully");
    
    // create tasks with specific core assignments
    // audio on core 0, processing+display on core 1 for load balancing
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
    Serial.println("System starting with two-pass (HPS + YIN) analysis...\n");
    
    // final delay before showing main interface
    delay(1000);
    
    // draw the main tuner interface
    drawTunerInterface();
}

// main loop does almost nothing
// all real work happens in the freertos tasks
void loop() {
    // just prevent watchdog timer from resetting the system
    vTaskDelay(pdMS_TO_TICKS(1000));
}
