#include <Arduino.h>
#include <cmath>
#include <driver/i2s.h>

// safe data structures for audio processing
// these structures are designed to prevent memory corruption and crashes

// this structure holds audio samples and metadata
// we use heap allocation because audio buffers are large (1024+ samples)
struct AudioBuffer {
    float* samples;                // pointer to actual audio data stored in heap memory
    uint64_t captureTime;          // timestamp when audio was recorded (in microseconds)
    uint32_t bufferID;             // unique number to track this specific buffer
    uint16_t sampleCount;          // how many audio samples we have (max 1024)
    float amplitude;               // maximum amplitude detected in this buffer
    float rmsLevel;                // rms level for audio monitoring
    bool isValid;                  // flag to check if this buffer got corrupted
    
    // constructor initializes all values to safe defaults
    AudioBuffer() : samples(nullptr), captureTime(0), bufferID(0), 
                   sampleCount(0), amplitude(0.0f), rmsLevel(0.0f), isValid(false) {}
    
    // allocate memory for audio samples on the heap
    // heap is better than stack for large arrays because stack is limited
    bool init(uint16_t size = 1024) {
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
    uint64_t processTime;         // when we finished processing it
    uint32_t bufferID;            // links back to the original audio buffer
    float confidence;             // how sure we are about the result (0.0 to 1.0)
    bool isValid;                 // corruption check flag
    
    // constructor sets everything to safe default values
    TuningResult() : frequency(0), centsOffset(0), captureTime(0), 
                    processTime(0), bufferID(0), confidence(0), isValid(false) {
        memset(noteName, 0, sizeof(noteName));
    }
    
    // check if the results make sense
    // frequency should be in audible range, confidence should be 0-100%
    bool validate() const {
        return isValid && frequency >= 20.0f && frequency <= 20000.0f && 
               confidence >= 0.0f && confidence <= 1.0f;
    }
};

// i2s configuration for inmp441 microphone
// esp32-s3 supports multiple i2s ports, we use i2s_num_0
#define I2S_PORT          I2S_NUM_0
#define I2S_SAMPLE_RATE   16000
#define I2S_SAMPLE_BITS   I2S_BITS_PER_SAMPLE_32BIT
#define I2S_CHANNELS      I2S_CHANNEL_MONO
#define I2S_DMA_BUF_COUNT 2        // double buffering
#define I2S_DMA_BUF_LEN   1024     // samples per dma buffer

// gpio pin assignments for inmp441
#define I2S_SCK_PIN       13       // serial clock (bit clock)
#define I2S_WS_PIN        12       // word select (left/right clock)
#define I2S_SD_PIN        11       // serial data (audio input)

// autocorrelation algorithm parameters
#define MIN_PERIOD        73       // minimum period for ~220hz at 16khz (low note detection)
#define MAX_PERIOD        727      // maximum period for ~22hz at 16khz (very low fundamentals)
#define CORRELATION_THRESHOLD 0.3f // minimum correlation needed for confident detection

// function declarations for audio processing and analysis
bool captureRealAudio(AudioBuffer* buffer);
bool autocorrelationAnalysis(const AudioBuffer* input, TuningResult* output);
void convertFrequencyToNote(float frequency, char* noteName, size_t nameSize);
float calculateCentsOffset(float frequency);
void displayResult(const TuningResult* result);

// task function declarations
void audioTask(void* parameter);
void processingAndDisplayTask(void* parameter);

// thread-safe global variables for multi-core communication
// freertos uses handles to reference queues, tasks, and mutexes

// mutex prevents multiple cores from writing to serial at the same time
// without this, serial output gets scrambled when cores try to print simultaneously
SemaphoreHandle_t serialMutex = nullptr;

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
        // divide by maximum 24-bit value (8388607 = 2^23 - 1)
        buffer->samples[i] = (float)sample24 / 8388607.0f;
        
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
    
    // clean up temporary buffer
    free(rawSamples);
    
    // debug output: show audio levels to verify microphone is working
    // remove this section when project is completed to reduce latency
    safePrintf("Audio captured: Buffer %lu, Peak=%.3f, RMS=%.3f\n",
              buffer->bufferID, buffer->amplitude, buffer->rmsLevel);
    
    return true;
}

// autocorrelation pitch detection algorithm
// analyzes audio buffer to find fundamental frequency using autocorrelation method
bool autocorrelationAnalysis(const AudioBuffer* input, TuningResult* output) {
    if (!input || !output || !input->validate()) {
        safePrint("ERROR: Invalid input to autocorrelationAnalysis\n");
        return false;
    }
    
    // record processing start time
    uint64_t startTime = esp_timer_get_time();
    
    // copy timing information from input
    output->bufferID = input->bufferID;
    output->captureTime = input->captureTime;
    
    // check if audio signal is strong enough for analysis
    // very quiet signals often give unreliable pitch detection
    if (input->rmsLevel < 0.01f) {
        safePrintf("Signal too quiet for analysis: RMS=%.4f\n", input->rmsLevel);
        return false;
    }
    
    // autocorrelation variables
    float bestCorrelation = 0.0f;
    int bestPeriod = 0;
    
    // search for the best autocorrelation peak within period range
    // period corresponds to fundamental frequency: freq = samplerate / period
    for (int period = MIN_PERIOD; period <= MAX_PERIOD && period < input->sampleCount / 2; period++) {
        float correlation = 0.0f;
        float energy1 = 0.0f;
        float energy2 = 0.0f;
        
        // calculate autocorrelation for this period (lag)
        // compare signal with itself shifted by 'period' samples
        int numSamples = input->sampleCount - period;
        for (int i = 0; i < numSamples; i++) {
            float sample1 = input->samples[i];
            float sample2 = input->samples[i + period];
            
            correlation += sample1 * sample2;  // cross-correlation
            energy1 += sample1 * sample1;      // energy of first segment
            energy2 += sample2 * sample2;      // energy of second segment
        }
        
        // normalize correlation by the geometric mean of energies
        // this prevents loud signals from dominating correlation values
        float totalEnergy = sqrtf(energy1 * energy2);
        if (totalEnergy > 0.0f) {
            correlation /= totalEnergy;
        }
        
        // check if this is the best correlation found so far
        if (correlation > bestCorrelation) {
            bestCorrelation = correlation;
            bestPeriod = period;
        }
    }
    
    // verify that we found a strong enough correlation peak
    if (bestCorrelation < CORRELATION_THRESHOLD || bestPeriod == 0) {
        safePrintf("No strong correlation found: best=%.3f, threshold=%.3f\n", 
                  bestCorrelation, CORRELATION_THRESHOLD);
        return false;
    }
    
    // convert period to frequency
    // frequency = sample_rate / period_in_samples
    output->frequency = (float)I2S_SAMPLE_RATE / (float)bestPeriod;
    
    // calculate confidence based on correlation strength
    // stronger correlation = higher confidence in the result
    output->confidence = fminf(bestCorrelation, 1.0f);
    
    // determine musical note name from frequency
    convertFrequencyToNote(output->frequency, output->noteName, sizeof(output->noteName));
    
    // calculate cents offset from perfect pitch
    output->centsOffset = calculateCentsOffset(output->frequency);
    
    // record processing completion time
    output->processTime = esp_timer_get_time();
    
    // mark result as valid
    output->isValid = true;
    
    // debug output showing analysis results
    uint64_t processingTime = output->processTime - startTime;
    safePrintf("Autocorr: Period=%d, Freq=%.1fHz, Corr=%.3f, Time=%lluμs\n",
              bestPeriod, output->frequency, bestCorrelation, processingTime);
    
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

// display tuning results to user
// shows frequency, note name, cents offset, and confidence
void displayResult(const TuningResult* result) {
    if (!result || !result->validate()) {
        safePrint("ERROR: Invalid result in displayResult\n");
        return;
    }
    
    // calculate total latency from capture to display
    uint64_t totalLatency = result->processTime - result->captureTime;
    
    // display comprehensive tuning information
    // note: in final version this will be replaced with gc9a01 graphics
    safePrintf("♪ Buffer %lu: %s %.1fHz %+.1f cents (%.0f%% conf) lat=%lluμs\n",
              result->bufferID, result->noteName, result->frequency,
              result->centsOffset, result->confidence * 100.0f, totalLatency);
    
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
            if (!buffer->init(1024)) {
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
            
            // send buffer pointer to processing task
            // we send pointer not full struct because copying 1024 floats is slow
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

// core 1: combined processing and display task
// this runs on core 1 and does pitch detection analysis then displays results
// combining both operations eliminates communication overhead and ensures latest results

void processingAndDisplayTask(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(150)); // wait for system startup
    
    uint8_t coreID = xPortGetCoreID();
    safePrintf("Processing+Display task started on core %d\n", coreID);
    
    while (true) {
        AudioBuffer* inputBuffer;
        
        // wait for audio data from capture task
        // portmax_delay means wait forever until data arrives
        if (xQueueReceive(audioQueue, &inputBuffer, portMAX_DELAY) == pdPASS) {
            if (!inputBuffer || !inputBuffer->validate()) {
                safePrint("ERROR: Received invalid buffer in processing task\n");
                if (inputBuffer) {
                    inputBuffer->cleanup();
                    delete inputBuffer;
                }
                continue;
            }
            
            // create result structure for our findings
            TuningResult result;
            
            // do the actual frequency analysis using autocorrelation
            if (autocorrelationAnalysis(inputBuffer, &result)) {
                // immediately display the results (no queue needed - same core)
                displayResult(&result);
                __atomic_fetch_add(&processedCount, 1, __ATOMIC_SEQ_CST);
            } else {
                // analysis failed - skip this buffer but don't crash
                safePrintf("Analysis failed for buffer %lu\n", inputBuffer->bufferID);
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
    
    Serial.println("=== ESP32-S3 Multi-Core Audio Tuner with Autocorrelation ===");
    Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("Free Heap: %lu bytes\n", (uint32_t)esp_get_free_heap_size());
    
    // initialize i2s for audio capture first
    if (!initI2S()) {
        Serial.println("FATAL: I2S initialization failed");
        return;
    }
    
    // create mutexes before anything else
    // these prevent cores from interfering with each other
    serialMutex = xSemaphoreCreateMutex();
    statsMutex = xSemaphoreCreateMutex();
    
    if (!serialMutex || !statsMutex) {
        Serial.println("FATAL: Failed to create mutexes");
        return;
    }
    
    Serial.println("Mutexes created successfully");
    
    // create queues for inter-task communication
    // size 4 means we can buffer up to 4 items before blocking
    audioQueue = xQueueCreate(4, sizeof(AudioBuffer*));
    
    if (!audioQueue) {
        Serial.println("FATAL: Failed to create audio queue");
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
        return;
    }
    
    Serial.println("All tasks created successfully");
    Serial.println("System starting with I2S capture and autocorrelation analysis...\n");
}

// main loop does almost nothing
// all real work happens in the freertos tasks

void loop() {
    // just prevent watchdog timer from resetting the system
    vTaskDelay(pdMS_TO_TICKS(1000));
}