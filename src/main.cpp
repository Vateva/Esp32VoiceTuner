#include <Arduino.h>
#include <cmath>

// safe data structures for audio processing
// these structures are designed to prevent memory corruption and crashes

// this structure holds audio samples and metadata
// we use heap allocation because audio buffers are large (1024+ samples)
struct AudioBuffer {
    float* samples;                // pointer to actual audio data stored in heap memory
    uint64_t captureTime;          // timestamp when audio was recorded (in microseconds)
    uint32_t bufferID;             // unique number to track this specific buffer
    uint16_t sampleCount;          // how many audio samples we have (max 1024)
    float amplitude;               // volume level of the test signal we generated
    bool isValid;                  // flag to check if this buffer got corrupted
    
    // constructor initializes all values to safe defaults
    AudioBuffer() : samples(nullptr), captureTime(0), bufferID(0), 
                   sampleCount(0), amplitude(0.0f), isValid(false) {}
    
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

// thread-safe global variables for multi-core communication
// freertos uses handles to reference queues, tasks, and mutexes

// mutex prevents multiple cores from writing to serial at the same time
// without this, serial output gets scrambled when cores try to print simultaneously
SemaphoreHandle_t serialMutex = nullptr;

// queues let cores send data to each other safely
// audioqueue sends audiobuffer pointers from core 0 to core 1
// resultqueue sends tuningresult pointers from core 1 back to core 0
QueueHandle_t audioQueue = nullptr;      // holds AudioBuffer* (just pointers, not full structs)
QueueHandle_t resultQueue = nullptr;     // holds TuningResult* (pointers are fast to copy)

// task handles let us monitor and control the running tasks
TaskHandle_t audioTaskHandle = nullptr;
TaskHandle_t mathTaskHandle = nullptr;
TaskHandle_t displayTaskHandle = nullptr;

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

// audio processing functions for testing and simulation

// create fake audio data for testing the system
// generates sine waves at known frequencies so we can verify detection works
bool generateTestAudio(AudioBuffer* buffer) {
    if (!buffer || !buffer->validate()) {
        safePrint("ERROR: Invalid buffer in generateTestAudio\n");
        return false;
    }
    
    // record when we captured this audio
    buffer->captureTime = esp_timer_get_time();
    
    // create slightly different frequency for each buffer so we can see changes
    float frequency = 440.0f + (buffer->bufferID % 20) - 10.0f; // 430-450 hz range
    // vary amplitude over time to simulate real audio
    buffer->amplitude = 0.5f + 0.3f * sinf(buffer->bufferID * 0.1f);
    
    // fill the sample array with sine wave data
    // 16khz sample rate means each sample represents 1/16000 seconds
    for (uint16_t i = 0; i < buffer->sampleCount; i++) {
        float time = i / 16000.0f; // convert sample index to time in seconds
        buffer->samples[i] = buffer->amplitude * sinf(2.0f * PI * frequency * time);
    }
    
    return true;
}

// fake frequency analysis that pretends to do fft and pitch detection
// in real code this would be complex math, but for testing we just simulate the delay
bool simulateProcessing(const AudioBuffer* input, TuningResult* output) {
    if (!input || !output || !input->validate()) {
        safePrint("ERROR: Invalid input to simulateProcessing\n");
        return false;
    }
    
    // simulate the time real fft/pitch detection would take (25-75ms)
    uint32_t processTimeMs = 25 + (input->bufferID % 50);
    vTaskDelay(pdMS_TO_TICKS(processTimeMs));
    
    // fill output with fake but realistic results
    output->bufferID = input->bufferID;
    output->captureTime = input->captureTime;
    output->processTime = esp_timer_get_time();
    output->frequency = 440.0f + (input->bufferID % 20) - 10.0f; // match input frequency
    output->confidence = input->amplitude; // louder signals = higher confidence
    output->centsOffset = (output->frequency - 440.0f) * 3.93f; // convert hz to cents
    
    // determine note name based on frequency
    // safe string operations prevent buffer overflows
    if (output->frequency < 435.0f) {
        strncpy(output->noteName, "A4-", sizeof(output->noteName) - 1);
    } else if (output->frequency > 445.0f) {
        strncpy(output->noteName, "A4+", sizeof(output->noteName) - 1);
    } else {
        strncpy(output->noteName, "A4", sizeof(output->noteName) - 1);
    }
    output->noteName[sizeof(output->noteName) - 1] = '\0'; // ensure null termination
    
    output->isValid = true;
    return output->validate();
}

// core 0: audio capture task
// this runs on core 0 and captures audio every 64ms
// in real implementation this would read from inmp441 microphone

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
            
            // fill with test audio data
            // in real version this would be: i2s_read() from inmp441
            if (!generateTestAudio(buffer)) {
                safePrint("ERROR: Failed to generate test audio\n");
                buffer->cleanup();
                delete buffer;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            // send buffer pointer to processing queue
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

// core 1: math processing task
// this runs on core 1 and does the heavy fft/pitch detection math
// separating this from audio capture prevents audio dropouts

void mathTask(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(150)); // wait for system startup
    
    uint8_t coreID = xPortGetCoreID();
    safePrintf("Math task started on core %d\n", coreID);
    
    while (true) {
        AudioBuffer* inputBuffer;
        
        // wait for audio data from capture task
        // portmax_delay means wait forever until data arrives
        if (xQueueReceive(audioQueue, &inputBuffer, portMAX_DELAY) == pdPASS) {
            if (!inputBuffer || !inputBuffer->validate()) {
                safePrint("ERROR: Received invalid buffer in math task\n");
                if (inputBuffer) {
                    inputBuffer->cleanup();
                    delete inputBuffer;
                }
                continue;
            }
            
            // create result structure for our findings
            TuningResult* result = new TuningResult();
            if (!result) {
                safePrint("ERROR: Failed to allocate result\n");
                inputBuffer->cleanup();
                delete inputBuffer;
                continue;
            }
            
            // do the actual frequency analysis
            // in real version this would be: fft, peak detection, pitch calculation
            if (simulateProcessing(inputBuffer, result)) {
                // send results to display task
                if (xQueueSend(resultQueue, &result, pdMS_TO_TICKS(100)) != pdPASS) {
                    safePrintf("ERROR: Failed to send result for buffer %lu\n", 
                              inputBuffer->bufferID);
                    delete result;
                } else {
                    __atomic_fetch_add(&processedCount, 1, __ATOMIC_SEQ_CST);
                }
            } else {
                safePrint("ERROR: Processing failed\n");
                delete result;
            }
            
            // always clean up input buffer memory
            inputBuffer->cleanup();
            delete inputBuffer;
        }
    }
}

// core 0: display task
// this runs on core 0 and shows results on gc9a01 display
// also handles serial output for debugging

void displayTask(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(200)); // wait for system startup
    
    uint8_t coreID = xPortGetCoreID();
    safePrintf("Display task started on core %d\n", coreID);
    
    while (true) {
        TuningResult* result;
        
        // check for new analysis results
        // timeout prevents blocking forever if no results come
        if (xQueueReceive(resultQueue, &result, pdMS_TO_TICKS(100)) == pdPASS) {
            if (result && result->validate()) {
                // calculate total time from capture to display
                uint64_t totalLatency = result->processTime - result->captureTime;
                updateStats(totalLatency);
                
                // show the tuning result
                // in real version this would update gc9a01 display graphics
                safePrintf("Buffer %lu: %s %.1fHz %+.1f cents (%.1f%%) lat=%llu μs\n",
                          result->bufferID, result->noteName, result->frequency,
                          result->centsOffset, result->confidence * 100.0f, totalLatency);
            } else {
                safePrint("ERROR: Received invalid result\n");
            }
            
            // clean up result memory
            if (result) delete result;
        }
        
        // small delay prevents excessive cpu usage
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// setup function runs once at startup
// initializes all the freertos objects and starts tasks

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=== ESP32-S3 Multi-Core Audio Tuner (Memory-Safe) ===");
    Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("Free Heap: %lu bytes\n", (uint32_t)esp_get_free_heap_size());
    
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
    resultQueue = xQueueCreate(4, sizeof(TuningResult*));
    
    if (!audioQueue || !resultQueue) {
        Serial.println("FATAL: Failed to create queues");
        return;
    }
    
    Serial.println("Queues created successfully");
    
    // create tasks with specific core assignments
    // audio and display on core 0, math on core 1 for load balancing
    BaseType_t result1 = xTaskCreatePinnedToCore(
        audioTask, "AudioTask", 16384, NULL, 3, &audioTaskHandle, 0);
    
    BaseType_t result2 = xTaskCreatePinnedToCore(
        displayTask, "DisplayTask", 12288, NULL, 2, &displayTaskHandle, 0);
    
    BaseType_t result3 = xTaskCreatePinnedToCore(
        mathTask, "MathTask", 16384, NULL, 3, &mathTaskHandle, 1);
    
    if (result1 != pdPASS || result2 != pdPASS || result3 != pdPASS) {
        Serial.println("FATAL: Failed to create tasks");
        return;
    }
    
    Serial.println("All tasks created successfully");
    Serial.println("System starting...\n");
}

// main loop does almost nothing
// all real work happens in the freertos tasks

void loop() {
    // just prevent watchdog timer from resetting the system
    vTaskDelay(pdMS_TO_TICKS(1000));
}