#include <Arduino.h>
#include "config.h"
#include "utilities.h"
#include "display.h"
#include "audioprocessing.h"
#include "tasks.h"

// system initialization
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    // startup banner and system info
    Serial.println("=== ESP32-S3 Multi-Core Voice Tuner with GC9A01 Display ===");
    Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("Timing Debug: %s\n", ENABLE_TIMING_DEBUG ? "ENABLED" : "DISABLED");
    Serial.println("Algorithm: Multi-Candidate YIN (Enhanced Harmonic Detection)");
    
    // display hardware setup
    initDisplay();
    
    // startup screen
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.drawCenterString("VOICE TUNER", 120, 80);
    tft.setTextSize(1);
    tft.drawCenterString("Multi-Candidate YIN", 120, 110);
    tft.drawCenterString("Initializing...", 120, 130);
    
    // audio subsystem initialization
    if (!initI2S()) {
        Serial.println("FATAL: I2S initialization failed");
        tft.setTextColor(TFT_RED);
        tft.drawCenterString("I2S FAILED!", 120, 150);
        return;
    }
    
    // i2s success indicator
    tft.setTextColor(TFT_GREEN);
    tft.drawCenterString("I2S OK", 120, 150);
    delay(500);
    
    // freertos synchronization primitives
    serialMutex = xSemaphoreCreateMutex();
    statsMutex = xSemaphoreCreateMutex();
    displayMutex = xSemaphoreCreateMutex();
    
    if (!serialMutex || !statsMutex || !displayMutex) {
        Serial.println("FATAL: Failed to create mutexes");
        tft.setTextColor(TFT_RED);
        tft.drawCenterString("MUTEX FAILED!", 120, 170);
        return;
    }
    
    Serial.println("Mutexes created successfully");
    
    // inter-task communication queue
    audioQueue = xQueueCreate(4, sizeof(AudioBuffer*));
    
    if (!audioQueue) {
        Serial.println("FATAL: Failed to create audio queue");
        tft.setTextColor(TFT_RED);
        tft.drawCenterString("QUEUE FAILED!", 120, 170);
        return;
    }
    
    Serial.println("Audio queue created successfully");
    
    // dual-core task creation with affinity
    BaseType_t result1 = xTaskCreatePinnedToCore(
        audioTask, "AudioTask", 16384, NULL, 3, &audioTaskHandle, 0);
    
    BaseType_t result2 = xTaskCreatePinnedToCore(
        processingAndDisplayTask, "ProcessingTask", 16384, NULL, 2, &processingTaskHandle, 1);
    
    if (result1 != pdPASS || result2 != pdPASS) {
        Serial.println("FATAL: Failed to create tasks");
        tft.setTextColor(TFT_RED);
        tft.drawCenterString("TASK FAILED!", 120, 170);
        return;
    }
    
    Serial.println("All tasks created successfully");
    Serial.println("System starting with Multi-Candidate YIN for enhanced harmonic detection...\n");
    
    delay(1000);
    
    // switch to main tuner interface
    drawTunerInterface();
}

// main loop with watchdog prevention
void loop() {
    // yield to freertos scheduler
    vTaskDelay(pdMS_TO_TICKS(1000));
}