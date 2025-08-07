#include <Arduino.h>
#include "config.h"
#include "utilities.h"
#include "display.h"
#include "audioprocessing.h"
#include "tasks.h"

// Setup: runs once at startup
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=== ESP32-S3 Multi-Core Voice Tuner with GC9A01 Display ===");
    Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("Timing Debug: %s\n", ENABLE_TIMING_DEBUG ? "ENABLED" : "DISABLED");
    
    // Initialize display for visual feedback
    initDisplay();
    
    // Display startup message
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.drawCenterString("VOICE TUNER", 120, 80);
    tft.setTextSize(1);
    tft.drawCenterString("Initializing...", 120, 120);
    
    // Initialize I2S audio
    if (!initI2S()) {
        Serial.println("FATAL: I2S initialization failed");
        tft.setTextColor(TFT_RED);
        tft.drawCenterString("I2S FAILED!", 120, 140);
        return;
    }
    
    // Display I2S success
    tft.setTextColor(TFT_GREEN);
    tft.drawCenterString("I2S OK", 120, 140);
    delay(500);
    
    // Create synchronization mutexes
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
    
    // Create inter-task communication queue
    audioQueue = xQueueCreate(4, sizeof(AudioBuffer*));
    
    if (!audioQueue) {
        Serial.println("FATAL: Failed to create audio queue");
        tft.setTextColor(TFT_RED);
        tft.drawCenterString("QUEUE FAILED!", 120, 160);
        return;
    }
    
    Serial.println("Audio queue created successfully");
    
    // Create tasks with core affinity
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
    
    delay(1000);
    
    // Render main tuner interface
    drawTunerInterface();
}

// Main loop: minimal activity
void loop() {
    // Prevent watchdog timeout
    vTaskDelay(pdMS_TO_TICKS(1000));
}