#include "audioprocessing.h"
#include "config.h"
#include "display.h"
#include "menu.h"
#include "tasks.h"
#include "utilities.h"
#include <Arduino.h>

// system initialization
void setup() {
#if ENABLE_SERIAL_MONITOR_PRINT
  Serial.begin(SERIAL_BAUD_RATE);
  delay(2000);

  // startup banner and system info
  Serial.println("=== ESP32-S3 Multi-Core Voice Tuner with GC9A01 Display ===");
  Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  Serial.printf("Timing Debug: %s\n",
                ENABLE_TIMING_DEBUG ? "ENABLED" : "DISABLED");
  Serial.println(
      "Algorithm: Multi-Candidate YIN (Enhanced Harmonic Detection)");
  Serial.printf("Audio Prefiltering: HP=%.0f Hz, LP=%.0f Hz\n",
                HIGHPASS_CUTOFF_HZ, LOWPASS_CUTOFF_HZ);

  // power management configuration
  Serial.println("=== POWER MANAGEMENT FEATURES ===");
  Serial.printf("dB Detection: Activation=%.1fdB, Deactivation=%.1fdB\n",
                DB_ACTIVATION_THRESHOLD, DB_DEACTIVATION_THRESHOLD);
  Serial.printf("Silence Timeout: %d seconds\n", SILENCE_TIMEOUT_MS / 1000);
  Serial.printf("CPU Frequencies: Detecting=%dMHz, Analyzing=%dMHz\n",
                DETECTING_CPU_FREQ, ANALYZING_CPU_FREQ);
  Serial.printf("Initial Power State: %s\n",
                (currentPowerState == DETECTING) ? "DETECTING" : "ANALYZING");
#endif

  // display hardware setup
  initDisplay();
    
  startupAnimation();

  // menu system early initialization (gpio setup)
  safePrintf("initializing menu system...\n");
  // note: full menu initialization happens in processing task
  // this just sets up gpio pins early
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);

  
  // load saved parameters from flash
  safePrintf("loading saved parameters...\n");
  loadParametersFromFlash();



  // audio subsystem initialization (includes filter setup)
  if (!initI2S()) {
    Serial.println("FATAL: I2S initialization failed");
    tft.setTextColor(TFT_RED);
    tft.drawCenterString("I2S FAILED!", 120, 170);
    return;
  }


  // freertos synchronization primitives
  serialMutex = xSemaphoreCreateMutex();
  statsMutex = xSemaphoreCreateMutex();
  displayMutex = xSemaphoreCreateMutex();

  if (!serialMutex || !statsMutex || !displayMutex) {
    Serial.println("FATAL: Failed to create mutexes");
    tft.setTextColor(TFT_RED);
    tft.drawCenterString("MUTEX FAILED!", 120, 190);
    return;
  }

  Serial.println("Mutexes created successfully");

  // inter-task communication queue
  audioQueue = xQueueCreate(AUDIO_QUEUE_LENGTH, sizeof(AudioBuffer *));

  if (!audioQueue) {
    Serial.println("FATAL: Failed to create audio queue");
    tft.setTextColor(TFT_RED);
    tft.drawCenterString("QUEUE FAILED!", 120, 190);
    return;
  }

  Serial.println("Audio queue created successfully");

  // initialize power management state
  currentPowerState = DETECTING;
  silenceTimer = 0;
  lastSoundTime = millis();

  // set initial cpu frequency for detecting mode
  setCpuFrequency(DETECTING_CPU_FREQ);

  // dual-core task creation with affinity
  BaseType_t result1 = xTaskCreatePinnedToCore(
      audioTask, "AudioTask", AUDIO_TASK_STACK_SIZE, NULL, AUDIO_TASK_PRIORITY,
      &audioTaskHandle, AUDIO_TASK_CORE);

  BaseType_t result2 = xTaskCreatePinnedToCore(
      processingAndDisplayTask, "ProcessingTask", PROCESSING_TASK_STACK_SIZE,
      NULL, PROCESSING_TASK_PRIORITY, &processingTaskHandle,
      PROCESSING_TASK_CORE);

  if (result1 != pdPASS || result2 != pdPASS) {
    Serial.println("FATAL: Failed to create tasks");
    tft.setTextColor(TFT_RED);
    tft.drawCenterString("TASK FAILED!", 120, 210);
    return;
  }

  Serial.println("All tasks created successfully");
  Serial.println("System starting with dB Detection Power Management...\n");
  // show power management status
  safePrintf("=== POWER STATE INITIALIZED ===\n");
  safePrintf("Mode: %s\n",
                (currentPowerState == DETECTING) ? "DETECTING" : "ANALYZING");
  safePrintf("CPU: %d MHz\n", getCpuFrequencyMhz());
  safePrintf("====================================\n\n");

  // switch to detecting mode display (shows power saving state)
  displayDetectingMode();
}

// main loop with watchdog prevention
void loop() {
  // yield to freertos scheduler
  vTaskDelay(pdMS_TO_TICKS(1000));
}