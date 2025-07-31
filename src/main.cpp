/*
 * Combined Test: Display + Microphone for ESP32-S3-Zero
 * Using LovyanGFX library for display control
 * Shows microphone audio levels on the circular display
 * 
 * Install LovyanGFX library first: 
 * Tools -> Manage Libraries -> Search "LovyanGFX" -> Install
 * 
 * Complete Wiring:
 * Display     ESP32-S3-Zero     Microphone    ESP32-S3-Zero
 * VCC      -> 3.3V             VCC        -> 3.3V
 * GND      -> GND              GND        -> GND
 * SCK      -> GPIO7            SCK        -> GPIO4
 * MOSI     -> GPIO8            WS         -> GPIO5
 * CS       -> GPIO9            SD         -> GPIO6
 * DC       -> GPIO10           L/R        -> GND
 * RST      -> GPIO11
 * BLK      -> GPIO12
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "driver/i2s.h"

// Display pin definitions
#define TFT_SCK     1   // SPI Clock
#define TFT_MOSI    2   // SPI Data  
#define TFT_CS      5   // Chip Select
#define TFT_DC      4   // Data/Command
#define TFT_RST     3   // Reset
#define TFT_BLK     6   // Backlight

// I2S Microphone pins
#define I2S_SCK     13   // I2S Serial Clock
#define I2S_WS      12   // I2S Word Select  
#define I2S_SD      11   // I2S Serial Data
#define I2S_PORT    I2S_NUM_0

// Audio settings
#define SAMPLE_RATE     44100
#define BUFFER_SIZE     512
#define BUFFER_COUNT    4

// Function declarations
void showStartupScreen();
void updateStatusText(const char* text, uint32_t color);
void setupDisplayLayout();
float calculateRMS(int samplesRead);
void updateDisplay(float rms);
bool setupI2S();

// LovyanGFX configuration class
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Bus_SPI _bus_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;    // SPI clock for writing (40MHz)
      cfg.freq_read = 16000000;     // SPI clock for reading  
      cfg.spi_3wire = true;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = TFT_SCK;
      cfg.pin_mosi = TFT_MOSI;
      cfg.pin_miso = -1;            // Not used
      cfg.pin_dc = TFT_DC;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = TFT_CS;
      cfg.pin_rst = TFT_RST;
      cfg.pin_busy = -1;            // Not used
      cfg.panel_width = 240;
      cfg.panel_height = 240;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = false;
      cfg.invert = true;            // GC9A01 typically needs inversion
      cfg.rgb_order = false;        // BGR order
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      _panel_instance.config(cfg);
    }

    setPanel(&_panel_instance);
  }
};

// Create display instance
LGFX tft;
int32_t samples[BUFFER_SIZE];

// Level history for smooth display
float levelHistory[60];
int historyIndex = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Combined Test: LovyanGFX + I2S Microphone ===");
  
  // Initialize backlight
  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);
  Serial.println("Backlight enabled");
  
  // Initialize display
  Serial.println("Initializing LovyanGFX display...");
  tft.init();
  tft.setRotation(0);
  
  // Test display with color flashes
  Serial.println("Testing display colors...");
  tft.fillScreen(TFT_RED);
  delay(200);
  tft.fillScreen(TFT_GREEN);
  delay(200);
  tft.fillScreen(TFT_BLUE);
  delay(200);
  tft.fillScreen(TFT_BLACK);
  
  Serial.println("Display initialized successfully!");
  
  // Show startup screen
  showStartupScreen();
  
  // Initialize I2S
  Serial.println("Initializing I2S microphone...");
  if (setupI2S()) {
    Serial.println("I2S initialized successfully");
    updateStatusText("I2S Ready", TFT_GREEN);
  } else {
    Serial.println("I2S initialization failed!");
    updateStatusText("I2S Failed", TFT_RED);
    while(1) delay(1000);
  }
  
  delay(2000);
  setupDisplayLayout();
  
  Serial.println("=== Ready! Make noise near the microphone... ===");
}

void loop() {
  static unsigned long lastUpdate = 0;
  
  // Read microphone data
  size_t bytesRead = 0;
  esp_err_t result = i2s_read(I2S_PORT, samples, sizeof(samples), &bytesRead, 100);
  
  if (result == ESP_OK && bytesRead > 0) {
    int samplesRead = bytesRead / sizeof(int32_t);
    
    // Calculate audio level
    float rms = calculateRMS(samplesRead);
    
    // Store in history for smoothing
    levelHistory[historyIndex] = rms;
    historyIndex = (historyIndex + 1) % 60;
    
    // Update display at ~30 FPS
    if (millis() - lastUpdate > 33) {
      lastUpdate = millis();
      updateDisplay(rms);
    }
  }
}

void showStartupScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.drawCenterString("Voice Tuner", 120, 80);
  tft.setTextSize(1);
  tft.drawCenterString("LovyanGFX Test", 120, 105);
  tft.drawCenterString("Initializing...", 120, 125);
}

void updateStatusText(const char* text, uint32_t color) {
  tft.fillRect(0, 120, 240, 25, TFT_BLACK);
  tft.setTextColor(color);
  tft.setTextSize(1);
  tft.drawCenterString(text, 120, 125);
}

void setupDisplayLayout() {
  tft.fillScreen(TFT_BLACK);
  
  // Draw title
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.drawCenterString("Audio Level", 120, 20);
  
  // Draw circular level meter guides
  tft.drawCircle(120, 120, 90, TFT_DARKGREY);
  tft.drawCircle(120, 120, 70, TFT_DARKGREY);
  tft.drawCircle(120, 120, 50, TFT_DARKGREY);
  tft.drawCircle(120, 120, 30, TFT_DARKGREY);
  tft.drawCircle(120, 120, 10, TFT_DARKGREY);
  
  // Draw level labels
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY);
  tft.drawCenterString("SILENT", 120, 45);
  tft.drawCenterString("LOUD", 120, 190);
  
  // Draw some tick marks around the circle
  for (int angle = 0; angle < 360; angle += 30) {
    float radian = angle * PI / 180.0;
    int x1 = 120 + cos(radian) * 85;
    int y1 = 120 + sin(radian) * 85;
    int x2 = 120 + cos(radian) * 90;
    int y2 = 120 + sin(radian) * 90;
    tft.drawLine(x1, y1, x2, y2, TFT_DARKGREY);
  }
}

float calculateRMS(int samplesRead) {
  float rms = 0;
  
  for (int i = 0; i < samplesRead; i++) {
    // Extract 24-bit value from 32-bit container
    int32_t sample = samples[i] >> 8;  // Shift to get 24-bit signed value
    
    // Normalize to ±1.0 range
    float gain = 2.0f;
    float normalized = (float)sample / 8388608.0f * gain; // 2^23 for 24-bit
    rms += normalized * normalized;
  }
  
  return sqrt(rms / samplesRead);
}

void updateDisplay(float rms) {
  static float smoothedLevel = 0;
  static float lastDisplayedLevel = 0;
  
  // Smooth the level for display (exponential moving average)
  smoothedLevel = smoothedLevel * 0.7 + rms * 0.3;
  
  // Only update if change is significant (reduce flicker)
  if (abs(smoothedLevel - lastDisplayedLevel) < 0.005) return;
  lastDisplayedLevel = smoothedLevel;
  
  // Clear the center area (avoid clearing the guide circles)
  tft.fillCircle(120, 120, 89, TFT_BLACK);
  
  // Calculate display radius (scale RMS to visual range)
  int radius = (int)(smoothedLevel * 400);  // Scale factor for visibility
  if (radius > 85) radius = 85;  // Max radius to stay within guides
  if (radius < 3) radius = 3;    // Minimum visible radius
  
  // Choose color based on level
  uint32_t color;
  if (smoothedLevel < 0.05) {
    color = TFT_BLUE;      // Very quiet
  } else if (smoothedLevel < 0.15) {
    color = TFT_CYAN;      // Quiet
  } else if (smoothedLevel < 0.3) {
    color = TFT_GREEN;     // Good level
  } else if (smoothedLevel < 0.5) {
    color = TFT_YELLOW;    // Getting loud
  } else if (smoothedLevel < 0.7) {
    color = TFT_ORANGE;    // Loud
  } else {
    color = TFT_RED;       // Very loud
  }
  
  // Draw the main level circle
  tft.fillCircle(120, 120, radius, color);
  
  // Draw inner rings for visual effect
  if (radius > 15) {
    tft.drawCircle(120, 120, radius - 5, TFT_WHITE);
  }
  if (radius > 25) {
    tft.drawCircle(120, 120, radius - 10, TFT_WHITE);
  }
  
  // Redraw the guide circles on top
  tft.drawCircle(120, 120, 90, TFT_DARKGREY);
  tft.drawCircle(120, 120, 70, TFT_DARKGREY);
  tft.drawCircle(120, 120, 50, TFT_DARKGREY);
  tft.drawCircle(120, 120, 30, TFT_DARKGREY);
  tft.drawCircle(120, 120, 10, TFT_DARKGREY);
  
  // Show numeric values in center
  tft.fillRect(90, 110, 60, 30, TFT_BLACK);
  tft.setTextColor(color);
  tft.setTextSize(1);
  
  // RMS value
  char rmsStr[10];
  sprintf(rmsStr, "%.3f", smoothedLevel);
  tft.drawCenterString(rmsStr, 120, 115);
  
  // dB value  
  float db = 20 * log10(smoothedLevel + 0.000001);  // Avoid log(0)
  char dbStr[15];
  sprintf(dbStr, "%.1fdB", db);
  tft.setTextColor(TFT_WHITE);
  tft.drawCenterString(dbStr, 120, 130);
  
  // Status text at bottom
  const char* status;
  if (smoothedLevel < 0.01) status = "TOO QUIET";
  else if (smoothedLevel < 0.1) status = "QUIET";
  else if (smoothedLevel < 0.3) status = "GOOD";
  else if (smoothedLevel < 0.6) status = "LOUD";
  else status = "TOO LOUD";
  
  tft.fillRect(0, 205, 240, 15, TFT_BLACK);
  tft.setTextColor(color);
  tft.drawCenterString(status, 120, 210);
  
  // Debug output to serial
  Serial.printf("RMS: %.4f | dB: %.1f | Radius: %d | Status: %s\n", 
                smoothedLevel, db, radius, status);
}

bool setupI2S() {
  // I2S configuration
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,  // 24-bit data in 32-bit container
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,   // Mono from left channel
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = BUFFER_COUNT,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = true  // Use audio PLL for precise timing
  };
  
  // Pin configuration
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,  // Not used for input
    .data_in_num = I2S_SD
  };
  
  // Install and start I2S driver
  esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (result != ESP_OK) {
    Serial.printf("Failed to install I2S driver: %s\n", esp_err_to_name(result));
    return false;
  }
  
  result = i2s_set_pin(I2S_PORT, &pin_config);
  if (result != ESP_OK) {
    Serial.printf("Failed to set I2S pins: %s\n", esp_err_to_name(result));
    return false;
  }
  
  result = i2s_start(I2S_PORT);
  if (result != ESP_OK) {
    Serial.printf("Failed to start I2S: %s\n", esp_err_to_name(result));
    return false;
  }
  
  return true;
}