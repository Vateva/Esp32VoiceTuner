#include "display.h"
#include "utilities.h"
#include <esp_timer.h>

// Global display instance
LGFX tft;
DisplayState displayState = {"", 0.0f, 0, true, 0, 0, false};

// LovyanGFX configuration implementation
LGFX::LGFX(void) {
  // SPI bus configuration for display communication
  {
    auto cfg = _bus_instance.config();
    cfg.spi_host = SPI2_HOST;
    cfg.spi_mode = 0;
    cfg.freq_write = 40000000;
    cfg.freq_read = 16000000;
    cfg.spi_3wire = true;
    cfg.use_lock = true;
    cfg.dma_channel = SPI_DMA_CH_AUTO;
    cfg.pin_sclk = TFT_SCK;
    cfg.pin_mosi = TFT_MOSI;
    cfg.pin_miso = -1;
    cfg.pin_dc = TFT_DC;
    _bus_instance.config(cfg);
    _panel_instance.setBus(&_bus_instance);
  }

  // GC9A01 panel configuration
  {
    auto cfg = _panel_instance.config();
    cfg.pin_cs = TFT_CS;
    cfg.pin_rst = TFT_RST;
    cfg.pin_busy = -1;
    cfg.panel_width = 240;
    cfg.panel_height = 240;
    cfg.offset_x = 0;
    cfg.offset_y = 0;
    cfg.offset_rotation = 0;
    cfg.dummy_read_pixel = 8;
    cfg.dummy_read_bits = 1;
    cfg.readable = false;
    cfg.invert = true;
    cfg.rgb_order = false;
    cfg.dlen_16bit = false;
    cfg.bus_shared = false;
    _panel_instance.config(cfg);
  }

  setPanel(&_panel_instance);
}

// Initialize display hardware and interface
void initDisplay() {
  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);
  safePrint("display backlight enabled\n");

  tft.init();
  tft.setRotation(0);

  // Display test sequence
  tft.fillScreen(TFT_RED);
  delay(200);
  tft.fillScreen(TFT_GREEN);
  delay(200);
  tft.fillScreen(TFT_BLUE);
  delay(200);
  tft.fillScreen(TFT_BLACK);

  safePrint("display initialized successfully\n");
  drawTunerInterface();
}

// Draw static tuner interface elements
void drawStaticTunerElements() {
  if (displayState.staticCirclesDrawn) {
    return;
  }

  // Draw three static reference circles
  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, DISPLAY_OUTER_RADIUS,
                 TFT_WHITE);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawCenterString("+10c", DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_OUTER_RADIUS - 9);

  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, DISPLAY_MIDDLE_RADIUS,
                 TFT_DARKGREY);
  tft.setTextColor(TFT_GREEN);
  tft.drawCenterString("0c", DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_MIDDLE_RADIUS + 0);

  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, DISPLAY_INNER_RADIUS,
                 TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.drawCenterString("-10c", DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_INNER_RADIUS + 4);

  displayState.staticCirclesDrawn = true;
}

void eraseDynamicCircle() { // erase dynamic circle and redraw middle
                            // circle+labels

  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y,
                 displayState.lastDynamicRadius, TFT_BLACK);
  /*tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y,
                 displayState.lastDynamicRadius + 1, TFT_BLACK);*/
  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, DISPLAY_MIDDLE_RADIUS,
                 TFT_DARKGREY);

  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawCenterString("+10c", DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_OUTER_RADIUS - 9);
  tft.setTextColor(TFT_GREEN);
  tft.drawCenterString("0c", DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_MIDDLE_RADIUS + 0);
  tft.setTextColor(TFT_WHITE);
  tft.drawCenterString("-10c", DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_INNER_RADIUS + 4);
}
// fastest possible - write directly to spi buffer
void drawPreciseCircle(int centerX, int centerY, int radius, uint16_t color) {
  // collect all pixels first, then send in one batch
  static uint16_t pixelBuffer[1000]; // static to avoid stack allocation
  static int16_t xCoords[1000];
  static int16_t yCoords[1000];
  int pixelCount = 0;

  int x = radius;
  int y = 0;
  int decision = 1 - radius;

  // collect all pixel coordinates first
  xCoords[pixelCount] = centerX + radius;
  yCoords[pixelCount++] = centerY;
  xCoords[pixelCount] = centerX - radius;
  yCoords[pixelCount++] = centerY;
  xCoords[pixelCount] = centerX;
  yCoords[pixelCount++] = centerY + radius;
  xCoords[pixelCount] = centerX;
  yCoords[pixelCount++] = centerY - radius;

  while (y < x) {
    y++;
    if (decision <= 0) {
      decision += 2 * y + 1;
    } else {
      x--;
      decision += 2 * (y - x) + 1;
    }

    // collect all 8 symmetric points
    xCoords[pixelCount] = centerX + x;
    yCoords[pixelCount++] = centerY + y;
    xCoords[pixelCount] = centerX - x;
    yCoords[pixelCount++] = centerY + y;
    xCoords[pixelCount] = centerX + x;
    yCoords[pixelCount++] = centerY - y;
    xCoords[pixelCount] = centerX - x;
    yCoords[pixelCount++] = centerY - y;
    xCoords[pixelCount] = centerX + y;
    yCoords[pixelCount++] = centerY + x;
    xCoords[pixelCount] = centerX - y;
    yCoords[pixelCount++] = centerY + x;
    xCoords[pixelCount] = centerX + y;
    yCoords[pixelCount++] = centerY - x;
    xCoords[pixelCount] = centerX - y;
    yCoords[pixelCount++] = centerY - x;
  }

  // now write all pixels in one optimized batch
  tft.startWrite();
  for (int i = 0; i < pixelCount; i++) {
    tft.writePixel(xCoords[i], yCoords[i], color);
  }
  tft.endWrite();
}

// Efficiently draw dynamic 1-pixel thick circle based on cents
void drawOptimizedCentsCircle(float cents, uint16_t circleColor) {

  // Calculate radius using linear mapping
  int newRadius = DISPLAY_MIDDLE_RADIUS + (int)(cents * 2.0f);

  // Handle extreme values
  if (cents < -10.0f) {
    newRadius = DISPLAY_INNER_RADIUS - (int)((fabs(cents) - 10.0f) * 1.0f);
    if (newRadius < (DISPLAY_INNER_RADIUS - 27))
      newRadius = (DISPLAY_INNER_RADIUS - 27);
  } else if (cents > 10.0f) {
    newRadius = DISPLAY_OUTER_RADIUS + (int)((cents - 10.0f) * 1.0f);
    if (newRadius > (DISPLAY_OUTER_RADIUS + 26))
      newRadius = (DISPLAY_OUTER_RADIUS + 26);//biggest possible circle in display
  }

  // skip over static inner and outer circles
  if (newRadius >= (DISPLAY_INNER_RADIUS - 1) && newRadius <= (DISPLAY_INNER_RADIUS + 1)) {
    newRadius = (cents < -10) ? (DISPLAY_INNER_RADIUS - 1) : (DISPLAY_INNER_RADIUS + 1);
  } else if (newRadius >= (DISPLAY_OUTER_RADIUS - 1) && newRadius <= (DISPLAY_OUTER_RADIUS + 1)) {
    newRadius = (cents < 10) ? (DISPLAY_OUTER_RADIUS - 1) : (DISPLAY_OUTER_RADIUS + 1);
  }
  // Erase previous dynamic circle
  if (displayState.lastDynamicRadius > 0 &&
      displayState.lastDynamicRadius != newRadius) {
    eraseDynamicCircle();
  }

  // Draw new dynamic circle
  if (newRadius != displayState.lastDynamicRadius ||
      circleColor != displayState.lastDynamicColor) {
    tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, newRadius, circleColor);
    /*tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, newRadius + 1,
                   circleColor);*/

    displayState.lastDynamicRadius = newRadius;
    displayState.lastDynamicColor = circleColor;
  }
}

// Draw complete interface
void drawTunerInterface() {
  tft.fillScreen(TFT_BLACK);
  displayState.staticCirclesDrawn = false;
  displayState.lastDynamicRadius = 0;
  displayState.needsFullRedraw = false;
  drawStaticTunerElements();
}

// Update dynamic tuner display elements efficiently
void updateTunerDisplay(const char *note, float cents, TuningResult *result,
                        bool hasAudio) {
  if (!displayMutex)
    return;

  if (result) {
    result->displayStartTime = esp_timer_get_time();
    printTiming("display start", result->bufferID, result->displayStartTime,
                result->captureTime);
  }

  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    drawStaticTunerElements();

    uint16_t noteColor;

    if (fabs(cents) <= 10.0f) {
      noteColor = TFT_GREEN; // 0-10 cents
    } else if (fabs(cents) <= 13.0f) {
      noteColor = TFT_GREENYELLOW; // 10-13 cents
    } else if (fabs(cents) <= 18.0f) {
      noteColor = TFT_YELLOW; // 13-18 cents
    } else if (fabs(cents) <= 20.0f) {
      noteColor = TFT_ORANGE; // 18-20 cents
    } else {
      noteColor = TFT_RED; // >20 cents
    }

    // update text only if significantly changed
    if (strcmp(note, displayState.lastNoteName) != 0 ||
        fabs(cents - displayState.lastCentsOffset) > 1.0f) {

      // smallest rectangle that erases the note and cent count
      tft.fillRect(93, 97, 51, 44, TFT_BLACK);

      tft.setTextColor(noteColor); // cents displaying
      tft.setTextSize(2);
      char centsStr[8];
      snprintf(centsStr, sizeof(centsStr), "%.0f", cents);
      tft.drawCenterString(centsStr, 120, 97);

      tft.setTextSize(3); //  note displaying
      tft.drawCenterString(note, 120, 120);

      strcpy(displayState.lastNoteName, note);
      displayState.lastCentsOffset = cents;
    }

    if (hasAudio) {
      drawOptimizedCentsCircle(cents, noteColor);
    } else if (displayState.lastDynamicRadius > 0) {
      // erase existing circle
      eraseDynamicCircle();
      displayState.lastDynamicRadius = 0;
    }

    xSemaphoreGive(displayMutex);

    if (result) {
      result->displayEndTime = esp_timer_get_time();
      printTiming("display end", result->bufferID, result->displayEndTime,
                  result->captureTime);
    }
  }
}

// Display tuning results on GC9A01
void displayResult(const TuningResult *result, const AudioBuffer *buffer) {
  if (!result || !result->validate()) {
    if (displayState.lastNoteName[0] != '\0') {
      updateTunerDisplay("--", 0.0f, nullptr, false);
      displayState.lastNoteName[0] = '\0';
    }
    return;
  }

  UBaseType_t queueDepth = uxQueueMessagesWaiting(audioQueue);
  bool skipDisplayUpdate = (queueDepth > 2);

  TuningResult mutableResult = *result;

  bool needsUpdate = false;

  if (strcmp(result->noteName, displayState.lastNoteName) != 0) {
    needsUpdate = true;
  }

  if (fabs(result->centsOffset - displayState.lastCentsOffset) > 2.0f) {
    needsUpdate = true;
  }

  if ((needsUpdate || displayState.needsFullRedraw) && !skipDisplayUpdate) {
    updateTunerDisplay(result->noteName, result->centsOffset, &mutableResult,
                       true);
    displayState.lastUpdateTime = millis();

    strcpy(displayState.lastNoteName, result->noteName);
    displayState.lastCentsOffset = result->centsOffset;
  } else if (skipDisplayUpdate) {
    mutableResult.displayStartTime = esp_timer_get_time();
    mutableResult.displayEndTime = mutableResult.displayStartTime + 1000;
    printTiming("display skipped", mutableResult.bufferID,
                mutableResult.displayEndTime, mutableResult.captureTime);
  }

  if (buffer) {
    printTimingSummary(&mutableResult, buffer);
  }

  uint64_t totalLatency = mutableResult.displayEndTime - result->captureTime;
  updateStats(totalLatency);
}