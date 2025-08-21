#include "display.h"
#include "utilities.h"
#include <esp_timer.h>

// global display instance
LGFX tft;
DisplayState displayState = {"", 0.0f, 0, true, 0, 0, false};

// lovyangfx spi bus configuration
LGFX::LGFX(void) {
  // spi bus setup for display communication
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

  // gc9a01 panel configuration
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

// hardware initialization and test sequence
void initDisplay() {
  // enable backlight
  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);
  safePrint("display backlight enabled\n");

  tft.init();
  tft.setRotation(0);

  // rgb test sequence
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

// draw static reference circles and labels
void drawStaticTunerElements() {
  if (displayState.staticCirclesDrawn) {
    return;
  }
  char tempStr[20];
  
  // outer circle (+cents threshold)
  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, DISPLAY_OUTER_RADIUS,
                 TFT_WHITE);
  tft.setTextSize(1);

  sprintf(tempStr, "%dc", CENTS_OVER_THRESHOLD);
  tft.setTextColor(TFT_WHITE);
  tft.drawCenterString(tempStr, DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_OUTER_RADIUS - 9);

  // middle circle (perfect pitch)
  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, DISPLAY_MIDDLE_RADIUS,
                 TFT_DARKGREY);
  tft.setTextColor(TFT_GREEN);
  tft.drawCenterString("0c", DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_MIDDLE_RADIUS + 0);

  // inner circle (-cents threshold)
  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, DISPLAY_INNER_RADIUS,
                 TFT_WHITE);

  sprintf(tempStr, "%dc", CENTS_UNDER_THRESHOLD);
  tft.setTextColor(TFT_WHITE);
  tft.drawCenterString(tempStr, DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_INNER_RADIUS + 4);

  displayState.staticCirclesDrawn = true;
}

// clean up dynamic circle and restore static elements
void eraseDynamicCircle() {
  // erase previous dynamic circle
  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y,
                 displayState.lastDynamicRadius, TFT_BLACK);
  
  // redraw middle reference circle
  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, DISPLAY_MIDDLE_RADIUS,
                 TFT_DARKGREY);

  tft.setTextSize(1);
  
  char tempStr[20];

  // restore labels that may have been overwritten
  sprintf(tempStr, "%dc", CENTS_OVER_THRESHOLD);
  tft.setTextColor(TFT_WHITE);
  tft.drawCenterString(tempStr, DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_OUTER_RADIUS - 9);
  tft.setTextColor(TFT_GREEN);

  tft.drawCenterString("0c", DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_MIDDLE_RADIUS + 0);

  sprintf(tempStr, "%dc", CENTS_UNDER_THRESHOLD);
  tft.setTextColor(TFT_WHITE);
  tft.drawCenterString(tempStr, DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_INNER_RADIUS + 4);
}

// optimized circle drawing using bresenham algorithm
void drawPreciseCircle(int centerX, int centerY, int radius, uint16_t color) {
  // static buffers to avoid repeated allocation
  static uint16_t pixelBuffer[1000];
  static int16_t xCoords[1000];
  static int16_t yCoords[1000];
  int pixelCount = 0;

  // bresenham circle algorithm
  int x = radius;
  int y = 0;
  int decision = 1 - radius;

  // initial cardinal points
  xCoords[pixelCount] = centerX + radius;
  yCoords[pixelCount++] = centerY;
  xCoords[pixelCount] = centerX - radius;
  yCoords[pixelCount++] = centerY;
  xCoords[pixelCount] = centerX;
  yCoords[pixelCount++] = centerY + radius;
  xCoords[pixelCount] = centerX;
  yCoords[pixelCount++] = centerY - radius;

  // calculate all 8-fold symmetric points
  while (y < x) {
    y++;
    if (decision <= 0) {
      decision += 2 * y + 1;
    } else {
      x--;
      decision += 2 * (y - x) + 1;
    }

    // store all 8 symmetric points
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

  // batch pixel write for efficiency
  tft.startWrite();
  for (int i = 0; i < pixelCount; i++) {
    tft.writePixel(xCoords[i], yCoords[i], color);
  }
  tft.endWrite();
}

// draw dynamic cents visualization circle
void drawOptimizedCentsCircle(int cents, uint16_t circleColor) {
  int newRadius;
  
  // calculate scaling factors based on threshold configuration
  float lowerScale = (float)(DISPLAY_MIDDLE_RADIUS - DISPLAY_INNER_RADIUS) / abs(CENTS_UNDER_THRESHOLD);
  float upperScale = (float)(DISPLAY_OUTER_RADIUS - DISPLAY_MIDDLE_RADIUS) / CENTS_OVER_THRESHOLD;
  
  // map cents to radius across four zones
  if (cents < CENTS_UNDER_THRESHOLD) {
    // zone 1: extreme flat (shrink inward from inner circle)
    float extraCents = abs(cents) - abs(CENTS_UNDER_THRESHOLD);
    newRadius = DISPLAY_INNER_RADIUS - (int)(extraCents * 1.0f);
    
    // clamp minimum radius
    if (newRadius < (DISPLAY_INNER_RADIUS - 27)) {
      newRadius = DISPLAY_INNER_RADIUS - 27;
    }
  } 
  else if (cents < 0) {
    // zone 2: moderate flat (between inner and middle)
    newRadius = DISPLAY_MIDDLE_RADIUS + (int)(cents * lowerScale);
  } 
  else if (cents <= CENTS_OVER_THRESHOLD) {
    // zone 3: moderate sharp (between middle and outer)
    newRadius = DISPLAY_MIDDLE_RADIUS + (int)(cents * upperScale);
  } 
  else {
    // zone 4: extreme sharp (grow outward from outer circle)
    float extraCents = cents - CENTS_OVER_THRESHOLD;
    newRadius = DISPLAY_OUTER_RADIUS + (int)(extraCents * 1.0f);
    
    // clamp maximum radius
    if (newRadius > (DISPLAY_OUTER_RADIUS + 26)) {
      newRadius = DISPLAY_OUTER_RADIUS + 26;
    }
  }
  
  // avoid collision with static reference circles
  if (newRadius >= (DISPLAY_INNER_RADIUS - 1) && newRadius <= (DISPLAY_INNER_RADIUS + 1)) {
    newRadius = (cents < CENTS_UNDER_THRESHOLD) ? (DISPLAY_INNER_RADIUS - 1) : (DISPLAY_INNER_RADIUS + 1);
  } 
  else if (newRadius >= (DISPLAY_OUTER_RADIUS - 1) && newRadius <= (DISPLAY_OUTER_RADIUS + 1)) {
    newRadius = (cents < CENTS_OVER_THRESHOLD) ? (DISPLAY_OUTER_RADIUS - 1) : (DISPLAY_OUTER_RADIUS + 1);
  }
  else if (newRadius >= (DISPLAY_MIDDLE_RADIUS - 1) && newRadius <= (DISPLAY_MIDDLE_RADIUS + 1)) {
    newRadius = (cents < 0) ? (DISPLAY_MIDDLE_RADIUS - 1) : (DISPLAY_MIDDLE_RADIUS + 1);
  }
  
  // erase previous circle if radius changed
  if (displayState.lastDynamicRadius > 0 &&
      displayState.lastDynamicRadius != newRadius) {
    eraseDynamicCircle();
  }
  
  // draw new circle if different from previous
  if (newRadius != displayState.lastDynamicRadius ||
      circleColor != displayState.lastDynamicColor) {
    tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, newRadius, circleColor);
    
    // update state tracking
    displayState.lastDynamicRadius = newRadius;
    displayState.lastDynamicColor = circleColor;
  }
}

// reset display to initial tuner state
void drawTunerInterface() {
  tft.fillScreen(TFT_BLACK);
  displayState.staticCirclesDrawn = false;
  displayState.lastDynamicRadius = 0;
  displayState.needsFullRedraw = false;
  drawStaticTunerElements();
}

// update tuner display with new pitch data
void updateTunerDisplay(const char *note, int cents, TuningResult *result,
                        bool hasAudio) {
  if (!displayMutex)
    return;

  // timestamp display update start
  if (result) {
    result->displayStartTime = esp_timer_get_time();
    printTiming("display start", result->bufferID, result->displayStartTime,
                result->captureTime);
  }

  // acquire display mutex with timeout
  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    drawStaticTunerElements();

    // determine color based on pitch accuracy
    uint16_t noteColor;

    if (abs(cents) <= CENTS_OVER_THRESHOLD) {
      noteColor = TFT_GREEN; 
    } else if (abs(cents) <= CENTS_OVER_THRESHOLD + 3) {
      noteColor = TFT_GREENYELLOW;
    } else if (abs(cents) <= CENTS_OVER_THRESHOLD + 5) {
      noteColor = TFT_YELLOW;
    } else if (abs(cents) <= CENTS_OVER_THRESHOLD + 7) {
      noteColor = TFT_ORANGE;
    } else {
      noteColor = TFT_RED;
    }

    // update text only when significantly changed
    if (strcmp(note, displayState.lastNoteName) != 0 ||
        fabs(cents - displayState.lastCentsOffset) > 1.0f) {

      // clear text area efficiently
      tft.fillRect(93, 97, 51, 44, TFT_BLACK);

      // draw cents value
      tft.setTextColor(noteColor);
      tft.setTextSize(2);
      char centsStr[8];
      snprintf(centsStr, sizeof(centsStr), "%d", cents);
      tft.drawCenterString(centsStr, 120, 97);

      // draw note name
      tft.setTextSize(3);
      tft.drawCenterString(note, 120, 120);

      // update state tracking
      strcpy(displayState.lastNoteName, note);
      displayState.lastCentsOffset = cents;
    }

    // update dynamic circle or clear if no audio
    if (hasAudio) {
      drawOptimizedCentsCircle(cents, noteColor);
    } else if (displayState.lastDynamicRadius > 0) {
      eraseDynamicCircle();
      displayState.lastDynamicRadius = 0;
    }

    xSemaphoreGive(displayMutex);

    // timestamp display update end
    if (result) {
      result->displayEndTime = esp_timer_get_time();
      printTiming("display end", result->bufferID, result->displayEndTime,
                  result->captureTime);
    }
  }
}

// main display update with queue management
void displayResult(const TuningResult *result, const AudioBuffer *buffer) {
  // handle invalid result
  if (!result || !result->validate()) {
    if (displayState.lastNoteName[0] != '\0') {
      updateTunerDisplay("--", 0.0f, nullptr, false);
      displayState.lastNoteName[0] = '\0';
    }
    return;
  }

  // skip display update if queue is backing up
  UBaseType_t queueDepth = uxQueueMessagesWaiting(audioQueue);
  bool skipDisplayUpdate = (queueDepth > 2);

  TuningResult mutableResult = *result;

  // determine if display update needed
  bool needsUpdate = false;

  if (strcmp(result->noteName, displayState.lastNoteName) != 0) {
    needsUpdate = true;
  }

  if (abs(result->centsOffset - displayState.lastCentsOffset) > 2) {
    needsUpdate = true;
  }

  // perform display update if needed and not skipping
  if ((needsUpdate || displayState.needsFullRedraw) && !skipDisplayUpdate) {
    updateTunerDisplay(result->noteName, result->centsOffset, &mutableResult,
                       true);
    displayState.lastUpdateTime = millis();

    strcpy(displayState.lastNoteName, result->noteName);
    displayState.lastCentsOffset = result->centsOffset;
  } else if (skipDisplayUpdate) {
    // fake timing for skipped update
    mutableResult.displayStartTime = esp_timer_get_time();
    mutableResult.displayEndTime = mutableResult.displayStartTime + 1000;
    printTiming("display skipped", mutableResult.bufferID,
                mutableResult.displayEndTime, mutableResult.captureTime);
  }

  // output timing summary
  if (buffer) {
    printTimingSummary(&mutableResult, buffer);
  }

  // update performance statistics
  uint64_t totalLatency = mutableResult.displayEndTime - result->captureTime;
  updateStats(totalLatency);
}