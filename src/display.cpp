#include "display.h"
#include "menu.h" // add menu.h include for menu state checking
#include "utilities.h"

// global display instance
LGFX tft;
DisplayState displayState = {"", 0.0f, 0, true, 0, 0, false, DETECTING, false};

// lovyangfx spi bus configuration
LGFX::LGFX(void) {
  // spi bus setup for display communication
  {
    auto cfg = _bus_instance.config();
    cfg.spi_host = SPI2_HOST;
    cfg.spi_mode = 0;
    cfg.freq_write = TFT_SPI_WRITE_FREQ;
    cfg.freq_read = TFT_SPI_READ_FREQ;
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
    cfg.panel_width = TFT_PANEL_WIDTH;
    cfg.panel_height = TFT_PANEL_HEIGHT;
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

// set display brightness using pwm control on backlight pin
void setDisplayBrightness(int brightnessPercent) {
  // clamp brightness to valid range
  if (brightnessPercent < 10)
    brightnessPercent = 10;
  if (brightnessPercent > 100)
    brightnessPercent = 100;

  // setup pwm for backlight control
  static bool pwmInitialized = false;
  if (!pwmInitialized) {
    // configure pwm channel for backlight
    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY,
              PWM_RESOLUTION);           // PWM setup for backlight
    ledcAttachPin(TFT_BLK, PWM_CHANNEL); // attach backlight pin to pwm channel
    pwmInitialized = true;
    safePrintf("pwm backlight control initialized\n");
  }

  // convert percentage to 8-bit pwm value (0-255)
  int pwmValue = (brightnessPercent * 255) / 100;

  // apply pwm to backlight pin
  ledcWrite(PWM_CHANNEL, pwmValue);

  safePrintf("display brightness set to %d%% (pwm=%d)\n", brightnessPercent,
             pwmValue);
}

// hardware initialization and test sequence
void initDisplay() {
  safePrintf("display backlight enabled\n");

  tft.init();
  tft.setRotation(0);

  safePrintf("display initialized successfully\n");
  drawTunerInterface();
}

// check if menu system is active and should block tuner display updates
bool isMenuActive() { return (menuSystem.currentMode != MENU_HIDDEN); }

// draw static reference circles and labels
void drawStaticTunerElements() {
  // prevent drawing tuner elements if menu is active
  if (isMenuActive()) {
    return;
  }

  if (displayState.staticCirclesDrawn) {
    return;
  }
  char tempStr[20];

  // use runtime parameter for threshold values instead of constants
  int centsOverThreshold = tunerParams.flatSharpThreshold;
  int centsUnderThreshold = -tunerParams.flatSharpThreshold;

  // outer circle (+cents threshold)
  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, DISPLAY_OUTER_RADIUS,
                 TFT_WHITE);
  tft.setTextSize(1);

  sprintf(tempStr, "%dc", centsOverThreshold);
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

  sprintf(tempStr, "%dc", centsUnderThreshold);
  tft.setTextColor(TFT_WHITE);
  tft.drawCenterString(tempStr, DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_INNER_RADIUS + 4);

  displayState.staticCirclesDrawn = true;
}

// clean up dynamic circle and restore static elements
void eraseDynamicCircle() {
  // prevent erasing if menu is active
  if (isMenuActive()) {
    return;
  }

  // erase previous dynamic circle
  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y,
                 displayState.lastDynamicRadius, TFT_BLACK);

  // redraw middle reference circle
  tft.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, DISPLAY_MIDDLE_RADIUS,
                 TFT_DARKGREY);

  tft.setTextSize(1);

  char tempStr[20];

  // use runtime parameters for threshold values
  int centsOverThreshold = tunerParams.flatSharpThreshold;
  int centsUnderThreshold = -tunerParams.flatSharpThreshold;

  // restore labels that may have been overwritten
  sprintf(tempStr, "%dc", centsOverThreshold);
  tft.setTextColor(TFT_WHITE);
  tft.drawCenterString(tempStr, DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_OUTER_RADIUS - 9);
  tft.setTextColor(TFT_GREEN);

  tft.drawCenterString("0c", DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_MIDDLE_RADIUS + 0);

  sprintf(tempStr, "%dc", centsUnderThreshold);
  tft.setTextColor(TFT_WHITE);
  tft.drawCenterString(tempStr, DISPLAY_CENTER_X,
                       DISPLAY_CENTER_Y - DISPLAY_INNER_RADIUS + 4);
}

// draw dynamic cents visualization circle
void drawOptimizedCentsCircle(int cents, uint16_t circleColor) {
  // prevent drawing dynamic circle if menu is active
  if (isMenuActive()) {
    return;
  }

  int newRadius;

  // use runtime parameters for threshold calculations
  int centsOverThreshold = tunerParams.flatSharpThreshold;
  int centsUnderThreshold = -tunerParams.flatSharpThreshold;

  // calculate scaling factors based on runtime threshold configuration
  float lowerScale = (float)(DISPLAY_MIDDLE_RADIUS - DISPLAY_INNER_RADIUS) /
                     abs(centsUnderThreshold);
  float upperScale = (float)(DISPLAY_OUTER_RADIUS - DISPLAY_MIDDLE_RADIUS) /
                     centsOverThreshold;

  // map cents to radius across four zones
  if (cents < centsUnderThreshold) {
    // zone 1: extreme flat (shrink inward from inner circle)
    float extraCents = abs(cents) - abs(centsUnderThreshold);
    newRadius =
        DISPLAY_INNER_RADIUS - (int)(extraCents * DYNAMIC_CIRCLE_EXTREME_SCALE);

    // clamp minimum radius
    if (newRadius < (DISPLAY_INNER_RADIUS - DYNAMIC_CIRCLE_MIN_RADIUS_OFFSET)) {
      newRadius = DISPLAY_INNER_RADIUS - DYNAMIC_CIRCLE_MIN_RADIUS_OFFSET;
    }
  } else if (cents < 0) {
    // zone 2: moderate flat (between inner and middle)
    newRadius = DISPLAY_MIDDLE_RADIUS + (int)(cents * lowerScale);
  } else if (cents <= centsOverThreshold) {
    // zone 3: moderate sharp (between middle and outer)
    newRadius = DISPLAY_MIDDLE_RADIUS + (int)(cents * upperScale);
  } else {
    // zone 4: extreme sharp (grow outward from outer circle)
    float extraCents = cents - centsOverThreshold;
    newRadius =
        DISPLAY_OUTER_RADIUS + (int)(extraCents * DYNAMIC_CIRCLE_EXTREME_SCALE);

    // clamp maximum radius
    if (newRadius > (DISPLAY_OUTER_RADIUS + DYNAMIC_CIRCLE_MAX_RADIUS_OFFSET)) {
      newRadius = DISPLAY_OUTER_RADIUS + DYNAMIC_CIRCLE_MAX_RADIUS_OFFSET;
    }
  }

  // avoid collision with static reference circles
  if (newRadius >= (DISPLAY_INNER_RADIUS - DYNAMIC_CIRCLE_COLLISION_BUFFER) &&
      newRadius <= (DISPLAY_INNER_RADIUS + DYNAMIC_CIRCLE_COLLISION_BUFFER)) {
    newRadius = (cents < centsUnderThreshold)
                    ? (DISPLAY_INNER_RADIUS - DYNAMIC_CIRCLE_COLLISION_BUFFER)
                    : (DISPLAY_INNER_RADIUS + DYNAMIC_CIRCLE_COLLISION_BUFFER);
  } else if (newRadius >=
                 (DISPLAY_OUTER_RADIUS - DYNAMIC_CIRCLE_COLLISION_BUFFER) &&
             newRadius <=
                 (DISPLAY_OUTER_RADIUS + DYNAMIC_CIRCLE_COLLISION_BUFFER)) {
    newRadius = (cents < centsOverThreshold)
                    ? (DISPLAY_OUTER_RADIUS - DYNAMIC_CIRCLE_COLLISION_BUFFER)
                    : (DISPLAY_OUTER_RADIUS + DYNAMIC_CIRCLE_COLLISION_BUFFER);
  } else if (newRadius >=
                 (DISPLAY_MIDDLE_RADIUS - DYNAMIC_CIRCLE_COLLISION_BUFFER) &&
             newRadius <=
                 (DISPLAY_MIDDLE_RADIUS + DYNAMIC_CIRCLE_COLLISION_BUFFER)) {
    newRadius = (cents < 0)
                    ? (DISPLAY_MIDDLE_RADIUS - DYNAMIC_CIRCLE_COLLISION_BUFFER)
                    : (DISPLAY_MIDDLE_RADIUS + DYNAMIC_CIRCLE_COLLISION_BUFFER);
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
  // only draw tuner interface if menu is not active
  if (isMenuActive()) {
    safePrintf("skipping tuner interface draw - menu is active\n");
    return;
  }

  tft.fillScreen(TFT_BLACK);
  displayState.staticCirclesDrawn = false;
  displayState.lastDynamicRadius = 0;
  displayState.needsFullRedraw = false;
  displayState.showingDetectingMode = false;

  drawStaticTunerElements();
}

// display detecting mode with slow zzz sleep animation
void displayDetectingMode() {
  // abort if menu is active - menu has display priority
  if (isMenuActive()) {
    return;
  }

  // acquire display mutex with timeout
  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE) {

    // slow animation timing - 1.3fps (750ms per frame)
    uint32_t currentTime = millis();
    uint32_t animationFrame = (currentTime / 750) % 24; // 24 frames total, 18 second cycle

    // only redraw if animation frame changed to prevent flickering
    static uint32_t lastAnimationFrame = 0xFFFFFFFF; // initialize to invalid value
    if (animationFrame != lastAnimationFrame) {
      lastAnimationFrame = animationFrame;

      // clear entire screen to black for clean sleep display
      tft.fillScreen(TFT_BLACK);

      // erase any dynamic circle state tracking
      if (displayState.lastDynamicRadius > 0) {
        displayState.lastDynamicRadius = 0;
      }

      // animation parameters for z character lifecycle
      const int baseY = 155;        // starting y position at bottom
      const int riseDistance = 100; // total upward travel distance
      const int zLifetime = 8;      // frames each z is visible (6 seconds)
      const int zDelay = 5;         // frames between z appearances (3.75 seconds)

      // discrete frame positions and sizes for smooth stepped animation
      const int frameY[] = {0, -10, -25, -40, -60, -75, -90, -100}; // upward positions
      const int frameSize[] = {1, 2, 3, 4, 5, 4, 3, 2}; // text size progression
      const uint16_t frameColors[] = {
          // color fade progression from dark to bright to dark
          TFT_DARKGREY, TFT_LIGHTGRAY, TFT_PINK, TFT_PINK,
          TFT_PINK, TFT_LIGHTGRAY, TFT_DARKGREY, TFT_DARKGREY};

      // draw up to 5 z characters at different animation stages
      for (int zIndex = 0; zIndex < 5; zIndex++) {
        // calculate this z's frame within its lifecycle with wraparound
        int zFrame = animationFrame - (zIndex * zDelay);
        if (zFrame < 0) zFrame += 24; // handle negative wraparound

        // skip z if outside its visible lifetime
        if (zFrame < 0 || zFrame >= zLifetime) continue;

        // apply discrete frame values without interpolation
        int yPos = baseY + frameY[zFrame];
        int textSize = frameSize[zFrame];
        uint16_t color = frameColors[zFrame];

        // subtle horizontal drift for natural floating effect
        int xOffset = 0;
        if (zFrame == 2 || zFrame == 6) xOffset = 1;
        if (zFrame == 4) xOffset = -1;

        int xPos = 120 + (zIndex - 2) * 10 + xOffset;

        // draw z character with calculated parameters
        tft.setTextSize(textSize);
        tft.setTextColor(color);
        tft.drawCenterString("z", xPos, yPos);
      }

      // draw status text at bottom of screen
      tft.setTextSize(1);
      tft.setTextColor(TFT_PINK);
      tft.drawCenterString("Asleep", 120, 204);
      tft.drawCenterString("but", 120, 212);
      tft.drawCenterString("listening...", 120, 220);

      // update state tracking for display management
      displayState.showingDetectingMode = true;
      displayState.powerState = currentPowerState;
      displayState.lastNoteName[0] = '\0'; // force refresh when switching back
    }

    xSemaphoreGive(displayMutex);
  }
}

// update tuner display with new pitch data
void updateTunerDisplay(const char *note, int cents, const TuningResult *result,
                        bool hasAudio) {
  // abort immediately if menu is active - menu has display priority
  if (isMenuActive()) {
    // optional debug output (only occasionally to avoid spam)
    static uint32_t menuSkipCounter = 0;
    if ((menuSkipCounter++ % 64) == 0) {
      safePrintf("skipping tuner display update - menu is active\n");
    }
    return;
  }

  if (!displayMutex)
    return;

  // acquire display mutex with timeout
  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    // double-check menu is not active after acquiring mutex
    if (isMenuActive()) {
      xSemaphoreGive(displayMutex);
      return;
    }

    // ensure we are not in detecting mode display
    if (displayState.showingDetectingMode) {
      drawTunerInterface(); // redraw full interface
      displayState.showingDetectingMode = false;
    }

    drawStaticTunerElements();

    // use runtime parameters for color thresholds
    int flatSharpThreshold = tunerParams.flatSharpThreshold;

    // determine color based on pitch accuracy using runtime parameters
    uint16_t noteColor;

    if (abs(cents) <= flatSharpThreshold) {
      noteColor = TFT_GREEN;
    } else if (abs(cents) <=
               flatSharpThreshold + CENTS_COLOR_THRESHOLD_GREENYELLOW) {
      noteColor = TFT_GREENYELLOW;
    } else if (abs(cents) <=
               flatSharpThreshold + CENTS_COLOR_THRESHOLD_YELLOW) {
      noteColor = TFT_YELLOW;
    } else if (abs(cents) <=
               flatSharpThreshold + CENTS_COLOR_THRESHOLD_ORANGE) {
      noteColor = TFT_ORANGE;
    } else {
      noteColor = TFT_RED;
    }

    // update text only when significantly changed
    if (strcmp(note, displayState.lastNoteName) != 0 ||
        abs(cents - displayState.lastCentsOffset) >= 1) {

      // clear text area efficiently
      tft.fillRect(TEXT_CLEAR_AREA_X, TEXT_CLEAR_AREA_Y, TEXT_CLEAR_AREA_WIDTH,
                   TEXT_CLEAR_AREA_HEIGHT, TFT_BLACK);

      // draw cents value ONLY if showCents parameter is enabled
      if (tunerParams.showCents) {
        tft.setTextColor(noteColor);
        tft.setTextSize(2);
        char centsStr[8];
        snprintf(centsStr, sizeof(centsStr), "%d", cents);
        tft.drawCenterString(centsStr, CENTS_TEXT_POS_X, CENTS_TEXT_POS_Y);
      }

      // draw note name
      tft.setTextSize(3);
      tft.setTextColor(noteColor);
      tft.drawCenterString(note, NOTE_TEXT_POS_X,
                           (tunerParams.showCents) ? NOTE_TEXT_POS_Y
                                                   : (NOTE_TEXT_POS_Y - 10));

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

    // update power state tracking
    displayState.powerState = currentPowerState;

    xSemaphoreGive(displayMutex);
  }
}

// main display update (receives already-smoothed results from audioprocessing)
void displayResult(const TuningResult *result, const AudioBuffer *buffer) {
  // abort immediately if menu is active - menu has display priority
  if (isMenuActive()) {
    return;
  }

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
  bool skipDisplayUpdate = (queueDepth > QUEUE_FULL_SKIP_THRESHOLD);

  // determine if display update needed (result is already smoothed)
  bool needsUpdate = false;

  if (strcmp(result->noteName, displayState.lastNoteName) != 0) {
    needsUpdate = true;
  }

  if (abs(result->centsOffset - displayState.lastCentsOffset) >
      CENTS_UPDATE_THRESHOLD) {
    needsUpdate = true;
  }

  // perform display update if needed and not skipping
  if ((needsUpdate || displayState.needsFullRedraw) && !skipDisplayUpdate) {
    updateTunerDisplay(result->noteName, result->centsOffset, result, true);
    displayState.lastUpdateTime = millis();

    strcpy(displayState.lastNoteName, result->noteName);
    displayState.lastCentsOffset = result->centsOffset;
  }
}