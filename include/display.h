#ifndef DISPLAY_H
#define DISPLAY_H

#include "config.h"
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// LovyanGFX configuration for GC9A01 display
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Bus_SPI _bus_instance;

public:
  LGFX(void);
};

// Global display instance
extern LGFX tft;
extern DisplayState displayState;

// Display function declarations
void initDisplay();
void drawTunerInterface();
void drawStaticTunerElements();
void eraseDynamicCircle();
void drawPreciseCircle(int centerX, int centerY, int radius, uint16_t color);
void updateTunerDisplay(const char* note, float cents, TuningResult* result, bool hasAudio);
void drawOptimizedCentsCircle(float cents, uint16_t circleColor);
void displayResult(const TuningResult* result, const AudioBuffer* buffer);

#endif // DISPLAY_H