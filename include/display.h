#ifndef DISPLAY_H
#define DISPLAY_H

#include "config.h"
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// lovyangfx driver configuration for gc9a01
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Bus_SPI _bus_instance;

public:
  LGFX(void);
};

// global display objects
extern LGFX tft;
extern DisplayState displayState;

// display control functions
void initDisplay();
void setDisplayBrightness(int brightnessPercent);  // brightness control function
void drawTunerInterface();
void drawStaticTunerElements();
void eraseDynamicCircle();
void updateTunerDisplay(const char* note, int cents, TuningResult* result, bool hasAudio);
void drawOptimizedCentsCircle(int cents, uint16_t circleColor);
void displayResult(const TuningResult* result, const AudioBuffer* buffer);
void displayDetectingMode();

#endif // DISPLAY_H