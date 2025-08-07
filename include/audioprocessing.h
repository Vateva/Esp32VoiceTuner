#ifndef AUDIOPROCESSING_H
#define AUDIOPROCESSING_H

#include "config.h"

// Audio processing function declarations
bool initI2S();
bool captureRealAudio(AudioBuffer* buffer);
int findCoarsePeriodYIN(const AudioBuffer* input, TuningResult* result);
bool yinAnalysis(const AudioBuffer* input, TuningResult* output, int hintedPeriod);
void convertFrequencyToNote(float frequency, char* noteName, size_t nameSize);
float calculateCentsOffset(float frequency);

#endif // AUDIOPROCESSING_H