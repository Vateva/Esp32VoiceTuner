#ifndef AUDIOPROCESSING_H
#define AUDIOPROCESSING_H

#include "config.h"

// audio filtering functions
void initAudioFilters();
void recalculateAudioFilters();  // recalculate filters when parameters change
void calculateButterworthCoefficients(float cutoffHz, float sampleRate, bool isHighpass, IIRFilter* filter);
float applyIIRFilter(float input, IIRFilter* filter);
void applyPrefiltering(float* samples, uint16_t sampleCount);

// audio capture and pitch analysis functions
bool initI2S();
bool captureRealAudio(AudioBuffer* buffer);
int findCoarsePeriodYIN(const AudioBuffer* input, TuningResult* result);
bool yinAnalysis(const AudioBuffer* input, TuningResult* output, int hintedPeriod);
void convertFrequencyToNote(float frequency, char* noteName, size_t nameSize);
int calculateCentsOffset(float frequency);

#endif // AUDIOPROCESSING_H