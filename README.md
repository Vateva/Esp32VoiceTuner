# esp32-s3-voice-tuner
Real-time voice/instrument tuner using ESP32-S3 with dual-core FreeRTOS, YIN pitch detection, and round GC9A01 display.

Main objective is to learn multithreading with true parallel processing across multiple cores and gain deeper experience with embedded systems development.

## Hardware Components
- ESP32-S3 Super Mini (or ESP32-S3 Zero)
- GC9A01 Round Display (240x240, 1.28")
- INMP441 MEMS Microphone
- TP4056 Charging Module
- 3.7V 3000mAh Li-Ion Battery
- 2x Momentary Push Buttons
- Power Switch

## Current Status
**Fully functional prototype** - all core features implemented and tested:
- [x] Dual-core audio processing (Core 0: capture, Core 1: analysis/display)
- [x] Two-pass YIN pitch detection with harmonic validation
- [x] Cascaded Butterworth filtering (highpass/lowpass)
- [x] Confidence-based exponential moving average smoothing
- [x] dB-based power management (80MHz detecting / 240MHz analyzing)
- [x] Runtime-configurable menu system with flash persistence
- [x] 10 musical scales with 2 note naming systems
- [x] Startup animation and sleep mode with animated zzz
- [x] 3D printed enclosure (files will be uploaded to repository)

## Pinout Configuration

### GC9A01 Display (SPI)
```cpp
TFT_SCK  = GPIO 1   // SPI clock
TFT_MOSI = GPIO 2   // SPI data out
TFT_CS   = GPIO 5   // chip select
TFT_DC   = GPIO 4   // data/command
TFT_RST  = GPIO 3   // reset
TFT_BLK  = GPIO 6   // backlight (PWM controlled)
```

### INMP441 Microphone (I2S)
```cpp
I2S_SCK = GPIO 13   // bit clock
I2S_WS  = GPIO 12   // word select (LRCK)
I2S_SD  = GPIO 11   // serial data
```

### Control Buttons
```cpp
BUTTON_DOWN   = GPIO 10  // menu navigation (with internal pullup)
BUTTON_SELECT = GPIO 9   // menu entry/parameter cycling (with internal pullup)
```

Buttons are active low (pressed = LOW, released = HIGH)

## Wiring and Assembly

### Breadboard
![tunerBreadBoard](https://github.com/user-attachments/assets/967493f7-d8b5-40e8-abf8-89485d1362c4)

### Soldered guts
![tunerSoloderedGuts](https://github.com/user-attachments/assets/8b4d445a-f375-49b6-b420-ef4f20fa73d1)

### Casing
![casing](https://github.com/user-attachments/assets/6567b193-762e-4b79-bc74-7838563c22dc)

### In casing
![inCasing](https://github.com/user-attachments/assets/f8b2cb8e-5c11-4834-ab48-861f482406df)

### Finished
![tunerFinished1](https://github.com/user-attachments/assets/ccd3fa0f-1f99-42ad-9edf-24b38e5d66ea)

**Power:**
- TP4056 handles battery charging via USB
- Power switch controls main power
- Battery provides portable operation


## Key Features

### True Dual-Core Processing
Leverages both ESP32-S3 cores for parallel processing:
- **Core 0 (Audio Task):** continuous I2S audio capture at 48kHz, dB level monitoring, power state management
- **Core 1 (Processing Task):** YIN pitch detection, display rendering, menu system, button handling
- FreeRTOS queues and mutexes ensure thread-safe communication
- Independent task priorities prevent blocking

### YIN Pitch Detection Algorithm
Two-pass implementation with harmonic validation:
- **Coarse Pass:** fast period estimation using 512 samples, quality gating at 0.5 threshold
- **Refined Pass:** focused search window (±40% configurable) around coarse estimate, parabolic interpolation for sub-sample precision
- **Harmonic Scoring:** validates fundamental by checking 2nd, 3rd, and 4th harmonics with weighted correlation
- **Multi-Candidate Ranking:** evaluates up to 8 period candidates using composite scoring (YIN quality 40%, harmonic content 50%, frequency bias 10%)

### Audio Preprocessing
Cascaded 2nd-order Butterworth IIR filters:
- **Highpass Filter:** removes DC offset and low frequency noise (60-120Hz configurable)
- **Lowpass Filter:** removes high frequency noise above vocal harmonics (4000-8000Hz configurable)
- **Audio Gain:** input amplification (1.0x to 10.0x configurable)

### Confidence-Based Smoothing
Multi-factor confidence calculation for adaptive filtering:
- **YIN Quality (35%):** exponential decay based on YIN difference function value
- **Harmonic Content (25%):** strength of detected harmonics in signal
- **Signal Strength (25%):** sigmoid-mapped RMS level
- **Frequency Stability (15%):** variance of recent 5 measurements
- Dynamic EMA alpha (0.25-0.85) scales smoothing based on overall confidence
- Automatic note change detection with immediate acquisition (6% frequency change threshold)

### Power Management
dB-based audio detection with dynamic CPU frequency scaling:
- **Detecting Mode:** 80MHz CPU, skips YIN analysis, monitors dB level only, displays animated sleep screen
- **Analyzing Mode:** 240MHz CPU, full pitch detection pipeline, normal tuner display
- Configurable activation threshold (-10 to -25dB), deactivation threshold (-20 to -40dB), silence timeout (2-10 seconds)
- Retriggerable timer prevents premature sleep during pauses

### Menu System
Comprehensive parameter configuration with flash persistence:
- **Tuning Parameters:** flat/sharp threshold (10-25 cents), smoothing level (1-5), scale selection (10 scales), root note (12 notes)
- **Audio Prefiltering:** highpass/lowpass cutoffs, audio gain
- **Display Settings:** brightness (PWM controlled 10-100%), cents display toggle, note naming (English/Solfège)
- **System Settings:** silence timeout, YIN search window, menu timeout, factory reset
- Two-button navigation (DOWN=navigate, SELECT=enter/cycle)
- Auto-save on exit with validation

### Musical Scales
Supports 10 different scales with configurable root note:
- Chromatic (all 12 notes)
- Major, Natural Minor, Harmonic Minor, Melodic Minor
- Major Pentatonic, Minor Pentatonic
- Blues, Dorian, Mixolydian
- Automatically snaps detected frequency to nearest scale note
- Calculates cents offset from perfect pitch within selected scale

### Visual Feedback
Round display optimized for tuning visualization:
- Three concentric reference circles (flat threshold, perfect pitch, sharp threshold)
- Dynamic circle expands/contracts based on cents offset with collision avoidance
- Color-coded accuracy: green (in tune) → yellow → orange → red (very out of tune)
- Large note name display with octave number
- Optional cents value display (toggleable)
- Startup animation with progressive letter reveal
- Sleep mode with floating animated zzz characters

## Technical Specifications

### Performance
- **Sample Rate:** 48kHz mono
- **Buffer Size:** 2048 samples (~43ms audio window)
- **Capture Interval:** 64ms (15.6 fps processing rate)
- **End-to-End Latency:** ~35ms (capture → analysis → display)
- **Frequency Range:** 40-1000Hz (code supports), 90-1000Hz (stable/verified)
- **YIN Period Range:** 32-600 samples (80-1500Hz theoretical at 48kHz)

## Known Limitations
- **Frequency Range:** while code theoretically supports 40-1000Hz, stable/reliable detection verified in 90-1000Hz range
- **Display Hardware:** code specifically written for GC9A01 240x240 round display, other displays require code modifications
- **Microphone Positioning:** optimal performance at ~15cm distance (varies with volume/gain settings)

## Build Environment
**PlatformIO Configuration:**
- Board: `esp32-s3-devkitc-1`
- Framework: Arduino-ESP32
- Library Dependencies: `lovyan03/LovyanGFX@^1.1.12`

Upload with default PlatformIO settings - no special configuration required.

## Videos

### Test video


https://github.com/user-attachments/assets/ada7eb09-d798-44a8-a253-f5996cc67c34

### Menu navigation and sleep



https://github.com/user-attachments/assets/7b2660a6-9329-4d96-aefc-cad8921d8ced

