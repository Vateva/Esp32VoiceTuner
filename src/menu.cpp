#include "menu.h"
#include "display.h"
#include "utilities.h"
#include "audioprocessing.h"
#include <esp_timer.h>

/*
 * streamlined menu system with direct parameter cycling:
 * 
 * navigation:
 * - gpio 10 (down) = navigate down through menu items
 * - gpio 9 (select) = enter submenu OR cycle parameter values directly
 * 
 * menu structure:
 * main menu -> submenus -> parameters with live value display and direct cycling
 * changes apply when exiting main menu (except display settings which apply immediately)
 */

// global menu system state and parameters
MenuSystem menuSystem;
TunerParameters tunerParams;

// helper function to create int range parameters
Parameter createIntRangeParam(int* valuePtr, int minVal, int maxVal, int step, const char* suffix) {
    Parameter p;
    p.type = PARAM_INT_RANGE;
    p.intRange.valuePtr = valuePtr;
    p.intRange.minValue = minVal;
    p.intRange.maxValue = maxVal;
    p.intRange.stepSize = step;
    p.intRange.suffix = suffix;
    return p;
}

// helper function to create float range parameters
Parameter createFloatRangeParam(float* valuePtr, float minVal, float maxVal, float step, const char* suffix) {
    Parameter p;
    p.type = PARAM_FLOAT_RANGE;
    p.floatRange.valuePtr = valuePtr;
    p.floatRange.minValue = minVal;
    p.floatRange.maxValue = maxVal;
    p.floatRange.stepSize = step;
    p.floatRange.suffix = suffix;
    return p;
}

// helper function to create boolean parameters
Parameter createBoolParam(bool* valuePtr, const char* trueText, const char* falseText) {
    Parameter p;
    p.type = PARAM_BOOL;
    p.boolToggle.valuePtr = valuePtr;
    p.boolToggle.trueText = trueText;
    p.boolToggle.falseText = falseText;
    return p;
}

// submenu definitions (declare arrays first, then reference in main menu)

// tuning parameters submenu
MenuItem tuningSubmenu[] = {
    MenuItem("Flat/Sharp Threshold", createIntRangeParam(&tunerParams.flatSharpThreshold, 10, 25, 1, "c")),
    MenuItem("YIN Search Window", createIntRangeParam(&tunerParams.yinSearchWindow, 20, 40, 5, "%")),
    MenuItem("Smoothing Level", createIntRangeParam(&tunerParams.smoothingLevel, 1, 5, 1, "")),
    MenuItem("Back", nullptr)
};

// audio prefiltering submenu  
MenuItem audioSubmenu[] = {
    MenuItem("Highpass Cutoff", createIntRangeParam(&tunerParams.highpassCutoff, 60, 120, 10, "Hz")),
    MenuItem("Lowpass Cutoff", createIntRangeParam(&tunerParams.lowpassCutoff, 4000, 8000, 500, "Hz")),
    MenuItem("Back", nullptr)
};

// display submenu
MenuItem displaySubmenu[] = {
    MenuItem("Brightness", createIntRangeParam(&tunerParams.brightness, 10, 100, 10, "%")),
    MenuItem("Show Cents", createBoolParam(&tunerParams.showCents, "ON", "OFF")),
    MenuItem("Back", nullptr)
};

// system submenu
MenuItem systemSubmenu[] = {
    MenuItem("Silence Timeout", createIntRangeParam(&tunerParams.silenceTimeout, 2, 10, 1, "s")),
    MenuItem("DB Activation", createIntRangeParam(&tunerParams.dbActivation, -25, -10, 1, "dB")),
    MenuItem("DB Deactivation", createIntRangeParam(&tunerParams.dbDeactivation, -40, -20, 1, "dB")),
    MenuItem("Menu Timeout", createIntRangeParam(&tunerParams.menuTimeout, 2, 10, 1, "s")),
    MenuItem("Back", nullptr)
};

// main menu structure (references submenus defined above)
MenuItem mainMenu[] = {
    MenuItem("Tuning Parameters", tuningSubmenu, 4),
    MenuItem("Audio Prefiltering", audioSubmenu, 3),
    MenuItem("Display", displaySubmenu, 3),
    MenuItem("System", systemSubmenu, 5),
    MenuItem("Exit", menuActionExit)
};

// initialize menu system and gpio pins
void initMenuSystem() {
    // configure button pins with internal pullups
    pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
    pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
    
    // setup menu structure
    setupMenuStructure();
    
    // initialize menu system state
    menuSystem.currentMode = MENU_HIDDEN;
    menuSystem.buttonState = BUTTONS_RELEASED;
    menuSystem.lastButtonCheck = millis();
    menuSystem.lastMenuActivity = millis();
    menuSystem.needsRedraw = false;
    
    safePrintf("streamlined menu system initialized\n");
    safePrintf("navigation: down=%d (navigate), select=%d (enter/cycle)\n", 
              BUTTON_DOWN_PIN, BUTTON_SELECT_PIN);
    safePrintf("direct parameter cycling: select=cycle values immediately\n");
}

// read button state with active low logic
bool readButtonState(int pin) {
    // buttons are active low (pressed = low, released = high)
    if (BUTTON_ACTIVE_LOW) {
        return !digitalRead(pin);  // invert logic for active low
    } else {
        return digitalRead(pin);   // direct read for active high
    }
}

// main button checking function called from processing task
void checkButtonsAndUpdateMenu() {
    uint32_t currentTime = millis();
    
    // use different polling rates: slow for trigger, fast for navigation
    uint32_t pollInterval = (menuSystem.currentMode == MENU_HIDDEN) ? 
                           BUTTON_POLL_INTERVAL_MS : BUTTON_MENU_POLL_INTERVAL_MS;
    
    // only check buttons at configured interval 
    if ((currentTime - menuSystem.lastButtonCheck) < pollInterval) {
        return;
    }
    
    menuSystem.lastButtonCheck = currentTime;
    
    // read current button states
    bool downPressed = readButtonState(BUTTON_DOWN_PIN);
    bool selectPressed = readButtonState(BUTTON_SELECT_PIN);
    
    
    // global button state tracking to prevent cross-mode button bouncing
    static bool globalLastDownState = false;
    static bool globalLastSelectState = false;
    
    // handle menu states and navigation
    if (menuSystem.currentMode == MENU_HIDDEN) {
        // check for menu activation (select button pressed)
        if (selectPressed && !globalLastSelectState && !downPressed) {
            triggerMenuEntry();
        }
        
    } else {
        // menu navigation mode (MENU_MAIN or MENU_SUB)
        
        // detect button press edges for navigation with debouncing
        if (downPressed && !globalLastDownState && !selectPressed) {
            navigateMenuDown();
            menuSystem.lastMenuActivity = currentTime;
        }
        
        // detect select button press for item selection or parameter cycling with debouncing
        if (selectPressed && !globalLastSelectState && !downPressed) {
            selectCurrentMenuItem();
            menuSystem.lastMenuActivity = currentTime;
        }
        
        // check for menu timeout
        handleMenuTimeout();
    }
    
    // update global button state tracking (shared across all menu modes)
    globalLastDownState = downPressed;
    globalLastSelectState = selectPressed;
}


// activate menu system and switch display
void triggerMenuEntry() {
    safePrintf("menu activated - direct parameter cycling enabled\n");
    
    menuSystem.currentMode = MENU_MAIN;
    menuSystem.buttonState = BUTTONS_RELEASED;
    menuSystem.mainMenuIndex = 0;
    menuSystem.subMenuIndex = 0;
    menuSystem.needsRedraw = true;
    menuSystem.lastMenuActivity = millis();
    
    // add debounce delay to prevent same button press from triggering submenu entry
    menuSystem.lastButtonCheck = millis() + BUTTON_DEBOUNCE_MS;
    
    // acquire display and draw menu interface
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        drawMenuInterface();
        xSemaphoreGive(displayMutex);
    }
    
    safePrintf("menu activation debounced - ready for navigation\n");
}

// navigate menu selection downward (with wraparound)
void navigateMenuDown() {
    if (menuSystem.currentMode == MENU_MAIN) {
        // navigate main menu down with wraparound
        menuSystem.mainMenuIndex++;
        if (menuSystem.mainMenuIndex >= menuSystem.mainMenuCount) {
            menuSystem.mainMenuIndex = 0;
        }
        safePrintf("menu nav: main index = %d\n", menuSystem.mainMenuIndex);
    } else if (menuSystem.currentMode == MENU_SUB) {
        // navigate submenu down with wraparound
        menuSystem.subMenuIndex++;
        int submenuCount = menuSystem.mainMenu[menuSystem.currentMainItem].submenuCount;
        if (menuSystem.subMenuIndex >= submenuCount) {
            menuSystem.subMenuIndex = 0;
        }
        safePrintf("menu nav: sub index = %d\n", menuSystem.subMenuIndex);
    }
    
    // update display
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (menuSystem.currentMode == MENU_MAIN) {
            drawMainMenu();
        } else {
            drawSubMenu();
        }
        xSemaphoreGive(displayMutex);
    }
}

// select current menu item
void selectCurrentMenuItem() {
    if (menuSystem.currentMode == MENU_MAIN) {
        MenuItem* selectedItem = &menuSystem.mainMenu[menuSystem.mainMenuIndex];
        
        // check if this is the exit item
        if (strcmp(selectedItem->displayText, "Exit") == 0) {
            menuActionExit();
            return;
        }
        
        // check if item has submenu
        if (selectedItem->hasSubmenu) {
            // enter submenu
            menuSystem.currentMode = MENU_SUB;
            menuSystem.currentMainItem = menuSystem.mainMenuIndex;
            menuSystem.subMenuIndex = 0;
            safePrintf("entering submenu: %s\n", selectedItem->displayText);
            
            if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                drawSubMenu();
                xSemaphoreGive(displayMutex);
            }
        }
    } else if (menuSystem.currentMode == MENU_SUB) {
        MenuItem* currentSubmenu = menuSystem.mainMenu[menuSystem.currentMainItem].submenuItems;
        MenuItem* selectedItem = &currentSubmenu[menuSystem.subMenuIndex];
        
        // check if this is back item
        if (strcmp(selectedItem->displayText, "Back") == 0) {
            returnToMainMenu();
            return;
        }
        
        // check if this is a parameter item - cycle value directly
        if (selectedItem->hasParameter) {
            cycleParameterValue(selectedItem);
            
            // redraw submenu to show updated value
            if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                drawSubMenu();
                xSemaphoreGive(displayMutex);
            }
            return;
        }
        
        // execute action if present
        if (selectedItem->parameter.type == PARAM_ACTION && 
            selectedItem->parameter.action.actionFunction) {
            safePrintf("executing action: %s\n", selectedItem->displayText);
            selectedItem->parameter.action.actionFunction();
        }
    }
}

// cycle parameter value to next valid option
void cycleParameterValue(MenuItem* item) {
    if (!item || !item->hasParameter) return;
    
    Parameter& param = item->parameter;
    bool valueChanged = false;
    
    switch (param.type) {
        case PARAM_INT_RANGE:
            {
                int* value = param.intRange.valuePtr;
                *value += param.intRange.stepSize;
                if (*value > param.intRange.maxValue) {
                    *value = param.intRange.minValue; // wrap around
                }
                valueChanged = true;
                safePrintf("parameter cycled: %s = %d%s\n", 
                          item->displayText, *value, param.intRange.suffix);
            }
            break;
            
        case PARAM_FLOAT_RANGE:
            {
                float* value = param.floatRange.valuePtr;
                *value += param.floatRange.stepSize;
                if (*value > param.floatRange.maxValue) {
                    *value = param.floatRange.minValue; // wrap around
                }
                valueChanged = true;
                safePrintf("parameter cycled: %s = %.1f%s\n", 
                          item->displayText, *value, param.floatRange.suffix);
            }
            break;
            
        case PARAM_BOOL:
            {
                bool* value = param.boolToggle.valuePtr;
                *value = !(*value); // toggle
                valueChanged = true;
                safePrintf("parameter toggled: %s = %s\n", 
                          item->displayText, (*value) ? 
                          param.boolToggle.trueText : param.boolToggle.falseText);
            }
            break;
            
        default:
            safePrintf("cannot cycle this parameter type\n");
            return;
    }
    
    // apply immediate changes for display parameters
    if (valueChanged && strcmp(menuSystem.mainMenu[menuSystem.currentMainItem].displayText, "Display") == 0) {
        applyDisplayChanges();
    }
}

// return from submenu to main menu
void returnToMainMenu() {
    safePrintf("returning to main menu\n");
    menuSystem.currentMode = MENU_MAIN;
    
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        drawMainMenu();
        xSemaphoreGive(displayMutex);
    }
}

// exit menu system and apply changes
void menuActionExit() {
    safePrintf("applying parameter changes and exiting menu\n");
    
    // apply all non-display parameter changes here
    applyParameterChanges();
    
    exitMenuSystem();
}

// exit menu system and return to tuner
void exitMenuSystem() {
    safePrintf("exiting menu system\n");
    
    // acquire display mutex for clean transition
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // set menu to hidden state BEFORE clearing screen
        menuSystem.currentMode = MENU_HIDDEN;
        menuSystem.buttonState = BUTTONS_RELEASED;
        
        // force complete screen clear to remove all menu remnants
        tft.fillScreen(TFT_BLACK);
        safePrintf("screen cleared - removing menu display\n");
        
        // now draw fresh tuner interface
        drawTunerInterface();
        
        xSemaphoreGive(displayMutex);
    } else {
        // fallback if mutex fails - still update menu state
        menuSystem.currentMode = MENU_HIDDEN;
        menuSystem.buttonState = BUTTONS_RELEASED;
    }
    
    safePrintf("menu system fully exited - tuner interface restored\n");
}

// handle menu timeout (auto-exit if inactive)
void handleMenuTimeout() {
    uint32_t currentTime = millis();
    uint32_t timeoutMs = tunerParams.menuTimeout * 1000; // use runtime parameter
    
    if ((currentTime - menuSystem.lastMenuActivity) > timeoutMs) {
        safePrintf("menu timeout - auto exit\n");
        exitMenuSystem();
    }
}

// draw complete menu interface
void drawMenuInterface() {
    clearMenuArea();
    
    if (menuSystem.currentMode == MENU_MAIN) {
        drawMainMenu();
    } else if (menuSystem.currentMode == MENU_SUB) {
        drawSubMenu();
    }
}

// clear display area for menu
void clearMenuArea() {
    tft.fillScreen(MENU_BACKGROUND_COLOR);
}

// draw main menu with selection indicator
void drawMainMenu() {
    clearMenuArea();
    drawMenuTitle("MAIN MENU");
    
    for (int i = 0; i < menuSystem.mainMenuCount; i++) {
        bool isSelected = (i == menuSystem.mainMenuIndex);
        int yPos = MENU_ITEM_START_Y + (i * MENU_ITEM_HEIGHT);
        drawMenuItem(i, isSelected, menuSystem.mainMenu[i].displayText, yPos);
    }
}

// draw submenu with selection indicator and parameter values
void drawSubMenu() {
    clearMenuArea();
    
    MenuItem* currentMainItem = &menuSystem.mainMenu[menuSystem.currentMainItem];
    drawMenuTitle(currentMainItem->displayText);
    
    MenuItem* submenu = currentMainItem->submenuItems;
    int submenuCount = currentMainItem->submenuCount;
    
    for (int i = 0; i < submenuCount; i++) {
        bool isSelected = (i == menuSystem.subMenuIndex);
        int yPos = MENU_ITEM_START_Y + (i * MENU_ITEM_HEIGHT);
        drawMenuItem(i, isSelected, submenu[i].displayText, yPos, &submenu[i]);
    }
}

// draw individual menu item with selection highlighting and parameter values
void drawMenuItem(int index, bool isSelected, const char* text, int yPosition, MenuItem* item) {
    if (!text) return;
    
    tft.setTextSize(1);  // use smaller text size for better fit
    
    if (isSelected) {
        // highlight selected item
        tft.setTextColor(MENU_SELECTED_COLOR);
        tft.drawString(">", 20, yPosition);  // moved selection indicator closer to edge
        tft.drawString(text, 35, yPosition);  // adjusted text position
    } else {
        // normal item
        tft.setTextColor(MENU_ITEM_COLOR);
        tft.drawString(text, 35, yPosition);  // consistent text position
    }
    
    // draw parameter value if this is a parameter item
    if (item && item->hasParameter) {
        char valueBuffer[32];
        formatParameterValue(item->parameter, valueBuffer, sizeof(valueBuffer));
        
        tft.setTextColor(isSelected ? TFT_YELLOW : TFT_DARKGREY);
        // position parameter values on right side with proper spacing
        tft.drawString(valueBuffer, 180, yPosition);
    }
}

// draw menu title at top of screen
void drawMenuTitle(const char* title) {
    if (!title) return;
    
    tft.setTextColor(MENU_TITLE_COLOR);
    tft.setTextSize(1);  // use smaller text size for title too
    tft.drawCenterString(title, 120, 30);  // moved title up slightly
}

// format parameter value for display
void formatParameterValue(const Parameter& param, char* buffer, size_t bufferSize) {
    if (!buffer || bufferSize < 16) return;
    
    switch (param.type) {
        case PARAM_INT_RANGE:
            snprintf(buffer, bufferSize, "%d%s", 
                    *(param.intRange.valuePtr), param.intRange.suffix);
            break;
            
        case PARAM_FLOAT_RANGE:
            snprintf(buffer, bufferSize, "%.1f%s", 
                    *(param.floatRange.valuePtr), param.floatRange.suffix);
            break;
            
        case PARAM_BOOL:
            {
                bool value = *(param.boolToggle.valuePtr);
                snprintf(buffer, bufferSize, "%s", 
                        value ? param.boolToggle.trueText : param.boolToggle.falseText);
            }
            break;
            
        default:
            strcpy(buffer, "");
            break;
    }
}

// apply display parameter changes immediately
void applyDisplayChanges() {
    safePrintf("applying display changes: brightness=%d%%, showCents=%s\n",
              tunerParams.brightness, tunerParams.showCents ? "ON" : "OFF");
    
    // apply brightness control using pwm on backlight pin
    setDisplayBrightness(tunerParams.brightness);
    
    // showCents flag is checked in display.cpp updateTunerDisplay() function
    // no immediate action needed - display will update on next frame
}

// apply all parameter changes when exiting menu
void applyParameterChanges() {
    safePrintf("applying all parameter changes:\n");
    
    // tuning parameters
    safePrintf("  flat/sharp threshold: %dc\n", tunerParams.flatSharpThreshold);
    safePrintf("  yin search window: %d%%\n", tunerParams.yinSearchWindow);
    safePrintf("  smoothing level: %d\n", tunerParams.smoothingLevel);
    
    // audio prefiltering - recalculate filter coefficients
    safePrintf("  highpass cutoff: %dHz\n", tunerParams.highpassCutoff);
    safePrintf("  lowpass cutoff: %dHz\n", tunerParams.lowpassCutoff);
    recalculateAudioFilters();
    
    // system parameters - these are used directly by checking tunerParams
    safePrintf("  silence timeout: %ds\n", tunerParams.silenceTimeout);
    safePrintf("  db activation: %ddB\n", tunerParams.dbActivation);
    safePrintf("  db deactivation: %ddB\n", tunerParams.dbDeactivation);
    safePrintf("  menu timeout: %ds\n", tunerParams.menuTimeout);
    
    // display parameters were already applied immediately
    safePrintf("  brightness: %d%% (applied immediately)\n", tunerParams.brightness);
    safePrintf("  show cents: %s (applied immediately)\n", tunerParams.showCents ? "ON" : "OFF");
    
    safePrintf("all parameters applied successfully\n");

    //save to flash after applying changes
    saveParametersToFlash();
}

// save all parameters to esp32 flash memory
void saveParametersToFlash() {
    Preferences preferences;
    
    if (!preferences.begin("tuner_config", false)) {
        safePrintf("ERROR: failed to open preferences for writing\n");
        return;
    }
    
    // tuning parameters
    preferences.putInt("flatSharpThr", tunerParams.flatSharpThreshold);
    preferences.putInt("yinWindow", tunerParams.yinSearchWindow);
    preferences.putInt("smoothLevel", tunerParams.smoothingLevel);
    
    // audio prefiltering
    preferences.putInt("hpCutoff", tunerParams.highpassCutoff);
    preferences.putInt("lpCutoff", tunerParams.lowpassCutoff);
    
    // display settings
    preferences.putInt("brightness", tunerParams.brightness);
    preferences.putBool("showCents", tunerParams.showCents);
    
    // system parameters
    preferences.putInt("silenceTimeout", tunerParams.silenceTimeout);
    preferences.putInt("dbActivation", tunerParams.dbActivation);
    preferences.putInt("dbDeactivation", tunerParams.dbDeactivation);
    preferences.putInt("menuTimeout", tunerParams.menuTimeout);
    
    preferences.end();
    
    safePrintf("parameters saved to flash memory\n");
}

// load all parameters from esp32 flash memory with fallback defaults
void loadParametersFromFlash() {
    Preferences preferences;
    
    if (!preferences.begin("tuner_config", true)) {
        safePrintf("no saved parameters found - using defaults\n");
        return;
    }
    
    // load parameters with current values as fallbacks
    tunerParams.flatSharpThreshold = preferences.getInt("flatSharpThr", tunerParams.flatSharpThreshold);
    tunerParams.yinSearchWindow = preferences.getInt("yinWindow", tunerParams.yinSearchWindow);
    tunerParams.smoothingLevel = preferences.getInt("smoothLevel", tunerParams.smoothingLevel);
    
    tunerParams.highpassCutoff = preferences.getInt("hpCutoff", tunerParams.highpassCutoff);
    tunerParams.lowpassCutoff = preferences.getInt("lpCutoff", tunerParams.lowpassCutoff);
    
    tunerParams.brightness = preferences.getInt("brightness", tunerParams.brightness);
    tunerParams.showCents = preferences.getBool("showCents", tunerParams.showCents);
    
    tunerParams.silenceTimeout = preferences.getInt("silenceTimeout", tunerParams.silenceTimeout);
    tunerParams.dbActivation = preferences.getInt("dbActivation", tunerParams.dbActivation);
    tunerParams.dbDeactivation = preferences.getInt("dbDeactivation", tunerParams.dbDeactivation);
    tunerParams.menuTimeout = preferences.getInt("menuTimeout", tunerParams.menuTimeout);
    
    preferences.end();
    
    // validate loaded parameters are within acceptable ranges
    if (validateParameterRanges()) {
        safePrintf("parameters loaded from flash memory\n");
        safePrintf("loaded: flatSharp=%dc, yinWindow=%d%%, smoothing=%d\n", 
                  tunerParams.flatSharpThreshold, tunerParams.yinSearchWindow, tunerParams.smoothingLevel);
        safePrintf("loaded: hp=%dHz, lp=%dHz, brightness=%d%%\n",
                  tunerParams.highpassCutoff, tunerParams.lowpassCutoff, tunerParams.brightness);
    } else {
        safePrintf("WARNING: loaded parameters out of range - using defaults\n");
    }
}

// validate all parameters are within acceptable ranges
bool validateParameterRanges() {
    bool valid = true;
    
    // check tuning parameters
    if (tunerParams.flatSharpThreshold < 10 || tunerParams.flatSharpThreshold > 25) {
        tunerParams.flatSharpThreshold = 15;
        valid = false;
    }
    if (tunerParams.yinSearchWindow < 20 || tunerParams.yinSearchWindow > 40) {
        tunerParams.yinSearchWindow = 40;
        valid = false;
    }
    if (tunerParams.smoothingLevel < 1 || tunerParams.smoothingLevel > 5) {
        tunerParams.smoothingLevel = 3;
        valid = false;
    }
    
    // check audio parameters
    if (tunerParams.highpassCutoff < 60 || tunerParams.highpassCutoff > 120) {
        tunerParams.highpassCutoff = 60;
        valid = false;
    }
    if (tunerParams.lowpassCutoff < 4000 || tunerParams.lowpassCutoff > 8000) {
        tunerParams.lowpassCutoff = 6000;
        valid = false;
    }
    
    // check display parameters
    if (tunerParams.brightness < 10 || tunerParams.brightness > 100) {
        tunerParams.brightness = 100;
        valid = false;
    }
    
    // check system parameters
    if (tunerParams.silenceTimeout < 2 || tunerParams.silenceTimeout > 10) {
        tunerParams.silenceTimeout = 5;
        valid = false;
    }
    if (tunerParams.dbActivation < -25 || tunerParams.dbActivation > -10) {
        tunerParams.dbActivation = -15;
        valid = false;
    }
    if (tunerParams.dbDeactivation < -40 || tunerParams.dbDeactivation > -20) {
        tunerParams.dbDeactivation = -25;
        valid = false;
    }
    if (tunerParams.menuTimeout < 2 || tunerParams.menuTimeout > 10) {
        tunerParams.menuTimeout = 10;
        valid = false;
    }
    
    return valid;
}

// factory reset all parameters to defaults and save
void factoryResetParameters() {
    safePrintf("performing factory reset of all parameters\n");
    
    // reset to constructor defaults
    tunerParams = TunerParameters();
    
    // save defaults to flash
    saveParametersToFlash();
    
    // apply changes immediately
    applyParameterChanges();
    
    safePrintf("factory reset complete - all parameters restored to defaults\n");
}

// setup menu structure with main menu and submenus
void setupMenuStructure() {
    menuSystem.mainMenu = mainMenu;
    menuSystem.mainMenuCount = sizeof(mainMenu) / sizeof(MenuItem);
    
    safePrintf("menu structure setup: %d main items with comprehensive parameters\n", 
              menuSystem.mainMenuCount);
}