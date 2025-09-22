#ifndef MENU_H
#define MENU_H

#include <Preferences.h>
#include "config.h"


// menu system operational states
enum MenuMode {
    MENU_HIDDEN,          // menu not active, normal tuner operation
    MENU_MAIN,           // showing main menu level
    MENU_SUB            // showing submenu level
};

// parameter types for menu items
enum ParameterType {
    PARAM_NONE,          // not a parameter (submenu or action)
    PARAM_INT_RANGE,     // integer with min/max range
    PARAM_FLOAT_RANGE,   // float with min/max range
    PARAM_BOOL,          // boolean toggle
    PARAM_ACTION         // executes function when selected
};

// parameter definition structure
struct Parameter {
    ParameterType type;
    union {
        struct {
            int* valuePtr;       // pointer to the actual variable
            int minValue;        // minimum allowed value
            int maxValue;        // maximum allowed value
            int stepSize;        // increment step
            const char* suffix;  // unit suffix (e.g., "c", "Hz", "%")
        } intRange;
        struct {
            float* valuePtr;     // pointer to the actual variable
            float minValue;      // minimum allowed value
            float maxValue;      // maximum allowed value
            float stepSize;      // increment step
            const char* suffix;  // unit suffix
        } floatRange;
        struct {
            bool* valuePtr;      // pointer to the actual variable
            const char* trueText;   // text for true value
            const char* falseText;  // text for false value
        } boolToggle;
        struct {
            void (*actionFunction)(); // function to execute
        } action;
    };
    
    Parameter() : type(PARAM_NONE) {}
};

// individual menu item structure
struct MenuItem {
    const char* displayText;     // text shown on screen
    bool hasSubmenu;            // true if this item opens a submenu
    bool hasParameter;          // true if this item edits a parameter
    Parameter parameter;        // parameter definition
    MenuItem* submenuItems;     // pointer to submenu array (null if parameter/action)
    int submenuCount;          // number of items in submenu
    
    // constructor for action items (no submenu, no parameter)
    MenuItem(const char* text, void (*action)()) 
        : displayText(text), hasSubmenu(false), hasParameter(false), 
          submenuItems(nullptr), submenuCount(0) {
        parameter.type = PARAM_ACTION;
        parameter.action.actionFunction = action;
    }
    
    // constructor for submenu items
    MenuItem(const char* text, MenuItem* subItems, int subCount)
        : displayText(text), hasSubmenu(true), hasParameter(false),
          submenuItems(subItems), submenuCount(subCount) {
        parameter.type = PARAM_NONE;
    }
    
    // constructor for parameter items
    MenuItem(const char* text, Parameter param)
        : displayText(text), hasSubmenu(false), hasParameter(true),
          parameter(param), submenuItems(nullptr), submenuCount(0) {}
    
    // default constructor
    MenuItem() : displayText(nullptr), hasSubmenu(false), hasParameter(false),
                submenuItems(nullptr), submenuCount(0) {}
};

// tuner parameters structure (current values)
struct TunerParameters {
    // tuning parameters
    int flatSharpThreshold;      // ±cents threshold (10-25)
    int yinSearchWindow;         // yin search window percentage (20-40)
    int smoothingLevel;          // smoothing level (1-5)
    int scaleType;               // ScaleType enum value (0-9)
    int rootNote;                // ChromaticNote enum value (0-11)

    // audio prefiltering
    int highpassCutoff;          // highpass cutoff hz (60-120)
    int lowpassCutoff;           // lowpass cutoff hz (4000-8000)
    
    // display
    int brightness;              // brightness percentage (10-100)
    bool showCents;              // show numerical cents value
    int noteNaming;              // NoteNamingSystem enum value (0-1)
    // system
    int silenceTimeout;          // silence timeout seconds (2-10)
    int dbActivation;            // db activation threshold (-10 to -25)
    int dbDeactivation;          // db deactivation threshold (-20 to -40)
    int menuTimeout;             // menu timeout seconds (2-10)
    
    TunerParameters() {
        // initialize with current config.h values
        flatSharpThreshold = 15;  // default ±15 cents
        yinSearchWindow = 40;     // default 40% from YIN_SEARCH_WINDOW
        smoothingLevel = 3;       // middle smoothing level
        scaleType = 0;            // SCALE_CHROMATIC (maintain current behavior)
        rootNote = 0;             // NOTE_C (c major/c minor scales)

        highpassCutoff = 60;      // from HIGHPASS_CUTOFF_HZ
        lowpassCutoff = 6000;     // from LOWPASS_CUTOFF_HZ
        
        brightness = 100;         // full brightness
        showCents = true;         // show cents by default
        noteNaming = 0;           // NAMING_ENGLISH (default)
        
        silenceTimeout = 5;       // from SILENCE_TIMEOUT_MS / 1000
        dbActivation = -15;       // from DB_ACTIVATION_THRESHOLD
        dbDeactivation = -25;     // from DB_DEACTIVATION_THRESHOLD  
        menuTimeout = 10;         // from MENU_TIMEOUT_MS / 1000
    }
};

// complete menu system state
struct MenuSystem {
    MenuMode currentMode;           // current menu operational state
    uint32_t buttonPressTime;       // timestamp when both buttons first pressed
    uint32_t lastButtonCheck;       // timestamp of last button poll
    uint32_t lastMenuActivity;      // timestamp of last menu interaction
    
    // navigation state
    int mainMenuIndex;             // current selection in main menu
    int subMenuIndex;              // current selection in submenu  
    int currentMainItem;           // which main item opened current submenu
    
    // menu structure
    MenuItem* mainMenu;            // pointer to main menu array
    int mainMenuCount;             // number of main menu items
    
    // display state  
    bool needsRedraw;              // full menu redraw required
    
    MenuSystem() : currentMode(MENU_HIDDEN),
                  buttonPressTime(0), lastButtonCheck(0), lastMenuActivity(0),
                  mainMenuIndex(0), subMenuIndex(0), currentMainItem(0),
                  mainMenu(nullptr), mainMenuCount(0), needsRedraw(false) {}
};

// global menu system instance and parameters
extern MenuSystem menuSystem;
extern TunerParameters tunerParams;

// button polling and detection functions
void initMenuSystem();
void checkButtonsAndUpdateMenu();
bool readButtonState(int pin);
void updateButtonStateMachine();
void handleMenuTimeout();

// menu navigation functions  
void triggerMenuEntry();
void navigateMenuDown(); 
void selectCurrentMenuItem();
void exitMenuSystem();
void returnToMainMenu();

// parameter editing functions
void cycleParameterValue(MenuItem* item);

// parameter persistence functions
void saveParametersToFlash();
void loadParametersFromFlash();
bool validateParameterRanges();

// menu display functions
void drawMenuInterface();
void drawMainMenu();
void drawSubMenu();
void clearMenuArea();
void drawMenuItem(int index, bool isSelected, const char* text, int yPosition, MenuItem* item = nullptr);
void drawMenuTitle(const char* title);

// parameter value formatting
void formatParameterValue(const Parameter& param, char* buffer, size_t bufferSize);

// menu action functions
void menuActionExit();

// parameter application functions
void applyDisplayChanges();
void applyParameterChanges();

// menu structure initialization
void setupMenuStructure();

#endif // MENU_H