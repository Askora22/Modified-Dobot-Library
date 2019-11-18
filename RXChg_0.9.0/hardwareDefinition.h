#include <Wire.h>
//INSERT ALL THE FOLLOWING INCLUDES AFTER INCLUDING WIRE LIB 
#include <LiquidCrystal_I2C.h>
#include <DS3231.h>
#include <buttons.h>

#define BACKLIGHT_PIN     (3)
#ifdef HW_VERSION
#define LED_ADDR (0x3F)   // My hardware
#else
#define LED_ADDR (0x27)
#endif

LiquidCrystal_I2C lcd(LED_ADDR, 2, 1, 0, 4, 5, 6, 7, BACKLIGHT_PIN, POSITIVE);

DS3231 clock;
RTCDateTime dt;



#ifdef HW_VERSION
// DEFINE ARDUINO PINS FOR THE NAVIGATION BUTTONS
#define UP_BUTTON_PIN       24                  // Change to pin 2
#define DOWN_BUTTON_PIN     25                  // Change to pin 4
#define LEFT_BUTTON_PIN     23                  // Change to pin 5
#define RIGHT_BUTTON_PIN    22                  // Change to pin 3
#define CONFIRM_BUTTON_PIN  26                  // Change to pin 6
#define ESCAPE_BUTTON_PIN   27                  // Change to pin 7
#define MODE_BUTTON_PIN     28
#define GO_BUTTON_PIN       29
#else
#define UP_BUTTON_PIN       2                  // Change to pin 2
#define DOWN_BUTTON_PIN     4                  // Change to pin 4
#define LEFT_BUTTON_PIN     3                  // Change to pin 5
#define RIGHT_BUTTON_PIN    5                  // Change to pin 3
#define CONFIRM_BUTTON_PIN  6                  // Change to pin 6
#define ESCAPE_BUTTON_PIN   7                  // Change to pin 7
#define MODE_BUTTON_PIN     29
#define GO_BUTTON_PIN       28
#define VAC_SWITCH          8
#endif

Button    BTU;
Button    BTD;
Button    BTL;
Button    BTR;
Button    BTE;
Button    BTC;
Button    BTM;
Button    BTG;
Button    VAC;

uint8_t numDI = 20;       // Should be 20
uint8_t numDO = 8;
#ifdef HW_VERSION
uint8_t inPin[] = {22, 23, 24, 25,
                   26, 27, 28, 29,
                   30, 31, 32, 33,
                   34, 35, 36, 37,
                   38, 39, 40, 41};
uint8_t outPin[] = {42, 43, 44, 45, 46, 47, 48, 49};
#else
uint8_t inPin[] = {2, 3, 4, 5,
                   6, 7, 28, 29,
                   30, 31, 32, 33,
                   34, 35, 36, 37,
                   38, 39, 8, 10};
uint8_t outPin[] = {42, 43, 44, 45, 46, 47, 48, 11};
#endif




uint8_t numAI = 2;
uint8_t aiPin[] = {A0, A1};

const char state_0[] PROGMEM = "    READY - IDLE    ";
const char state_1[] PROGMEM = " STATION - 1 CLEAN  ";
const char state_2[] PROGMEM = " STATION - 2 CLEAN  ";
const char state_3[] PROGMEM = " TRANSFER - 1 TO 2  ";
const char state_4[] PROGMEM = " STATION - 3 CLEAN  ";
const char state_5[] PROGMEM = " TRANSFER - 1 TO 3  ";
const char state_6[] PROGMEM = " TRANSFER - 2 TO 3  ";
const char state_7[] PROGMEM = " INVALID OPERATION  ";
const char state_8[] PROGMEM = "    READY - IDLE    ";
const char state_9[] PROGMEM = "STATION - 1 CASE CHG";
const char state_10[] PROGMEM = "STATION - 2 CASE CHG";
const char state_11[] PROGMEM = " TRANSFER - 2 TO 1  ";
const char state_12[] PROGMEM = "STATION - 3 CASE CHG";
const char state_13[] PROGMEM = " TRANSFER - 3 TO 1  ";
const char state_14[] PROGMEM = " TRANSFER - 3 TO 2  ";
const char state_15[] PROGMEM = " INVALID OPERATION  ";
const char state_16[] PROGMEM = "\nPRESS GO TO RESET\nROBOT!\n";
const char state_17[] PROGMEM = "*** PLEASE WAIT ***";
const char state_18[] PROGMEM = "CHG CASE & PRESS GO";
const char state_19[] PROGMEM = " OPERATION COMPLETE ";

const char *const states[] PROGMEM = {state_0, state_1, state_2, state_3, state_4, state_5, state_6, state_7, state_8, state_9, state_10, state_11, state_12, state_13, state_14, state_15, state_16, state_17, state_18, state_19};

const char message_0[] PROGMEM = " X-FAB TEXAS, INC.\n\n  RETICLE HANDLER\n  firmware: v";
const char message_1[] PROGMEM = "***DIGITAL INPUTS***\n";
const char message_2[] PROGMEM = "**DIGITAL OUTPUTS***\n";
const char message_3[] PROGMEM = "***ANALOG INPUTS****\n";
const char message_4[] PROGMEM = "**TEACH STATION X***\n L/R TO SELECT AXIS \nU/D TO SELECT SPEED \n     GO TO SAVE\n";
const char message_5[] PROGMEM = "\n   POSITION SAVED   \n\nHOLD CANCEL TO EXIT.\n";
const char message_6[] PROGMEM = "";

const char *const messages[] PROGMEM = {message_0, message_1, message_2, message_3, message_4, message_5};

const char tracem_0[] PROGMEM = "joy_stg[0] = ";
const char tracem_1[] PROGMEM = ", joy_stg[1] = ";
const char tracem_2[] PROGMEM = "JoyX MIN = ";
const char tracem_3[] PROGMEM = " JoyX MID = ";
const char tracem_4[] PROGMEM = " JoyX MAX = ";
const char tracem_5[] PROGMEM = ", JoyY MIN = ";
const char tracem_6[] PROGMEM = " JoyY MID = ";
const char tracem_7[] PROGMEM = " JoyY MAX = ";

const char *const traces[] PROGMEM = {tracem_0, tracem_1, tracem_2, tracem_3, tracem_4, tracem_5, tracem_6, tracem_7};
