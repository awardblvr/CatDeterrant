/*
  Cat Deterrent

  Small nozzle (Solder paste small-syringe tip) with small pump on a rotating servo
  to sweep a very small stream of distalled water if the motion detector detects
  the cat on the bed. Set for D1 Mini Pro (ESP8266)

  Calibration of the sweep range is donoe by interacting with the single Button and

  Push and hold button: 3 sec
    flashing LED yellow
    Set Sweep to full CW position (this is "zero"). Pause 5 sec.
    flashing light to green
    begin reading pot and moving arm to the corresponding position
  Timeout set to 2 minutes OR
  When button is short pressed;
    Set color to Orange
    write to flash
    3 quick orange flashes
  reset sweep to 0
  return to upper detect and fire mode

  short button press
      flashing LED yellow
      set sweep to current pot position
  Timeout set to 10 sec
        return to upper detect and fire mode
  If LONG press
    Squirt as long as button is held
  when reease

*/
#include "debug_print.h"
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <EasyButton.h>
#include <vector>
#include <forward_list>
#include <movingAvg.h>                  // https://github.com/JChristensen/movingAvg
//#include <EEPROM.h>
#include <ESP_EEPROM.h>
#include <TaskManagerIO.h>

//-------------------- HW PIN ASSIGNMENTS --------------------

#define SERVO_PIN      D1
#define SQUIRT_PIN     D2
#define BUTTON_IN_PIN  D5
#define NEOPIXEL_PIN   D7
#define PIR_IN_PIN     D6
#define POT_IN_PIN     A0

//-------------------- EEPROM  LOCATIONS  --------------------
#define EE_ADDR_DEBUG_MODE      0x00
#define EE_ADDR_SWEEP_MAX_POS   0x01   // int16_t  2 bytes
#define EE_ADDR_SWEEP_PASSES    0x03   // int16_t  2 bytes

#define DEFAULT_SWEEP_MAX_POS  180
#define DEFAULT_SWEEP_PASSES  1

enum _EE_init_mode { NORMAL=0, FORCE_DEFAULTS, FORCE_SWEEP_MAX_POS, FORCE_SWEEP_PASSES, FORCE_ALL };
const char *EE_init_mode_str[] = {"NORMAL","FORCE_DEFAULTS","FORCE_SWEEP_MAX_POS","FORCE_SWEEP_PASSES","FORCE_ALL"};
enum _EE_init_mode EE_init_mode = NORMAL;

//-------------------- BUTTON SUPPORT --------------------
EasyButton litButton(BUTTON_IN_PIN);

//-------------------- SERVO SUPPORT --------------------
Servo spraySweep;

int16_t sweepMaxPos=0;    // 2-bytes
int intraStepDelayMillis = 0;
int servoStep = 2;

//-------------------- NEOPIXEL SUPPORT --------------------
#define PIXELS_IN_USE 2

Adafruit_NeoPixel pxl(PIXELS_IN_USE, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define pixelButton     0
#define pixelExternal   1


#define RGB_BRIGHTNESS 64 // Change white brightness (max 255)

//typedef struct {
//    uint32_t color0;
//    uint32_t color1;
//    uint32_t durationMillis;
//    uint8_t pixelSelect;
//    int16_t flashes;
//    uint8_t current_color;    // always 0 (color0) or 1 (color1)
//    uint32_t nextChangeMillis;
//} PixelFlashEvent_t;

typedef struct {
    uint32_t color0;
    uint32_t color1;
    int16_t flashes;
    uint8_t current_color;    // always 0 (color0) or 1 (color1)
} PixelFlashEvent_t;
PixelFlashEvent_t pixelAction[PIXELS_IN_USE];
#define PIXEL_TRIG_NOW 1
#define PIXEL_FOREVER -1

// ht_state = RGB_BRIGHTNESS;
uint8_t  bright_state = 64;

#define RGB_FULL_BRIGHTNESS 255

// Neopixel colors
#define NeoBlack    0x000000
#define NeoWhite    0xFFFFFF
#define NeoRed      0xFF0000
#define NeoGreen    0x00FF00
#define NeoDimGreen 0x000F00
#define NeoMinGreen 0x000100
#define NeoBlue     0x0000FF
#define NeoYellow   0xFFFF00
#define NeoMagenta  0xFF00FF
#define NeoPurple   0x800080
#define NeoDarkPurple 0x2c0222
#define NeoOrange   0xFF8C00
#define NeoLime     0x00FF00
#define NeoNavyBlue 0x000080
#define NeoGray     0x696969
#define NeoSilver   0xC0C0C0
#define NeoBrown    0x8B4513

//-------------------- RUNTINE VARIABLES --------------------

static uint32_t tick50=0, tick1000=0, tick10000=0;  // future:, tick128=0, tick30000=0;
bool squirtOK = true;
uint32_t timeoutDurationMillis=0;   // inactivity duration timeout
uint32_t nextTimeout=0;
int16_t currentSweepPass=0;
int16_t sweepPasses=0;            // Updated from EE @ startup
int16_t calibrateSweepPasses=0;
int16_t currentSweep;
bool motionDetectorCleared=true;
enum _currentSweepPos { ZERO, FULL };
enum _currentSweepPos currentSweepPos = ZERO;
enum _mode { RUN, BEGIN_CALIBRATE_SWEEP, CALIBRATE_SWEEP, CALIBRATE_PASSES };
enum _mode mode = RUN;
enum _mode previousMode = (enum _mode) 0xFF;

taskid_t ledTaskId;
taskid_t sweepTaskId;
taskid_t calibrateTaskId;



//-------------------- FORWARD DECLARATIONS --------------------




void onLitButtonPressShortRelease(void)
{
    debug_printf("onLitButtonPressShort()!\n");

    switch (mode) {
    case RUN:
        mode = CALIBRATE_SWEEP;
        debug_printf("RUN--->CALIBRATESWEEP\n");
        //pixelAction[pixelButton] = {NeoRed, NeoBlack, PIXEL_FOREVER, NeoBlack};
        //pixelAction[pixelExternal] = {NeoOrange, NeoOrange, PIXEL_FOREVER, NeoBlack};
        //ledTaskId = taskManager.scheduleFixedRate(300, ledTask);
        calibrateSweepPasses = 20;
        // WAS WORKING sweepTaskId = taskManager.execute(sweepTask);
        //taskManager.execute(ledTask, [deleteWhenDone=false]);
        //taskManager.execute(ledTask);
        break;

    case CALIBRATE_SWEEP:
        debug_printf("EXITING CALIBRATESWEEP... Save Sweep value in EEPROM then RUN \n");
        EE_init_mode = FORCE_SWEEP_MAX_POS;
        initialization();

        //taskManager.cancelTask(ledTaskId);
        //pixelAction[pixelButton] = {NeoRed, NeoBlack, PIXEL_FOREVER, NeoBlack};
        //pixelAction[pixelExternal] = {NeoOrange, NeoOrange, PIXEL_FOREVER, NeoBlack};
        //ledTaskId = taskManager.scheduleFixedRate(300, ledTask);
        mode = RUN;

        break;

    case BEGIN_CALIBRATE_SWEEP:
        debug_printf("EXITING CALIBRATESWEEP... now RUN \n");
        taskManager.cancelTask(ledTaskId);
        break;

    case CALIBRATE_PASSES:
        break;

    //default:
    }

  //lastActionMillis = millis();

}

void onLitButtonPressLongRelease(void)
{
    debug_printf("onLitButtonPressLongRelease()!\n");
    //timeoutDurationMillis = 2 * 60 * 1000;
    //mode = CALIBRATE_SWEEP;
    //nextTimeout = millis() + timeoutDurationMillis;
    //
    //sweepTaskId = taskManager.execute(sweepTask);

    EE_init_mode = FORCE_DEFAULTS;
    initialization();

  //lastActionMillis = millis();

  /*
  flashing LED yellow
    Set Sweep to full CW position (this is "zero"). Pause 5 sec.
    flashing light to green
    begin reading pot and moving arm to the corresponding position
  Timeout set to 2 minutes OR
   */
  if (!litButton.isPressed()) {
  }

}

void setPixelOuts(int pixelID, uint32_t colorVar, int brightVar)
{
  pxl.setPixelColor(pixelID, colorVar);
  pxl.setBrightness(brightVar);
  pxl.show();
}

void sweepTask(void)
{
    int startPos=0, endPos=sweepMaxPos;
    int curPos, potVal;

    int16_t *currentSweepPassLocal = &currentSweepPass;


    if (squirtOK & mode == RUN) {
        debug_printf("SQUIRT\n");
        digitalWrite(SQUIRT_PIN, HIGH);
    } else if (mode == CALIBRATE_SWEEP) {
        debug_printf("CALIBRATE_SWEEP\n");
        potVal = analogRead(POT_IN_PIN);
        sweepMaxPos = map(potVal, 0, 1023, 0, 179);
        endPos=sweepMaxPos;
        currentSweepPassLocal = &calibrateSweepPasses;
    }

    debug_printf("currentSweepPassLocal = %d", *currentSweepPassLocal);

    if (currentSweepPos == ZERO) {
        debug_printf("\nsweep Left: ");
        for (curPos=startPos; curPos < endPos; curPos += servoStep) {
            //Serial.print(curPos);
            //Serial.print(" ");
            spraySweep.write(curPos);
            delay(intraStepDelayMillis);
        }
        currentSweepPos = FULL;
    } else {
        debug_printf("\nsweep Right ");
        for (curPos=endPos; curPos > startPos; curPos -= servoStep) {
            //Serial.print(curPos);
            //Serial.print(" ");
            spraySweep.write(curPos);
            delay(intraStepDelayMillis);
        }
        currentSweepPos = ZERO;
        debug_printf("Decrementing *currentSweepPassLocal (%d) to", *currentSweepPassLocal);
        *currentSweepPassLocal -= 1;
        debug_printf("%d\n", *currentSweepPassLocal);
    }

    if (*currentSweepPassLocal <= 0) {
        debug_printf("currentSweepPassLocal ZERO, returning to RUN\n");
        digitalWrite(SQUIRT_PIN, LOW); // Turn OFF water Spray
        mode = RUN;
        // KILL current sweep task   ;
    } else {
        sweepTaskId = taskManager.execute(sweepTask);
    }
}


void handleTimeout()
{
    nextTimeout = 0;

    // stop flashing

    switch(mode) {
        case RUN:
            // do nothing
            return;

        case CALIBRATE_SWEEP:
            // do nothing
            return;

        case CALIBRATE_PASSES:
            // do nothing
            return;
    }

}

void handleCompleted()
{
    nextTimeout = 0;

    // stop flashing

    switch(mode) {
        case RUN:
            // do nothing
            return;

        case CALIBRATE_SWEEP:
            // do nothing
            return;

        case CALIBRATE_PASSES:
            // do nothing
            return;
    }

}


void ledTask(void)
{
    //debug_printf("ledTask (flashes=%d %d)current colors [0]0x%X  0x%X [1]0x%X  0x%X \n",
    //             pixelAction[0].flashes, pixelAction[1].flashes,
    //             pixelAction[0].color0, pixelAction[0].color1,
    //             pixelAction[1].color0, pixelAction[1].color1);

    for (int curPixel=0; curPixel < PIXELS_IN_USE; curPixel++) {

        if (pixelAction[curPixel].current_color == 0) {
            //debug_printf("pxl %d Sent color0 0x%X\n", curPixel, pixelAction[curPixel].color0);
            pxl.setPixelColor(curPixel, pixelAction[curPixel].color0);
            pxl.show();
            //debug_pln(") to pixel");
            //pixelAction[curPixel].nextChangeMillis = millis() + (pixelAction[curPixel].durationMillis / 2);
            pixelAction[curPixel].current_color = 1;

        } else if (pixelAction[curPixel].current_color == 1) {
            //debug_printf("pxl %d Sent color1 0x%X\n", curPixel, pixelAction[curPixel].color1);
            pxl.setPixelColor(curPixel, pixelAction[curPixel].color1);
            pxl.show();
            pixelAction[curPixel].current_color = 0 ;
            if (pixelAction[curPixel].flashes >= 0 ) {
                pixelAction[curPixel].flashes -= 1;
            }

        }
        //if (pixelAction[curPixel].flashes == 0) {
        //    pixelAction[curPixel].nextChangeMillis = 0;
        //}
    }
}


void task50ms(void)
{
    /* reference only
        typedef struct {
            uint32_t color0;
            uint32_t color1;
            uint32_t durationMillis;
            uint8_t flashes;
            uint8_t current_color;    // always 0 (color0) or 1 (color1)
            uint32_t nextChangeMillis;
        } PixelFlashEvent_t;

    */
    //static PixelFlashEvent_t last_pe={NeoBlack, NeoBlack, 0, 0, NeoBlack, PIXEL_TRIG_NOW};
    //static bool didAction=false;
    uint32_t current_millis = millis();

    //if (memcmp((const void *) &last_pe, (const void *) &pixelAction, sizeof(PixelFlashEvent_t)) != 0) {
    //    dump_pixelAction("task50ms new pe: ", pixelAction);
    //    last_pe = pixelAction;
    //}

    // can also pxl.setBrightness(50);  (before .show)

    //if (pixelAction.nextChangeMillis > 0 && pixelAction.nextChangeMillis <= current_millis  ) {
    //    if (pixelAction.current_color == 0) {
    //        pxl.setPixelColor(pixelAction.pixelSelect, pixelAction.color0);
    //        pxl.show();
    //        //debug_p("Sent color0 (");
    //        //debug_p(pixelAction.color0);
    //        //debug_pln(") to pixel");
    //        pixelAction.nextChangeMillis = millis() + (pixelAction.durationMillis / 2);
    //        pixelAction.current_color = 1;
    //
    //    } else if (pixelAction.current_color == 1) {
    //        pxl.setPixelColor(pixelAction.pixelSelect, pixelAction.color1);
    //        pxl.show();
    //        //debug_p("Sent color1 (");
    //        //debug_p(pixelAction.color1);
    //        //debug_pln(") to pixel");
    //        pixelAction.nextChangeMillis = millis() + (pixelAction.durationMillis / 2);
    //        pixelAction.current_color = 0 ;
    //        // pixelAction.flashes = pixelAction.flashes - 1;
    //        pixelAction.flashes -= 1;
    //
    //    }
    //}
    //if (pixelAction.flashes == 0) {
    //    pixelAction.nextChangeMillis = 0;
    //}
}

void task1s(void)
{
}

void task10s(void)
{
//    int x;
//    updateBatteryDisplay();
//
//    if (!crashCheckDone) {
//        crashCheckDone=true;
//        uint8_t ee_last_boot_behavior = EEPROM.read(EE_LAST_BOOT_BEHAVIOR_ADDR);
//        if (ee_last_boot_behavior == EE_LAST_BOOT_BEHAVIOR_CRASH) {
//            EEPROM.write(EE_LAST_BOOT_BEHAVIOR_ADDR, EE_LAST_BOOT_BEHAVIOR_OK);
//            EEPROM.commit();
//            debug_pln("Updated EEPROM to EE_LAST_BOOT_BEHAVIOR_OK");
//        }
//    }
//
//    //debug_p("task10s.. Begin: avgRightTouchLong.getCount():");
//    //debug_p(avgRightTouchLong.getCount());
//    //debug_p(", avgRightTouch.getCount():");
//    //debug_pln(avgRightTouch.getCount());
//
//    // Do touch averaging
//    if (longTermAvgRightTouch == 0) {
//        x = avgRightTouch.getAvg();
//        avgRightTouchLong.reading(x);
//        longTermAvgRightTouch = avgRightTouchLong.getAvg();
//    //}else if (!isBeingTouched) {
//    //    x = avgRightTouch.getAvg();
//    //    avgRightTouchLong.reading(x);
//    //    longTermAvgRightTouch = avgRightTouchLong.getAvg();
//    //}
//    }else {
//        x = avgRightTouch.getAvg();
//        avgRightTouchLong.reading(x);
//        longTermAvgRightTouch = avgRightTouchLong.getAvg();
//    }
//    //debug_p("task10s.. END: avgRightTouchLong.getCount():");
//    //debug_p(avgRightTouchLong.getCount());
//    //debug_p(", avgRightTouch.getCount():");
//    //debug_pln(avgRightTouch.getCount());
//
//    // Relocate display after 1 minute
//    if ( (! relocateStatusLineDone) && (millis() > AUTO_RELOCATE_STATUS_MS)) {
//        relocate_updateStatusLineDisplay();
//        clearMyIPaddressLine();
//        relocateStatusLineDone = true;
//    }
}



//-------------------- INITIALIZATION SUPPORT  --------------------
void writeLongIntoEEPROM(int address, long number)
{
    EEPROM.write(address, (number >> 24) & 0xFF);
    EEPROM.write(address + 1, (number >> 16) & 0xFF);
    EEPROM.write(address + 2, (number >> 8) & 0xFF);
    EEPROM.write(address + 3, number & 0xFF);
}


long readLongFromEEPROM(int address)
{
    return ((long)EEPROM.read(address) << 24) +
           ((long)EEPROM.read(address + 1) << 16) +
           ((long)EEPROM.read(address + 2) << 8) +
           (long)EEPROM.read(address + 3);
}

void writeInt16IntoEEPROM(int address, uint16_t number)
{
    EEPROM.write(address, (number >> 8) & 0xFF);
    EEPROM.write(address + 1, number & 0xFF);
}


uint16_t readInt16FromEEPROM(int address)
{
    return  ((uint16_t)EEPROM.read(address) << 8) +
            ((uint16_t)EEPROM.read(address + 1));
}

/*
 * initialization() Read/Write EEPROM values as needed:
 *
 * uses global EE_init_mode
 *  NORMAL:               Reads: if uninitialized switch to FORCE_DEFAULTS behavior
 *  FORCE_DEFAULTS:       Writes the DEFAULT hard-coded values into EE
 *  FORCE_SWEEP_MAX_POS:  Uses the CURRENT value of global SweepMaxPos to update EE
 *  FORCE_SWEEP_PASSES:   Uses the CURRENT value of global SweepPasses to update EE
 *  FORCE_ALL:            DOES BOTH steps above
 *
 *  Notice fpr ESP8266  (and ESP32??) EEPROM.begin()/end()  are required
 *  also... WEMOS D1 MINI Pro required setting board type to 'LOLIN(WEMOS) D1 Mini (clone)', NOT
 *  'LOLIN(WEMOS) D1 Mini pro' !!!
 */
void initialization()
{
    uint8_t eeDebug=0;
    bool writtenFlag=false, uninitializedFlag=false;
    int16_t eeSweepMaxPos=0, eeSweepPasses=0;
    bool force_update_SweepMaxPos=false,
         force_update_SweepPasses=false;
    
    
    // EE_init_mode { NORMAL, FORCE_DEFAULTS, FORCE_SWEEP_MAX_POS, FORCE_SWEEP_PASSES, FORCE_ALL };

    EEPROM.begin(5);

    EEPROM.get(EE_ADDR_DEBUG_MODE, eeDebug);
    EEPROM.get(EE_ADDR_SWEEP_MAX_POS, eeSweepMaxPos);
    EEPROM.get(EE_ADDR_SWEEP_PASSES, eeSweepPasses);


    // uninitialized eeproms are -1 (0xFFFF)
    //if (eeSweepMaxPos == 0xFFFF ||  eeSweepPasses == 0xFFFF) {
    //    uninitializedFlag = true;
    //    EE_init_mode = FORCE_DEFAULTS;
    //}

    switch (EE_init_mode) {
        case NORMAL:
            // uninitialized eeproms are -1 (0xFFFF)
            if (eeSweepMaxPos == 0xFFFF ||  eeSweepPasses == 0xFFFF) {
                debug_printf("initialization(): UNINITIALIZED! .. INITIALIZING with defaults\n");
                // fall Thru! to FORCE_DEFAULTS
            } else {
                break;
            }
            
        case  FORCE_DEFAULTS:
            eeSweepMaxPos = DEFAULT_SWEEP_MAX_POS;
            eeSweepPasses = DEFAULT_SWEEP_PASSES;
            force_update_SweepMaxPos = true;
            force_update_SweepPasses = true;
            break;

        case  FORCE_SWEEP_MAX_POS:
            eeSweepMaxPos = sweepMaxPos;
            force_update_SweepMaxPos = true;
            break;

        case  FORCE_SWEEP_PASSES:
            eeSweepPasses = sweepPasses;
            force_update_SweepPasses = true;
            break;

        case  FORCE_ALL:
            eeSweepMaxPos = sweepMaxPos;
            eeSweepPasses = sweepPasses;
            force_update_SweepMaxPos = true;
            force_update_SweepPasses = true;
            break;
    }

    debug_printf("initialization() (%s) initial reads: DEBUG_MODE:0x%X, SWEEP_MAX_POS:0x%X=%d, SWEEP_PASSES:0x%X=%d\n",
                 EE_init_mode_str[EE_init_mode],
                 eeDebug, eeSweepMaxPos, eeSweepMaxPos, eeSweepPasses, eeSweepPasses);

    if (force_update_SweepMaxPos) {
        EEPROM.put(EE_ADDR_SWEEP_MAX_POS, eeSweepMaxPos);
        writtenFlag = true;
        debug_printf("... wrote %d @EE_ADDR_SWEEP_MAX_POS(0x%x)\n", eeSweepMaxPos, EE_ADDR_SWEEP_MAX_POS);
    }

    if (force_update_SweepPasses) {
        EEPROM.put(EE_ADDR_SWEEP_PASSES, eeSweepPasses);
        writtenFlag = true;
        debug_printf("... wrote %d @EE_ADDR_SWEEP_PASSES(0x%x)\n", eeSweepPasses, EE_ADDR_SWEEP_PASSES);
    }

    sweepMaxPos = eeSweepMaxPos;
    sweepPasses = eeSweepPasses;

    if (writtenFlag) {
        boolean ok1 = EEPROM.commit();
        debug_printf("EEPROM Commit %s\n", ok1 ? "OK" : "FAILED");

        debug_printf("initialization() Completion w/ write SWEEP_MAX_POS:%d, SWEEP_PASSES:%d\n",
             sweepMaxPos, sweepPasses);

    }
    EEPROM.end();
    EE_init_mode = NORMAL;
}

//-------------------- STARTUP / SETUP  --------------------
void setup() {
  //int potVal;

  // Remember to set your serial monitor to 74880 baud
  // This odd speed will show ESP8266 boot diagnostics too
  Serial.begin(74880);
  //Serial.begin(115200);
  delay(400);  // // 400 required for ESP8266 "D1 Mini Pro"
  debug_printf("\n\n");
  delay(100);
  debug_printf("Cat Deterrent\n");

  initialization();
  mode=RUN;

  pinMode(NEOPIXEL_PIN, OUTPUT);
  setPixelOuts(pixelExternal, NeoGreen, bright_state);

  setPixelOuts(pixelButton, NeoWhite, bright_state);
  pinMode(PIR_IN_PIN, INPUT);
  pinMode(SQUIRT_PIN, OUTPUT);
  digitalWrite(SQUIRT_PIN, LOW);

  //potVal = analogRead(POT_IN_PIN);
  //sweepMaxPos = map(potVal, 0, 1023, 0, 179);
  //debug_printf("Maximum deflection readings: raw: %d, Mapped degrees: %d\n", potVal, sweepMaxPos);
  debug_printf("Maximum deflection degrees: %d\n", sweepMaxPos);
  debug_printf("Current default passes: %d\n", sweepPasses);

  spraySweep.attach(SERVO_PIN,500,2400);

  litButton.begin();
  litButton.onPressed(onLitButtonPressShortRelease);
  litButton.onPressedFor(3000, onLitButtonPressLongRelease);

  //pixelAction = {color0, color1, duration, pixelExternal, flashes, NeoBlack, PIXEL_TRIG_NOW};

}


//-------------------- ACTION LOOP --------------------
void loop() {
  uint32_t sysTick = millis();
  int noMotion = digitalRead(PIR_IN_PIN);  //  Low True

  //  // ----- Button / user interaction Handling ------
    litButton.read();
  //
    taskManager.runLoop();
  //
  //  // Serial.println("Loop!");
  //
  //  if(sysTick - tick50 > 50){
  //    tick50 = sysTick;
  //    task50ms();
  //  }
  //
  //  if(sysTick - tick1000 > 1000){
  //      tick1000 = sysTick;
  //      task1s();
  //  }
  //
  //  if(sysTick - tick10000 > 10000){
  //      tick10000 = sysTick;
  //      task10s();
  //  }
  //

  if (noMotion && !motionDetectorCleared){
      motionDetectorCleared = true;
  }

  switch (mode) {
      case RUN:
        if (mode != previousMode) {
            debug_printf("Mode change %d --> RUN\n");
            pixelAction[pixelButton] =   {NeoDarkPurple, NeoDarkPurple, PIXEL_FOREVER, NeoBlack};
            pixelAction[pixelExternal] = {NeoGreen, NeoBlack, PIXEL_FOREVER, NeoBlack};
            ledTaskId = taskManager.scheduleFixedRate(1000, ledTask);
            previousMode = mode;
        }

        // ----- Beast detection and response Handling ------
        if (motionDetectorCleared && ! noMotion && currentSweepPass <= 0) {
            debug_printf("MOTION! & currentSweepPass:%d\n", currentSweepPass);
          sweepTaskId = taskManager.execute(sweepTask);
        }
        break;

      case CALIBRATE_SWEEP:
        if (mode != previousMode) {
            debug_printf("Mode change %d --> CALIBRATE_SWEEP\n");
            taskManager.cancelTask(ledTaskId);
            pixelAction[pixelButton] = {NeoRed, NeoBlack, PIXEL_FOREVER, NeoBlack};
            pixelAction[pixelExternal] = {NeoOrange, NeoOrange, PIXEL_FOREVER, NeoBlack};
            ledTaskId = taskManager.scheduleFixedRate(300, ledTask);
            previousMode = mode;

            sweepTaskId = taskManager.execute(sweepTask);
        }
        break;

  }

 // int pos, val;
 //
 // // reads the value of the potentiometer (0 to 1023)
 //
 // val = analogRead(POT_IN_PIN);
 //
 //// scale it to use it with the servo (0 to 180)
 //
 // val = map(val, 0, 1023, 0, 179);
 //
 // // sets servo position according to the scaled value
 //
 // spraySweep.write(val);
 //
  // waits for the servo to get there

}
