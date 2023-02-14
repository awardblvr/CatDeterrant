/*
  BlinkRGB

  Demonstrates usage of onboard RGB LED on some ESP dev boards.

  Calling digitalWrite(RGB_BUILTIN, HIGH) will use hidden RGB driver.
    
  RGBLedWrite demonstrates controll of each channel:
  void neopixelWrite(uint8_t pin, uint8_t red_val, uint8_t green_val, uint8_t blue_val)

  WARNING: After using digitalWrite to drive RGB LED it will be impossible to drive the same pin
    with normal HIGH/LOW level
*/
#include "debug_print.h"
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <EasyButton.h>
#include <vector>
#include <forward_list>
#include <movingAvg.h>                  // https://github.com/JChristensen/movingAvg
#include <EEPROM.h>

//-------------------- HW PIN ASSIGNMENTS --------------------

#define SERVO_PIN      D1
#define SQUIRT_PIN     D2
#define BUTTON_IN_PIN  D5
#define NEOPIXEL_PIN   D7
#define PIR_IN_PIN     D6
#define POT_IN_PIN     A0

//-------------------- BUTTON SUPPORT --------------------
EasyButton litButton(BUTTON_IN_PIN);

//-------------------- SERVO SUPPORT --------------------
Servo spraySweep;
#define STEP_DELAY 2000

int sweepMaxPos=180;
int intraStepDelayMillis = 1;

//-------------------- NEOPIXEL SUPPORT --------------------
#define PIXELS_IN_USE 2

Adafruit_NeoPixel pxl(PIXELS_IN_USE, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define pixelButton     0
#define pixelExternal   1


#define RGB_BRIGHTNESS 64 // Change white brightness (max 255)

typedef struct {
    uint32_t color0;
    uint32_t color1;
    uint32_t durationMillis;
    uint8_t pixelSelect;
    uint8_t flashes;
    uint8_t current_color;    // always 0 (color0) or 1 (color1)
    uint32_t nextChangeMillis;
} PixelFlashEvent_t;
PixelFlashEvent_t pixelAction;
#define PIXEL_TRIG_NOW 1

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


static uint32_t tick50=0;  // future:, tick128=0, tick10000=0, tick30000=0;


void onLitButtonPressShortRelease(void)
{
  //lastActionMillis = millis();
}

void onLitButtonPressLongRelease(void)
{
  //lastActionMillis = millis();

  if (!litButton.isPressed()) {
  }

}

void setPixelOuts(int pixelID, uint32_t colorVar, int brightVar)
{
  pxl.setPixelColor(pixelID, colorVar);
  pxl.setBrightness(brightVar);
  pxl.show();
}

void flash_pixel(uint8_t pixelSelect, uint32_t color0, uint8_t flashes, uint32_t duration, uint32_t color1)
{
    /* reference only
        typedef struct {
            uint32_t color0;
            uint32_t color1;
            uint32_t durationMillis;
            uint8_t pixelSelect;
            uint8_t flashes;
            uint8_t current_color;    // always 0 (color0) or 1 (color1)
            uint32_t nextChangeMillis;
        } PixelFlashEvent_t;
        std::forward_list<PixelFlashEvent_t> PixelFlashEvents {};
    */

    //PixelFlashEvents.push_front({color0, color1, pixelSelect, duration, flashes, NeoBlack, 1});
    pixelAction = {color0, color1, duration, pixelSelect, flashes, NeoBlack, PIXEL_TRIG_NOW};
    //dump_pixelAction("@instant: ", pixelAction);
}

void sweep(int passes)
{
    int startPos=0, endPos=sweepMaxPos;
    int curPos;
    int motionState;

    for (;passes; passes--) {
        Serial.print("Pass ");
        Serial.println(passes);
        spraySweep.write(startPos);

        if (startPos < endPos) {
            Serial.print("\nsweep Left: ");
            setPixelOuts(pixelExternal, NeoMagenta, RGB_FULL_BRIGHTNESS);
            for (curPos=startPos; curPos < endPos; curPos++) {
                //Serial.print(curPos);
                //Serial.print(" ");
                spraySweep.write(curPos);
                delay(intraStepDelayMillis);

            }
            Serial.print("\nsweep Right: ");
            setPixelOuts(pixelExternal, NeoOrange, RGB_FULL_BRIGHTNESS);
            for (curPos=endPos; curPos > startPos; curPos--) {
                //Serial.print(curPos);
                //Serial.print(" ");
                spraySweep.write(curPos);
                delay(intraStepDelayMillis);
            }
        }
        delay(500);
        setPixelOuts(pixelExternal, NeoNavyBlue, RGB_BRIGHTNESS);
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

    if (pixelAction.nextChangeMillis > 0 && pixelAction.nextChangeMillis <= current_millis  ) {
        if (pixelAction.current_color == 0) {
            pxl.setPixelColor(pixelAction.pixelSelect, pixelAction.color0);
            pxl.show();
            //debug_p("Sent color0 (");
            //debug_p(pixelAction.color0);
            //debug_pln(") to pixel");
            pixelAction.nextChangeMillis = millis() + (pixelAction.durationMillis / 2);
            pixelAction.current_color = 1;

        } else if (pixelAction.current_color == 1) {
            pxl.setPixelColor(pixelAction.pixelSelect, pixelAction.color1);
            pxl.show();
            //debug_p("Sent color1 (");
            //debug_p(pixelAction.color1);
            //debug_pln(") to pixel");
            pixelAction.nextChangeMillis = millis() + (pixelAction.durationMillis / 2);
            pixelAction.current_color = 0 ;
            // pixelAction.flashes = pixelAction.flashes - 1;
            pixelAction.flashes -= 1;

        }
    }
    if (pixelAction.flashes == 0) {
        pixelAction.nextChangeMillis = 0;
    }
}


//-------------------- STARTUP / SETUP  --------------------
void setup() {
  int potVal;

  pinMode(NEOPIXEL_PIN, OUTPUT);
  setPixelOuts(pixelExternal, NeoGreen, bright_state);
  setPixelOuts(pixelButton, NeoWhite, bright_state);
  pinMode(PIR_IN_PIN, INPUT);

  Serial.begin(115200);
  delay(1500);  // // 400 required for ESP8266 "D1 Mini Pro"

  Serial.println("Cat Deterrent\n");
  Serial.print("Maximum deflection readings: raw:");
  potVal = analogRead(POT_IN_PIN);
  Serial.print(potVal);
  Serial.print(", Mapped degrees:");
  sweepMaxPos = map(potVal, 0, 1023, 0, 179);
  Serial.println(sweepMaxPos);

  spraySweep.attach(SERVO_PIN,500,2400);

  litButton.begin();
  litButton.onPressed(onLitButtonPressShortRelease);
  litButton.onPressedFor(2000, onLitButtonPressLongRelease);

  //pixelAction = {color0, color1, duration, pixelExternal, flashes, NeoBlack, PIXEL_TRIG_NOW};

}


//-------------------- ACTION LOOP --------------------
void loop() {
  uint32_t sysTick = millis();
  int noMotion = digitalRead(PIR_IN_PIN);  //  Low True

  // Serial.println("Loop!");

  if(sysTick - tick50 > 50){
    tick50 = sysTick;
    task50ms();
  }

  // ----- Button / user interaction Handling ------
  litButton.read();


  // ----- Beast detection and response Handling ------
  if (! noMotion) {
    sweep(2);
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
