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


#define SERVO_PIN      D1
#define SQUIRT_PIN     D2
#define BUTTON_IN_PIN  D5
#define NEOPIXEL_PIN   D7
#define PIR_IN_PIN     D6
#define POT_IN_PIN     A0

Adafruit_NeoPixel pixel_33(2 /*NUMPIXELS*/, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define pixelButton     0
#define pixelExternal   1

Servo spraySweep;


#define RGB_BRIGHTNESS 64 // Change white brightness (max 255)

// Defaults from running it!
//  RGB_BUILTIN is 30,   RGB_BRIGHTNESS is 64


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

int sweepMaxPos=180;
 
#define STEP_DELAY 2000

//uint8_t  brig
//
// ht_state = RGB_BRIGHTNESS;
uint8_t  bright_state = 64;

int intraStepDelayMillis = 1;

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
}

void setPixelOuts(int pixelID, uint32_t colorVar, int brightVar)
{
  pixel_33.setPixelColor(pixelID, colorVar);
  pixel_33.setBrightness(brightVar);
  pixel_33.show();
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


void loop() {

  int noMotion = digitalRead(PIR_IN_PIN);  //  Low True

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

  //delay(15);
  //Serial.println(bright_state);
  //for (pos = 0; pos <= 180; pos += 1) {  // goes from 0 degrees to 180 degrees
  //  // in steps of 1 degree
  //  spraySweep.write(pos);  // tell servo to go to position in variable 'pos'
  //  delay(15);           // waits 15ms for the servo to reach the position
  //}
  //for (pos = 180; pos >= 0; pos -= 1) {  // goes from 180 degrees to 0 degrees
  //  spraySweep.write(pos);                  // tell servo to go to position in variable 'pos'
  //  delay(15);                           // waits 15ms for the servo to reach the position
  //}


  Serial.println("Loop!");
//  digitalWrite(RGB_BUILTIN, HIGH);   // Turn the RGB LED white
//  delay(STEP_DELAY);
//  neopixelWrite(RGB_BUILTIN,bright_state,bright_state,bright_state);



  //setPixelOuts(NeoWhite, bright_state);
  //
  //delay(STEP_DELAY);
  //
  //Serial.println("OFF");
  ////digitalWrite(RGB_BUILTIN, LOW);    // Turn the RGB LED off
  //delay(STEP_DELAY);
  //
  //Serial.print("RED ");
  ////neopixelWrite(RGB_BUILTIN,bright_state,0,0); // Red
  //setPixelOuts(NeoRed, bright_state);
  //delay(STEP_DELAY);
  //
  //Serial.println("GREEN");
  ////neopixelWrite(RGB_BUILTIN,0,bright_state,0); // Green
  //setPixelOuts(NeoGreen, bright_state);
  //delay(STEP_DELAY);
  //
  //Serial.println("BLUE");
  ////neopixelWrite(RGB_BUILTIN,0,0,bright_state); // Blue
  //setPixelOuts(NeoBlue, bright_state);
  //delay(STEP_DELAY);
  //
  //Serial.println("OFF");
  ////neopixelWrite(RGB_BUILTIN,0,0,0); // Off / black
  //setPixelOuts(NeoBlack, bright_state);
  //
  //// Toggle Brightness
  //bright_state = (bright_state == RGB_BRIGHTNESS) ? RGB_FULL_BRIGHTNESS: RGB_BRIGHTNESS;
  //Serial.print("\ncurrent bright_state: ");
  //Serial.println(bright_state);
  //
  //delay(STEP_DELAY);
  //
}
