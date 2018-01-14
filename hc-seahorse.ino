#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROMex.h>

#include <Adafruit_NeoPixel.h>

#include <HC_BouncyButton.h>

/**********************************
  PIN DEFINITIONS
 
 BAT  incoming battery (lipo 7.4v)
 GND  ground
 BUS  disconnected
 5V   Vcc for low bat LED
 8  Neopixel Control
 6  Motor Direction
 5  Motor Direction
 4  Motor Direction
 3  Low Battery Output (conn to MOSFET switch on vregulator)
 1  disconnected
 0  disconnected
 RST  disconnected
 
 9   PWM Speed Control (motor enable)
 10  PWM Speed Control (motor enable)
 11  Calibrate Button (Green/Black leads)
 12  Action Button    (Red/Yellow leads)
 13  Motor Direction
 Aref  disconnected
 A0  Light Sensor 0
 A1  Light Sensor 1
 A2  Light Sensor 2
 A3  Light Sensor 3
 A4  SDA
 A5  SCL
 A6  Light Sensor 4
 A7  Battery Level Monitor
 **********************************/

// normal Arduino (and trinket PRO) uses pin 13, trinket uses pin 1
//#define LED_PIN 13

#define MOTOR_PIN_ENABLE_R 10
#define MOTOR_PIN_ENABLE_L 9

#define MOTOR_PIN_DIR_R1 5
#define MOTOR_PIN_DIR_R2 6

#define MOTOR_PIN_DIR_L1 4
#define MOTOR_PIN_DIR_L2 13

#define PIX_PIN 8
#define PIX_COUNT 5


#define ACTION_BUTTON_PIN 12
#define CALIBRATE_BUTTON_PIN 11

#define BATTERY_INDICATOR_PIN 3

#define NUM_LIGHT_SENSORS 5
// A4 & A5 are not available as ADC, they are for I2C
// NICE MOVE, NUMB NUTS!  Once PCB and connectors were all assembled, got the ADC pins reversed.  All good, fix in software!
//byte lightPins[] = {A0, A1, A2, A3, A6 }; 
byte lightPins[] = {A6, A3, A2, A1, A0 }; 

#define BATTERY_MONITOR_ADC_PIN A7


/**********************************
  EEPROM Address Space
  0     single byte, 1=FOLLOW_DARK, 2=FOLLOW_LIGHT
  1-10  five integer sensor line values
  11-20 five integer sensor background values
  21-31 five integer sensor threshold values
 **********************************/
#define EEPROM_FOLLOW_MODE 0
#define EEPROM_VAL_LINE 1
#define EEPROM_VAL_BACK 11
#define EEPROM_VAL_THRESH 21


#define SYS_IDLE 0
#define SYS_CALIBRATE_LINE 1
#define SYS_CALIBRATE_BACKGROUND 2
#define SYS_RUN 3

byte SYS_MODE=SYS_IDLE;

#define FOLLOW_DARK 1
#define FOLLOW_LIGHT 2

byte FOLLOW_MODE=FOLLOW_DARK;


// first argument is the I2C address of your LCD, check the datasheet for your LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);


BouncyButton actionButton=BouncyButton(ACTION_BUTTON_PIN);
BouncyButton calibrateButton=BouncyButton(CALIBRATE_BUTTON_PIN);


// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel neopixels = Adafruit_NeoPixel(PIX_COUNT, PIX_PIN, NEO_GRB + NEO_KHZ800);

// (0=darkest, 255=brightest)
#define PIX_BRIGHTNESS 5

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.


// constants for my colors
uint32_t pixelColorA, pixelColorB, pixelColor1, pixelColor2;


int lightVal[NUM_LIGHT_SENSORS];    // variable to read the value from the analog pin
int lightValMax[NUM_LIGHT_SENSORS];
int lightValMin[NUM_LIGHT_SENSORS];
int lightValLine[NUM_LIGHT_SENSORS];
int lightValBack[NUM_LIGHT_SENSORS];
int lightValThresh[NUM_LIGHT_SENSORS];
int lineVisible[NUM_LIGHT_SENSORS];

#define LINE_VISIBLE 1
#define LINE_NOT_VISIBLE 0

#define LINE_MIDDLE 0
#define LINE_LEFT 1
#define LINE_RIGHT 2

byte lastSideSawLine=LINE_MIDDLE;

byte neopixelState=0, neopixelDirection=0;

#define READING_DELAY_MILLIS 1
#define DISPLAY_DELAY_MILLIS 1000
#define NEOPIXEL_DELAY_MILLIS 150
#define BATTERY_DELAY_MILLIS 2000

// 120,85 works as initial test
// 130,85 seems good but starting to require straight reverse a little
// 120,90 loses the line
// 120,89 actually keeps the line
// 121,89 good
// 122,89 good
// 123,89 zippy
// 124,89 good but lost the line once (only once, didnt repeat)
// 125,89 starting to appear jittery, like its spending too much time out at the extremes (not measured)
// 126,89 wobbly but looks fast, had to do the straight backup once
// 127,89 same as 126
// 128,89 was crusin
// 129,89 fast but some issues (backup, wobbly)
// 130,89 fast but lost line once
// 140,89 fast, only lost once
// 150,89 losing line more often and backups
// 160,89 more issues losing the line
// 170,89 pretty fast, some quality issues, battery low issues too (not sure if bat was low or if drops at high speed???)


// Timetrials.  3 laps, counterclockwise.  If you turn around, you get one more try, two fails and you're deemed unreliable config
// 170,89  2xFAIL
// 160,89  2xFAIL
// 150,89  2xFAIL
// 140,89  11.43
// 130,89  12.40
// 120,89  12.99
// back to 140,89 to see if batt is lower, failed the line :-(  still calling it the best!

// on a full battery, suffering at 150,89 or faster :-(  Seems trials from earlier had less and less battery power so it could perform at higher settings, but not after a recharge.

#define MOTOR_SPEED_FULL 140
#define MOTOR_SPEED_SLOW 89
    

#define BATTERY_OK 0
#define BATTERY_WARN 1
#define BATTERY_ERROR 2
byte BATTERY_STATE=BATTERY_OK;

// 700=6.8v, 730=7.1v
int BATTERY_ADC=0;
#define BATTERY_LOW_ERROR 700
#define BATTERY_LOW_WARN 730

unsigned long lastReadingMillis, lastDisplayMillis, lastNeopixelMillis, lastBatteryMillis;

// the setup function runs once when you press reset or power the board
void setup() {
  //Serial.begin(9600);
  
  // initialize digital pin 13 as an output.
  //pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, LOW);
  
  pinMode(ACTION_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP);
  
  // activate the regulated voltage circuit
  // make sure you do this before initializing neopixels and LCD so they have power
  pinMode(BATTERY_INDICATOR_PIN, OUTPUT);
  digitalWrite(BATTERY_INDICATOR_PIN, LOW);

  pinMode(MOTOR_PIN_ENABLE_L, OUTPUT);
  pinMode(MOTOR_PIN_ENABLE_R, OUTPUT);
  pinMode(MOTOR_PIN_DIR_L1, OUTPUT);
  pinMode(MOTOR_PIN_DIR_L2, OUTPUT);
  pinMode(MOTOR_PIN_DIR_R1, OUTPUT);
  pinMode(MOTOR_PIN_DIR_R2, OUTPUT);
  
  stopMotors();
  
  actionButton.init();
  calibrateButton.init();
  
  for (byte sensorIdx=0; sensorIdx<NUM_LIGHT_SENSORS; sensorIdx++) {
    lightValMax[sensorIdx]=0;
    lightValMin[sensorIdx]=1024;
  }
  
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.home();
  
  neopixels.begin();
  neopixels.setBrightness(PIX_BRIGHTNESS);
  neopixels.show(); // Initialize all pixels to 'off'
  
  
  pixelColorA=neopixels.Color(  0,   0, 255);
  pixelColorB=neopixels.Color(  255, 0,   0);
//  pixelColorB=neopixels.Color(204, 255,   0);  // orange


  // retrieve saved follow mode and sensor thresholds from EEPROM
  FOLLOW_MODE = EEPROM.read(EEPROM_FOLLOW_MODE);
  for (byte sensorIdx=0; sensorIdx<NUM_LIGHT_SENSORS; sensorIdx++) {
    lightValLine[sensorIdx]   = EEPROM.readInt(EEPROM_VAL_LINE+sensorIdx*2);
    lightValBack[sensorIdx]   = EEPROM.readInt(EEPROM_VAL_BACK+sensorIdx*2);
    lightValThresh[sensorIdx] = EEPROM.readInt(EEPROM_VAL_THRESH+sensorIdx*2);
  }

}

// the loop function runs over and over again forever
void loop() {

  /******************************************
   * Check the battery level
   ******************************************/
  if ((millis()-lastBatteryMillis)>BATTERY_DELAY_MILLIS) {
    lastBatteryMillis=millis();
    
    // read the battery level
    delay(1);
    BATTERY_ADC = analogRead(BATTERY_MONITOR_ADC_PIN);

    if (BATTERY_ADC<BATTERY_LOW_ERROR) {
      // if below ERROR threshold, shut the factory down
      BATTERY_STATE = BATTERY_ERROR;
      
      stopMotors();

      digitalWrite(BATTERY_INDICATOR_PIN, HIGH);
      
      lcd.clear();
      for (byte lineIdx=0; lineIdx<4; lineIdx++) {
        lcd.setCursor(0,lineIdx);
        lcd.print(F("LOW BATTERY"));
      }
      
      while (true) {
        // you'll need to reset the chip to get out of this loop
      }
    } else if (BATTERY_ADC<BATTERY_LOW_WARN) {
      // if below WARN threshold, illuminate the LED
      BATTERY_STATE = BATTERY_WARN;
    } else {
      // battery is fine
      BATTERY_STATE = BATTERY_OK;
    }
    
  }

  /******************************************
   * Check for button presses
   ******************************************/
  if (actionButton.update()) {
    // button state changed, take action
    if (actionButton.getState()) {
      // released (state is HIGH)
      if (SYS_MODE == SYS_IDLE) {
        SYS_MODE = SYS_RUN;
      } else if (SYS_MODE == SYS_RUN) {
        SYS_MODE = SYS_IDLE;
      }
    } else {
      // pressed (state is LOW)
    }
    updateLCD();
  }

  if (calibrateButton.update()) {
    // button state changed, take action
    if (calibrateButton.getState()) {
      // released (state is HIGH)
      if (SYS_MODE == SYS_IDLE) {
        SYS_MODE = SYS_CALIBRATE_LINE;
      } else if (SYS_MODE == SYS_CALIBRATE_LINE) {
        
        // record the line values
        for (byte sensorIdx=0; sensorIdx<NUM_LIGHT_SENSORS; sensorIdx++) {
          // do it twice, something funky with the values drifting
          lightValLine[sensorIdx] = analogRead(lightPins[sensorIdx]);
          //delay(5);
          lightValLine[sensorIdx] = analogRead(lightPins[sensorIdx]);
          EEPROM.writeInt(EEPROM_VAL_LINE+sensorIdx*2, lightValLine[sensorIdx]);
        }
        
        SYS_MODE = SYS_CALIBRATE_BACKGROUND;
      } else if (SYS_MODE == SYS_CALIBRATE_BACKGROUND) {

        // record the background values
        for (byte sensorIdx=0; sensorIdx<NUM_LIGHT_SENSORS; sensorIdx++) {
          // do it twice, something funky with the values drifting
          lightValBack[sensorIdx] = analogRead(lightPins[sensorIdx]);
          //delay(5);
          lightValBack[sensorIdx] = analogRead(lightPins[sensorIdx]);
          EEPROM.writeInt(EEPROM_VAL_BACK+sensorIdx*2, lightValBack[sensorIdx]);
        }
        
        // is the background brighter or darker than the line? set following mode and sensor thresholds
        // index=2 uses the middle sensor, no particular reason, had to pick one. Wanted to use idx=0 but that one is behaving funny in early testing.
        int refIdx=2;
        if (refIdx>=NUM_LIGHT_SENSORS) {
          refIdx = NUM_LIGHT_SENSORS-1;
        }
        
        if (lightValLine[refIdx]>lightValBack[refIdx]) {
          FOLLOW_MODE = FOLLOW_LIGHT;
        } else {
          FOLLOW_MODE = FOLLOW_DARK;
        }
        EEPROM.write(EEPROM_FOLLOW_MODE, FOLLOW_MODE);

        // calculate a unique tripping threshold for each sensor
        for (byte sensorIdx=0; sensorIdx<NUM_LIGHT_SENSORS; sensorIdx++) {
          
          // simpleton algorithm, midpoint between line and dark, better algorithm puts thresh closer to line, since line is constant-ish and background may vary wildly on carpets
          //lightValThresh[sensorIdx] = (int)((lightValLine[sensorIdx]+lightValBack[sensorIdx])/2);
          
          // maybe MAYBE my black tape is the darkest i can find, so maybe narrow down the window very close to the qline?
          if (FOLLOW_MODE == FOLLOW_DARK) {
            lightValThresh[sensorIdx] = lightValLine[sensorIdx]+35;
          } else {
            lightValThresh[sensorIdx] = lightValLine[sensorIdx]-35;
          }
          
          EEPROM.writeInt(EEPROM_VAL_THRESH+sensorIdx*2, lightValThresh[sensorIdx]);
        }
        
        SYS_MODE = SYS_IDLE;
      }
    } else {
      // pressed (state is LOW)
    }
    updateLCD();
  }

  
  /******************************************
   * Scan the floor frequently
   ******************************************/
  if ((millis()-lastReadingMillis)>READING_DELAY_MILLIS) {
    lastReadingMillis=millis();

    for (byte sensorIdx=0; sensorIdx<NUM_LIGHT_SENSORS; sensorIdx++) {
      // do it twice, something funky with the values drifting
      lightVal[sensorIdx] = analogRead(lightPins[sensorIdx]);
      //delay(5);
      lightVal[sensorIdx] = analogRead(lightPins[sensorIdx]);
      
      if (lightVal[sensorIdx] < lightValMin[sensorIdx]) { lightValMin[sensorIdx]=lightVal[sensorIdx]; }
      if (lightVal[sensorIdx] > lightValMax[sensorIdx]) { lightValMax[sensorIdx]=lightVal[sensorIdx]; }
      
      if ((FOLLOW_MODE==FOLLOW_DARK  && lightVal[sensorIdx]<=lightValThresh[sensorIdx]) ||
          (FOLLOW_MODE==FOLLOW_LIGHT && lightVal[sensorIdx]>=lightValThresh[sensorIdx])) {
        // this sensor is on the line
        lineVisible[sensorIdx] = LINE_VISIBLE;
      } else {
        // this sensor is not on the line
        lineVisible[sensorIdx] = LINE_NOT_VISIBLE;
      }
    }
  }

  /******************************************
   * Motor control
   ******************************************/
  if (SYS_MODE == SYS_RUN) {
   // x x 1 x x
   

    
    if (lineVisible[1]==LINE_VISIBLE) {
      // x 1 x x x
      //setMotors(MOTOR_SPEED_SLOW,HIGH,LOW,MOTOR_SPEED_FULL,HIGH,LOW);
      setMotors(0,LOW,LOW,MOTOR_SPEED_FULL,HIGH,LOW);     
      lastSideSawLine = LINE_LEFT;
      
    } else if (lineVisible[3]==LINE_VISIBLE) {
      // x x x 1 x
      //setMotors(MOTOR_SPEED_FULL,HIGH,LOW,MOTOR_SPEED_SLOW,HIGH,LOW);
      setMotors(MOTOR_SPEED_FULL, HIGH, LOW, 0, LOW, LOW);
      lastSideSawLine = LINE_RIGHT;
      
    } else if (lineVisible[0]==LINE_VISIBLE) {
      // 1 0 x 0 x
      setMotors(MOTOR_SPEED_SLOW,LOW,HIGH,MOTOR_SPEED_FULL,HIGH,LOW);
      lastSideSawLine = LINE_LEFT;
      
    } else if (lineVisible[4]==LINE_VISIBLE) {
      // 0 0 x 0 1
      setMotors(MOTOR_SPEED_FULL, HIGH, LOW, MOTOR_SPEED_SLOW, LOW, HIGH);
      lastSideSawLine = LINE_RIGHT;
      
    } else if (lineVisible[2]==LINE_VISIBLE) {
      // 0 0 1 0 0
      setMotors(MOTOR_SPEED_FULL, HIGH, LOW, MOTOR_SPEED_FULL, HIGH, LOW);
      lastSideSawLine = LINE_MIDDLE;

    } else {
      // rotate back in the direction that last saw the line
      if (lastSideSawLine == LINE_RIGHT) {
        setMotors(MOTOR_SPEED_SLOW, HIGH, LOW, MOTOR_SPEED_SLOW, LOW, HIGH);
        
      } else if (lastSideSawLine == LINE_LEFT) {
        setMotors(MOTOR_SPEED_SLOW, LOW, HIGH, MOTOR_SPEED_SLOW, HIGH, LOW);
        
      } else {
        // middle was last seen but we dont see line now. Do a little dance
        setMotors(MOTOR_SPEED_SLOW, LOW, HIGH, MOTOR_SPEED_SLOW, LOW, HIGH);
      }
//      stopMotors();
    }
   
    
  } else {
    // any other mode, shut down the motors
    stopMotors();
  }


  /******************************************
   * Update the LCD display only periodically to make it a bit more readable
   ******************************************/
  if ((millis()-lastDisplayMillis)>DISPLAY_DELAY_MILLIS) {
    lastDisplayMillis=millis();
    
    updateLCD();  
  }

  /******************************************
   * Update the Neopixels frequently, any animation delays will be handled inside the function
   ******************************************/
  updateNeopixels();

}

void updateNeopixels() {

  if (SYS_MODE==SYS_CALIBRATE_LINE || SYS_MODE==SYS_CALIBRATE_BACKGROUND) {
  
    if (SYS_MODE==SYS_CALIBRATE_LINE) {
      pixelColor1=pixelColorA;
      pixelColor2=pixelColorB;
    } else {
      pixelColor1=pixelColorB;
      pixelColor2=pixelColorA;
    }
    
    /******************************************
     * Cute animated effect for the Neopixels during calibration
     ******************************************/
    if ((millis()-lastNeopixelMillis)>NEOPIXEL_DELAY_MILLIS) {
      lastNeopixelMillis=millis();

      for (byte pixIdx=0; pixIdx<PIX_COUNT; pixIdx++) {
        neopixels.setPixelColor(pixIdx, pixelColor1);
      }

      if (neopixelState<=0) {
        neopixels.setPixelColor(0, pixelColor2);
        neopixelDirection=1;      
      } else if (neopixelState==1) {
        neopixels.setPixelColor(1, pixelColor2); 
      } else if (neopixelState==2) {
        neopixels.setPixelColor(2, pixelColor2);
      } else if (neopixelState==3) {
        neopixels.setPixelColor(3, pixelColor2);
      } else if (neopixelState>=4) {
        neopixels.setPixelColor(4, pixelColor2);
        neopixelDirection=-1;
      }

      neopixelState += neopixelDirection;
    }

  } else if (SYS_MODE==SYS_IDLE || SYS_MODE==SYS_RUN) {
    for (byte sensorIdx=0; sensorIdx<NUM_LIGHT_SENSORS; sensorIdx++) {
      if (lineVisible[sensorIdx]==LINE_VISIBLE) {
        neopixels.setPixelColor(sensorIdx, pixelColorA);
      } else {
        neopixels.setPixelColor(sensorIdx, pixelColorB);
      }      
    }
  }
  
  neopixels.show();
}

void setMotors(int leftEnable, boolean leftA, boolean leftB, int rightEnable, boolean rightA, boolean rightB) {
  analogWrite(MOTOR_PIN_ENABLE_L, leftEnable);
  analogWrite(MOTOR_PIN_ENABLE_R, rightEnable);

  digitalWrite(MOTOR_PIN_DIR_L1, leftA);
  digitalWrite(MOTOR_PIN_DIR_L2, leftB);
  digitalWrite(MOTOR_PIN_DIR_R1, rightA);
  digitalWrite(MOTOR_PIN_DIR_R2, rightB);
}

void stopMotors() {
  setMotors(0,LOW,LOW,0,LOW,LOW);
}

void updateLCD() {
   
  for (byte sensorIdx=0; sensorIdx<NUM_LIGHT_SENSORS; sensorIdx++) {
    lcd.setCursor(sensorIdx*4,0);
    lcd.print(map(lightValMin[sensorIdx],0,1023,0,999),DEC);
    lcd.setCursor(sensorIdx*4,1);
    lcd.print(map(lightVal[sensorIdx],0,1023,0,999),DEC);
    lcd.setCursor(sensorIdx*4,2);
    lcd.print(map(lightValMax[sensorIdx],0,1023,0,999),DEC);  
  }
  
  if (actionButton.getState()) {
    // released (state is HIGH)
    lcd.setCursor(19,3);
    lcd.print(0);
  } else {
    // pressed (state is LOW)
    lcd.setCursor(19,3);
    lcd.print(1);
  }
  
  if (calibrateButton.getState()) {
    // released (state is HIGH)
    lcd.setCursor(18,3);
    lcd.print(0);
  } else {
    // pressed (state is LOW)
    lcd.setCursor(18,3);
    lcd.print(1);
  }

  lcd.setCursor(0,3);
  if (SYS_MODE == SYS_IDLE) {
    lcd.print(F("IDLE           "));
  } else if (SYS_MODE == SYS_RUN) {
    lcd.print(F("RUN            "));
  } else if (SYS_MODE == SYS_CALIBRATE_LINE) {
    lcd.print(F("PUT ON LINE    "));
  } else if (SYS_MODE == SYS_CALIBRATE_BACKGROUND) {
    lcd.print(F("PUT ON BACKGRND"));
  }
  
  lcd.setCursor(17,3);
  if (FOLLOW_MODE == FOLLOW_DARK) {
    lcd.print(F("D"));
  } else {
    lcd.print(F("L"));
  }
  
  lcd.setCursor(13,3);
  lcd.print(BATTERY_ADC);
 
   if (BATTERY_STATE == BATTERY_WARN) {
     lcd.setCursor(10,3);
     lcd.print(F("BAT"));
   }
}
