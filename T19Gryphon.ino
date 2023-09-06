// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce-Arduino-Wiring

#include <Bounce2.h>
#include <Servo.h>


#include <Wire.h>

// Include the PWM library, to change the PWM Frequency for pin controlling the flywheel, found here :
// https://code.google.com/archive/p/arduino-pwm-frequency-library/downloads

#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"


//defining pin assignments
#define ESC_PIN                      9    // (Orange) PIN to control ESC 
#define PIN_REV                     5    // (White)  PIN listening to change in the Nerf Rev Button 
#define PIN_DARTTRIGGER             6    // (Purple) PIN listening to trigger pull event
#define PIN_SOLENOID                4    // (Purple) PIN to control solenoid
#define PIN_BUTTON                  10   // (Grey)   Button for changing menus
#define PIN_SELECTOR                11   // (Green)  Selector switch for Semi/(Burst/Full)
#define PIN_VOLTREAD                A6   // (Yellow) PIN to receive voltage reading from battery 
// Note                             A4      (Blue)   are used by the OLED SDA (Blue)
// Note                             A5      (Yellow) are used by the OLED SCL (Yellow)


#define MODE_SINGLE                 0    // Integer constant to indicate firing single shot
#define MODE_BURST                  1    // Integer constant to indicate firing burst
#define MODE_AUTO                   2    // Integer constant to indicate firing full auto


#define MAIN_MENU                   0    //main menu
#define ROF                         1    //ROF menu
#define AUTO_BURST                  2    //Selection of either burst/auto
#define BURST_LIMIT                 3    //selection of how many darts per burst
#define PWM                         4    //PWM menu


#define MAXSOLENOIDDELAY            100  //controls how slow of a ROF you can set 
#define REV_UP_DELAY                300  // Increase/decrease this to control the flywheel rev-up time (in milliseconds) 
#define MINSOLENOIDDELAY            42   //see delaySolenoidRetracted 42
#define DELAYSOLENOIDEXTENDED       22  //time for solenoid to fully extend 40
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define I2C_ADDRESS 0x3C

//flyshot
#define MOTOR_P_DIV2               7  // # motor poles divided by 2
  // Define the timings for the T0H, T1H, and TL pulses in microseconds
#define T0H  100
#define T1H  400
#define TL  500




SSD1306AsciiWire oled;



Servo ESC;
int     delayOffset               = 0;           //used to control ROF but not sacrifice semi performance
int     delaySolenoidRetracted    = MINSOLENOIDDELAY; //delay from when solenoid to fully retract to allow another extension
int     burstLimit                = 3;           // darts per burst
int     modeFire                  = MODE_SINGLE; // track the mode of fire, Single, Burst or Auto, Single by default
int     dartToBeFire              = 0;           // track amount of dart(s) to fire when trigger pulled

//int32_t frequency                 = 490;        //frequency (in Hz) for PWM controlling Flywheel motors
int     fwSpeed                   = 180;         //flywheel speed from 0-255
int     currfw                    = 180;
long    RPMsetting                = 25;         //in percentage
boolean isRevving                 = false;       // track if blaster firing
boolean isFiring                  = false;       // track if blaster firing
boolean isBurst                   = false;       // track selector switch behavior for burst/full.

unsigned long timerSolenoidDetect = 0;           //tracks how long solenoid has been extended/retracted
boolean       isSolenoidExtended  = false;
float   battVoltage;
unsigned long timer               = 0;
unsigned long fwtimer             = 0;
boolean setupBlaster = false;

int     currentState              = 4;
int     nextState                 = 0;
int     screenOffset              = 1;

String menus [] = {"MAIN MENU:", "Rate of fire", "AUTO/BURST", "Burst Settings", "RPM Settings", "Save and exit"};

// Declare and Instantiate Bounce objects
Bounce btnRev            = Bounce();
Bounce btnTrigger        = Bounce();
Bounce switchSelector    = Bounce();
Bounce switchButton      = Bounce();


//speed packet
void updateSpeed(long MotorRPM) {
  Serial.println(MotorRPM);
  ESC.detach();
  // pinMode(9,OUTPUT);
  //MotorRPM = 5000;
  unsigned long SetPoint = (unsigned long)320000000 / (MotorRPM * MOTOR_P_DIV2);
  unsigned int packet = SetPoint | 0x8000;
  Serial.println(packet,BIN);
  // Send the packet using the 4-level protocol
  //packet = 0b1000000011100000;
  
  for (int pksend = 0; pksend < 10;pksend++){
  // Send the leading throttle-range pulse
  digitalWrite(ESC_PIN, HIGH);
  delayMicroseconds(1000);
  
  digitalWrite(ESC_PIN,LOW);
  delayMicroseconds(10);
  
  // Send the packet MSB first
  for (int i = 15; i >= 0; i--) {
    //Serial.println(packet & (0x0001 << i),BIN);
    if(packet & (0x0001 << i)){
      
      // Send a T1H pulse
      digitalWrite(ESC_PIN, HIGH);
      delayMicroseconds(T1H);
      digitalWrite(ESC_PIN, LOW);
      delayMicroseconds(TL);
    } else {
      // Send a T0H pulse
      digitalWrite(ESC_PIN, HIGH);
      delayMicroseconds(T0H);
      digitalWrite(ESC_PIN, LOW);
      delayMicroseconds(TL);
    }
  }



  // Send the trailing throttle-range pulse
  digitalWrite(ESC_PIN, HIGH);
  delayMicroseconds(1000);
  digitalWrite(ESC_PIN, LOW);
  delayMicroseconds(10);
  }

  ESC.attach(9,1000,2000);

}




// Function: shotFiredHandle

void shotFiringHandle() {

  //allows the blaster to have full semi perfomance while still controlling ROF
  if (modeFire == MODE_SINGLE) {
    delaySolenoidRetracted = MINSOLENOIDDELAY;
  } else {
    delaySolenoidRetracted = MINSOLENOIDDELAY + delayOffset;
  }


  if (isFiring) {
    if (isSolenoidExtended) {
      if ((millis() - timerSolenoidDetect) >= DELAYSOLENOIDEXTENDED) {
        digitalWrite(PIN_SOLENOID, LOW); // Retract Solenoid
        dartToBeFire--;
        isSolenoidExtended = false;
        timerSolenoidDetect = millis();
      }
    } else { // Solenoid had returned to rest position
      if (dartToBeFire == 0) { //done firing
        isFiring = false;
        if (!isRevving) { // Rev button not pressed
          isRevving = false;
          //digitalWrite(ESC_PIN, LOW);
          //pwmWrite(ESC_PIN, 128);// stop flywheels
        }
      } else if ((millis() - timerSolenoidDetect) >= delaySolenoidRetracted) {
        digitalWrite(PIN_SOLENOID, HIGH); // Extend Solenoid
        isSolenoidExtended = true;
        timerSolenoidDetect = millis();
      }
    }
  }
}


// Function: triggerPressedHandle

void triggerPressedHandle(int caseModeFire) {
  //if (!isRevving) {
    //currfw = fwSpeed;
    //ESC.write(fwSpeed);
      //pwmWrite(ESC_PIN, fwSpeed); // start flywheels
    //digitalWrite(ESC_PIN, HIGH);
    //delay(REV_UP_DELAY);
    //isRevving = true;
  //}

  switch (caseModeFire) {
    case MODE_SINGLE: dartToBeFire++; break;
    case MODE_BURST : dartToBeFire += burstLimit; break;
    case MODE_AUTO  : dartToBeFire += 100; break; //generic value
  }

  // Start Firing
  if (!isFiring) {
    isFiring = true;
    digitalWrite(PIN_SOLENOID, HIGH); // extend pusher
    timerSolenoidDetect = millis();
    isSolenoidExtended = true;

  }
}


// Function: triggerReleasedHandle

void triggerReleasedHandle() {
  if (((modeFire == MODE_AUTO) || (modeFire == MODE_BURST)) && isFiring && (dartToBeFire > 1)) {
    dartToBeFire = 1;    // fire off last shot
  }
}


// Function: readVoltage

void readVoltage() {
  // you might have to adjust the formula according to the voltage sensor you use
  battVoltage = ((analogRead(PIN_VOLTREAD) * 0.259)-8); //converts digital to a voltage

}


// Function: updateDisplay

void updateDisplay() {

  readVoltage();
  oled.clear();
  oled.set2X();
  oled.setCursor(0, 0 + screenOffset);
  oled.println(battVoltage / 10, 1);
  oled.set1X();
  switch (modeFire) {
    case (MODE_SINGLE):
      oled.println("SEMI");
      break;
    case (MODE_BURST):
      oled.println("BURST");
      break;
    case (MODE_AUTO):
      oled.println("AUTO");
      break;
  }
  oled.print("ROF: ");
  oled.println(delayOffset);
  oled.print("Burst Limit: ");
  oled.println(burstLimit);
  oled.print("RPM: ");
  oled.println(RPMsetting);
}




// Function: main menu

void menu() {
  oled.set1X();
  for (int i = 0; i <= 50; i += 10) {
    oled.setCursor(0, i + screenOffset);
    oled.println(menus[i / 10]);
  }
  unsigned long menuTime  = millis();
  boolean showCursor = true;
  int menuCursor = 10;

  while (1) {
    if (millis() - menuTime >= 500) {
      menuTime = millis();
      oled.clear();
      if (showCursor) {
        showCursor = false;
      } else {
        showCursor = true;
      }
      for (int i = 0; i <= 50; i += 10) {
        oled.setCursor(0, i+ screenOffset);
        if (showCursor && i == menuCursor) {
          oled.print(menus[i / 10]);
          oled.println("<<");
        } else {
          oled.setCursor(0, i + screenOffset);
          oled.println(menus[i / 10]);
        }
      }
    }

    btnTrigger.update(); //trigger acts as "confirm selection"
    switchButton.update(); //button acts as "change value"
    if (switchButton.fell()) {
      if (menuCursor == 50) {
        menuCursor = 10;
        menuTime -= 500;
      } else {
        menuCursor += 10;
        menuTime -= 500; //so it updates the cursor location
      }
    }
    if (btnTrigger.fell()) {
      nextState = menuCursor / 10;
      break;
    }
  }
}

// Function: Change Values

void changeValue() {
  oled.clear();
  oled.set1X();
  oled.setCursor(0, 0 + screenOffset);
  oled.println(menus[currentState]);
  
  switch (currentState) {
    case (ROF):
      oled.setCursor(0, 10 + screenOffset);
      oled.println("Max value: 100");
      oled.println("Min Value: 0");
      oled.print(delayOffset);
      break;
      
    case (AUTO_BURST):
      oled.setCursor(0, 10 + screenOffset);
      oled.println("Burst: True");
      oled.println("Auto: False");
      oled.print(isBurst ? "True" : "False");
      break;
      
    case (BURST_LIMIT):
      oled.setCursor(0, 10 + screenOffset);
      oled.println("Max value: 10");
      oled.println("Min Value: 2");
      oled.print(burstLimit);
      break;
      
    case (PWM):
      oled.setCursor(0, 10 + screenOffset);
      oled.println("Max value: 40k");
      oled.println("Min Value: 10k");
      oled.println(RPMsetting);
      break;
  }

  while (1) {
    btnTrigger.update(); //trigger acts as "confirm selection"
    switchButton.update(); //button acts as "change value"
    if (switchButton.fell()) {
      switch (currentState) {
        
        case (ROF):
          oled.clear();
          oled.setCursor(0, screenOffset);
          oled.println(menus[currentState]);
          oled.println("Max value: 100");
          oled.println("Min Value: 0");
          delayOffset += 10;
          if (delayOffset > 100) {
            delayOffset = 0;
          }
          oled.print(delayOffset);
          break;
          
        case (AUTO_BURST):
          oled.clear();
          oled.setCursor(0, screenOffset);
          oled.println(menus[currentState]);
          oled.println("Burst: True");
          oled.println("Auto: False");
          if (isBurst) {
            isBurst = false;
          } else {
            isBurst = true;
          }
          oled.print(isBurst ? "True" : "False");
          break;
          
        case (BURST_LIMIT):
          oled.clear();
          oled.setCursor(0, screenOffset);
          oled.println(menus[currentState]);
          oled.println("Max value: 10");
          oled.println("Min Value: 2");
          burstLimit++;
          if (burstLimit > 10) {
            burstLimit = 2;
          }
          oled.print(burstLimit);
          break;
          
        case (PWM):
          oled.clear();
          oled.setCursor(0, screenOffset);
          oled.println(menus[currentState]);
          oled.println("Max value: 40k");
          oled.println("Min Value: 10k");
          RPMsetting++;
          if (RPMsetting > 40) {
            RPMsetting = 10;
          }
          oled.println(RPMsetting);
          break;
      }
    }
    if (btnTrigger.fell()) {
      nextState = 0;
      break;
    }
  }
}





void setup() { // initilze
  //sets up the setup loop if trigger is pulled
  
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.displayRemap(true);
  Serial.begin(9600);
  ESC.attach(9,1000,2000);
  //initialize all timers except for 0, to save time keeping functions
  //InitTimersSafe();


  //sets the frequency for the specified pin
  //bool success = SetPinFrequencySafe(ESC_PIN, frequency);

  // INPUT PINs setup
  // Note: Most input pins will be using internal pull-up resistor. A fall in signal indicate button pressed.


  pinMode(PIN_REV, INPUT_PULLUP);             // PULLUP
  btnRev.attach(PIN_REV);
  btnRev.interval(5);                         //debounce period

  pinMode(PIN_DARTTRIGGER, INPUT_PULLUP);     // PULLUP
  btnTrigger.attach(PIN_DARTTRIGGER);
  btnTrigger.interval(5);

  pinMode(PIN_SELECTOR, INPUT_PULLUP);    // PULLUP
  switchSelector.attach(PIN_SELECTOR);
  switchSelector.interval(5);

  pinMode(PIN_BUTTON, INPUT_PULLUP);    // PULLUP
  switchButton.attach(PIN_BUTTON);
  switchButton.interval(5);

  pinMode(PIN_VOLTREAD, INPUT); 


  // OUTPUT PINs setup

  //pinMode (ESC_PIN, OUTPUT);
  //pwmWrite(ESC_PIN, 255);
  //delay(3000);
  //pwmWrite(ESC_PIN, 128);
  //digitalWrite(ESC_PIN, LOW);
  pinMode(PIN_SOLENOID, OUTPUT);
 // digitalWrite(PIN_SOLENOID, LOW);


  if (digitalRead(PIN_SELECTOR) == LOW) {
    modeFire = MODE_AUTO;
  } else {
    modeFire = MODE_SINGLE;
  }
  oled.clear();
  updateDisplay();
  timer = millis();
  //digitalWrite(PIN_SOLENOID, HIGH); // extend pusher
  //delay(3000);
  ESC.write(180);
  delay(3000);
  ESC.write(0);
  delay(3000);
  //digitalWrite(PIN_SOLENOID, LOW); // extend pusher
  Serial.println("finsihed setup");


 updateSpeed(25000);
}


// Function: loop

void loop() { // Main Loop



  btnRev.update();
  btnTrigger.update();
  switchSelector.update();
  switchButton.update();


  // Listen to Rev Press/Release

  if (btnRev.fell()) {// press
    Serial.println("rev ouill");
    isRevving = true;
    currfw = fwSpeed;
    ESC.write(currfw);
     //pwmWrite(ESC_PIN, fwSpeed);
    //digitalWrite(ESC_PIN, HIGH);
  } else if (btnRev.rose()) {        // released
    isRevving = false;
    fwtimer = millis();
    currfw = fwSpeed - 10;
    ESC.write(currfw);
   // pwmWrite(ESC_PIN, currfw);

    /*
    if (!isFiring) {
      pwmWrite(ESC_PIN, 128);
      //digitalWrite(ESC_PIN, LOW);
    }
    */
  }

  if (!isRevving && currfw > 0){ //if flywheel still gaming
    if (millis() - fwtimer > 10){
      currfw = currfw - 5;
      if (currfw < 0){
        currfw = 0;
      }
      ESC.write(0);
     // pwmWrite(ESC_PIN, currfw);
    }
  }

  // Listen to Trigger Pull/Release

  if (btnTrigger.fell()) {               // pull
    Serial.println("Trigger pull");
    triggerPressedHandle(modeFire);
  } else if (btnTrigger.rose()) {        // released
    triggerReleasedHandle();
  }


  // Listen to Firing

  shotFiringHandle();

  // Listen to Firing Mode change: Single Shot, Burst, Full Auto

  if (switchSelector.changed()) {
    if (switchSelector.read()) { //if selector is in full/burst position
      if (isBurst) {
        modeFire = MODE_BURST;
      } else {
        modeFire = MODE_AUTO;
      }
    } else { // single
      modeFire = MODE_SINGLE;
    }
    updateDisplay();
    timer = millis();
  }


  if (switchButton.fell()) {
    timer = millis(); 
    setupBlaster = true;
  }

  if (switchButton.rose()) {
    setupBlaster = false;
    
  }

  if (setupBlaster && ((millis() - timer) > 3000)) { //only enters setup if switchbutton held for 3s
    while (setupBlaster) {
      switch (nextState) {
        case (MAIN_MENU):
          if (nextState != currentState) {
            currentState = nextState;
            menu();
          }
          break;
        case (5):
         currentState = 4;
         nextState = 0;
          setupBlaster = false; //exit case
          updateSpeed(RPMsetting * 1000);
          break;
        default:
          if (nextState != currentState) {
            currentState = nextState;
            changeValue();
          }
      }
    }
  }

  if (millis() - timer > 10000) { //only waste time updating every 5000 seconds
    updateDisplay();
    timer = millis();
  }



}
