#include <EEPROM.h>

// Boost mode 1 correction values
const int MODE_1_MIN = 0;
const int MODE_1_MAX = 1536; // 50%

// Boost mode 2 correction values
const int MODE_2_MIN = 0;
const int MODE_2_MAX = 2048; // 100%

// Input signal pins, to read pedal position
const int sensorPot0Pin = A0;
const int sensorPot1Pin = A1;

// Led mode pins
const int ledPin0 = 6;
const int ledPin1 = 7;

// Output signal pins
const int output0Pin = 9;
const int output1Pin = 10;

// Button pin (used to change boost mode)
const int buttonModePin = 2;

// Control pin, to enable/disable mux
const int enableMuxPin = 11;

// Pedal values
int sensorPot0Value = 0;
int sensorPot1Value = 0;

// Maximum pedal value readed
int sensorPot0ValueMax = 0;
int sensorPot1ValueMax = 0;

// Output boost values
int output0Value = 0;
int output1Value = 0;

// Minimum pedal value, used to disable boost when the pedal is idle
int sensorPot0CutValue = 0;
int sensorPot1CutValue = 0;

// Current boost mode
// 0 -> original
// 1 -> boost 1 (50%)
// 2 -> boost 2 (100%)
int mode = 0;

// Arduino setup
void setup() {
  
  // Changing Timer1 frequency to 31372.55Hz
  // This change the PWM (of analogWrite) frequency
  // This improves RC low pass filter
  //
  // Documentation about Timer frequency change:
  // http://playground.arduino.cc/Main/TimerPWMCheatsheet
  // Online tool to calculate RC low pass filter:
  // http://sim.okawa-denshi.jp/en/PWMtool.php
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  
  // Init debug serial
  Serial.begin(9600);
  
  // Configure pins mode
  
  //pinMode(sensorPot0Pin, INPUT_PULLUP);
  //pinMode(sensorPot1Pin, INPUT_PULLUP);
  pinMode(ledPin0, OUTPUT);  
  pinMode(ledPin1, OUTPUT);
  pinMode(output0Pin, OUTPUT);
  pinMode(output1Pin, OUTPUT);
  pinMode(buttonModePin, INPUT);
  pinMode(enableMuxPin, OUTPUT);
  
  // To avoid boost when pedal is not pressed
  sensorPot0CutValue = analogRead(sensorPot0Pin);
  sensorPot1CutValue = analogRead(sensorPot1Pin);
  // Engineering margin of safety :)
  sensorPot0CutValue = sensorPot0CutValue * 1.1;
  sensorPot1CutValue = sensorPot1CutValue * 1.1;
  
  // Load and show saved working mode
  mode = readMode();
  showMode(mode);
}

// Arduino main loop
void loop() {

  // read pedal values
  sensorPot0Value = analogRead(sensorPot0Pin);
  sensorPot1Value = analogRead(sensorPot1Pin);
  
  if (sensorPot0Value > sensorPot0ValueMax) {
    sensorPot0ValueMax = sensorPot0Value;
  }
  if (sensorPot1Value > sensorPot1ValueMax) {
    sensorPot1ValueMax = sensorPot1Value;
  }
  
  // read button mode
  int buttonModeState = digitalRead(buttonModePin);

  // if pressed, change the boost mode
  static boolean pressed = false;
  if (buttonModeState == HIGH) {
    if(!pressed) {
      pressed = true;
      // Change to next boost mode
      mode = nextMode();
    }
  } else {
    pressed = false;
  }
  
  //Serial.println(mode);
  //Serial.println(sensorPot0Value);
  //Serial.println(sensorPot1Value);
  //Serial.println(sensorPot0CutValue);
  //Serial.println(sensorPot1CutValue);
  
  // Boost mode 1 (check if pedal is not in idle position, too)
  if(mode == 1 && sensorPot0Value > sensorPot0CutValue && sensorPot1Value > sensorPot1CutValue) {
    
    //Serial.println("BOOST 1");
    
    // Apply boost mode 1 correction
    
    int tmpOut0 = map(sensorPot0Value, 0, 1023, MODE_1_MIN, MODE_1_MAX);
    int tmpOut1 = map(sensorPot1Value, 0, 1023, MODE_1_MIN, MODE_1_MAX);
    
    // Veryfies if the corrected value isnt too big
    // If the value is more than ECU max value, the ECU can put the engine in safe mode
    if (tmpOut0 > sensorPot0ValueMax) {
      tmpOut0 = sensorPot0ValueMax;
    }
    if (tmpOut1 > sensorPot1ValueMax) {
      tmpOut1 = sensorPot1ValueMax;
    }
    
    // Prepare to PWN (0-255)
    output0Value = map(tmpOut0, 0, 1023, 0, 255);
    output1Value = map(tmpOut1, 0, 1023, 0, 255);
    
    
    //Serial.println(tmpOut0);
    //Serial.println(output0Value);
    //Serial.println();
  } 
  // Boost mode 2 (check if pedal is not in idle position, too)
  else if(mode == 2 && sensorPot0Value > sensorPot0CutValue && sensorPot1Value > sensorPot1CutValue) {
    
    //Serial.println("BOOST 2");
    
    // Apply boost mode 2 correction
    
    int tmpOut0 = map(sensorPot0Value, 0, 1023, MODE_2_MIN, MODE_2_MAX);
    int tmpOut1 = map(sensorPot1Value, 0, 1023, MODE_2_MIN, MODE_2_MAX);

    // Veryfies if the corrected value isnt too big
    // If the value is more than ECU max value, the ECU can put the engine in safe mode
    if (tmpOut0 > sensorPot0ValueMax) {
      tmpOut0 = sensorPot0ValueMax;
    }
    if (tmpOut1 > sensorPot1ValueMax) {
      tmpOut1 = sensorPot1ValueMax;
    }
    
    // Prepare to PWN (0-255)    
    output0Value = map(tmpOut0, 0, 1023, 0, 255);
    output1Value = map(tmpOut1, 0, 1023, 0, 255);
    
  } 
  // Boost mode 0 (no boost) or pedal is in idle position
  else {
    
    //Serial.println("BOOST 0");
    
    output0Value = map(sensorPot0Value, 0, 1023, 0, 255);
    output1Value = map(sensorPot1Value, 0, 1023, 0, 255);
    
  }
  
  if(output0Value > 255) {
    output0Value = 255; 
  }
  if(output1Value > 255) {
    output1Value = 255; 
  }
  if(output0Value < 0) {
    output0Value = 0; 
  }
  if(output1Value < 0) {
    output1Value = 0; 
  }
  
  analogWrite(output0Pin, output0Value);
  analogWrite(output1Pin, output1Value);
  
  showMode(mode);
  
  //Serial.println();
  
  delay(10);
  
  digitalWrite(enableMuxPin, HIGH);
}

int nextMode() {
  
  int newVal = mode + 1;
  if(newVal > 2) {
    newVal = 0;
  }
  EEPROM.write(0, newVal);
  
  return newVal;
}

int readMode() {
  int value = EEPROM.read(0);
  
  if(value > 2) {
    value = 0;
  }
  
  return value;
}

void showMode(int mode) {
  
  if(mode == 0) {
    digitalWrite(ledPin0, LOW);   
    digitalWrite(ledPin1, LOW);
  } else if(mode == 1) {
    digitalWrite(ledPin0, HIGH);   
    digitalWrite(ledPin1, LOW);
  } else if(mode == 2) {
    digitalWrite(ledPin0, LOW);   
    digitalWrite(ledPin1, HIGH);
  }
}





