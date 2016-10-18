#include <EEPROM.h>
#include <avr/wdt.h>

#include "pid.h"

// TODO LIST:
// Support to pedal with inverted signal (sensor 1 increase and sensor 2 decrease or vice-versa)
// Tests with POT pedal sensor
// Auto find min sensor value (today, is readed is setup function)
// *NOK* Save max pesal value * This can be a problem, if change the car
// *OK* Enable watchdog * WD enabled
// *NOK* Bypass when on idle * Not needed with PWM 10 bits
// *OK* Lower engineering margin :) (10% to 5% [?]) * 5%

// Enable/Disable serial debug
const bool DEBUG = false;

// Debug timer counter
unsigned int dbg_time_count = 0;
const unsigned long dbg_time = 50; // loops

// Boost mode 1 correction values (to use with map function)
const int MODE_1_MIN = 0;
const int MODE_1_MAX = 1535; // 50% (1023 to 1535)

// Boost mode 2 correction values (to use with map function)
const int MODE_2_MIN = 0;
const int MODE_2_MAX = 2046; // 100% (1023 to 2046)

// Max value to PWM
const int MAX_PWM = 1023;

// Number of times that ADC (sig0 and sig1) will be read to make a mean
const int BASE_VALUES_LOOP_COUNT = 10;

// Input signal pins, to read pedal position
const int sensorPot0Pin = A0;
const int sensorPot1Pin = A1;

// Feedback signal pins, to read and correct output signal
const int outputFeedback0Pin = A2;
const int outputFeedback1Pin = A3;

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

// Feedback values
int feedback0Value = 0;
int feedback1Value = 0;

// Maximum pedal value readed
int sensorPot0ValueMax = 0;
int sensorPot1ValueMax = 0;

// Output boost values
int output0Value = 0;
int output1Value = 0;

// Minimum pedal value, used to disable boost when the pedal is idle
//int sensorPot0CutValue = 0;
//int sensorPot1CutValue = 0;
int sensorPot0IdleValue = 0;
int sensorPot1IdleValue = 0;

// Proportion between sig0 and sig1
// Used to validate subsequent read
float sig_021_proportion = 2.0;

// Flag to control ADC initialization
bool adc_done = false;

bool need_full_init = false;

// Current boost mode
// 0 -> original
// 1 -> boost 1 (50%)
// 2 -> boost 2 (100%)
int mode = 0;

// Feedback PID
// Used to correct output signal
float Kp = 0.5;
float Ki = 0.5;
float Kd = 0.0;
PID fb_pid0 = PID(Kp, Ki, Kd);
PID fb_pid1 = PID(Kp, Ki, Kd);

// Configure digital pins 9 and 10 as 10-bit PWM outputs.
// With this config, PWM frequency is about 15khz
void setupPWM10bits() {
    DDRB |= _BV(PB1) | _BV(PB2);        // Set pins as outputs
    TCCR1A = _BV(COM1A1) | _BV(COM1B1)  // Non-inverting PWM
        | _BV(WGM11);                   // Mode 14: fast PWM, TOP=ICR1
    TCCR1B = _BV(WGM13) | _BV(WGM12)
        | _BV(CS10);                    // No prescaling
    ICR1 = 0x400;                       // TOP counter value (1024/10bits)
}

// 10-bit version of analogWrite()
void analogWrite10b(uint8_t pin, uint16_t val) {
    switch (pin) {
        case  9: OCR1A = val; break;
        case 10: OCR1B = val; break;
    }
}

void readBaseValues() {
  
  // Calculate initial idle pedal position
  // To avoid boost when pedal is not pressed
  // TODO: auto find idle position
  
  unsigned int adc_0_sum = 0;
  unsigned int adc_1_sum = 0;
  
  for(int i=0; i<BASE_VALUES_LOOP_COUNT; i++) {
    adc_0_sum += analogRead(sensorPot0Pin);
    adc_1_sum += analogRead(sensorPot1Pin);
  }
  
  sensorPot0IdleValue = (adc_0_sum / BASE_VALUES_LOOP_COUNT);
  sensorPot1IdleValue = (adc_1_sum / BASE_VALUES_LOOP_COUNT);
  
  // Engineering margin of safety :)
  // TODO: smooth transition
  //sensorPot0CutValue = adc_0_val * 1.05;
  //sensorPot1CutValue = adc_1_val * 1.05;
  
  // Sig1 to sig2 proportion
  // Theoretically, sensorPot1IdleValue (or sensorPot0IdleValue) never will be zero
  sig_021_proportion = sensorPot0IdleValue / sensorPot1IdleValue;
  
  //Serial.println(adc_0_sum);
  //Serial.println(adc_1_sum);
  //Serial.println(sensorPot0IdleValue);
  //Serial.println(sensorPot1IdleValue);
  //Serial.println(sig_021_proportion);
}

void enableBoost() {
  // Change mux/demux to use arduino signal
  digitalWrite(enableMuxPin, HIGH);
}

void initOutput() {
  
  // Only (at setup) put the input value to output pin
  // This will improves the PID
  
  sensorPot0Value = analogRead(sensorPot0Pin);
  sensorPot1Value = analogRead(sensorPot1Pin);
  
  output0Value = map(sensorPot0Value, 0, 1023, 0, MAX_PWM);
  output1Value = map(sensorPot1Value, 0, 1023, 0, MAX_PWM);
  
  analogWrite10b(output0Pin, output0Value);
  analogWrite10b(output1Pin, output1Value);
}

ISR (ADC_vect)
{
  
  if(!adc_done) {
    Serial.println ("done");
    adc_done = true;
    need_full_init = true;
  }
} 

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
  // *** Doesnt work with PWM 10-bit (with 10-bit, the frequency is always about 15kHz) ***
  // TCCR1B = TCCR1B & 0b11111000 | 0x01;
  
  // Enable ADC interrupt
  ADCSRA |= B00001000;
  
  // Init debug serial
  Serial.begin(9600);
  Serial.println("Serial initialized!");
  
  // Configure pins mode
  //pinMode(sensorPot0Pin, INPUT_PULLUP);
  //pinMode(sensorPot1Pin, INPUT_PULLUP);
  //pinMode(outputFeedback0Pin, INPUT_PULLUP);
  //pinMode(outputFeedback1Pin, INPUT_PULLUP);
  
  pinMode(ledPin0, OUTPUT);  
  pinMode(ledPin1, OUTPUT);
  pinMode(output0Pin, OUTPUT);
  pinMode(output1Pin, OUTPUT);
  pinMode(buttonModePin, INPUT);
  pinMode(enableMuxPin, OUTPUT);
  
  // Init PWM 10-bit
  setupPWM10bits();

  // Blink leds
  showBoot(0);
  
  // First ADC convertion
  // To initialize ADC... wait interruption to complete setup
  analogRead(sensorPot0Pin);
  
  // Load and show saved working mode
  mode = readMode();
  //showMode(mode);
}

// Arduino main loop
void loop() {

  if(!adc_done) {
    // Watchdog reset
    //wdt_reset();
    //Serial.println("Skip");
    return;
  }
  
  if(need_full_init) {
    
    need_full_init = false;
    
    // Blink leds
    showBoot(1);
    
    readBaseValues();
    initOutput();
    enableBoost();
  
    // Show saved working mode
    showMode(mode); 
    
    // Enable watchdog
    wdt_enable(WDTO_15MS);
  }
  
  //Serial.println(mode);
  
  // read feedback values (0-1023)
  feedback0Value = analogRead(outputFeedback0Pin);
  feedback1Value = analogRead(outputFeedback1Pin);
  
  // read pedal values (0-1023)
  sensorPot0Value = analogRead(sensorPot0Pin);
  sensorPot1Value = analogRead(sensorPot1Pin);
  
  // Compute max read value
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
      showMode(mode);
    }
  } else {
    pressed = false;
  }
  
  //Serial.println(mode);
  //Serial.println(sensorPot0Value);
  //Serial.println(sensorPot1Value);
  //Serial.println(sensorPot0CutValue);
  //Serial.println(sensorPot1CutValue);
  
  // TODO: 1.05 constant
  int sensorPot0CutValue = sensorPot0IdleValue * 1.05;
  int sensorPot1CutValue = sensorPot1IdleValue * 1.05;
  
  // Boost mode 1 (check if pedal is not in idle position, too)
  if(mode == 1 && sensorPot0Value > sensorPot0CutValue && sensorPot1Value > sensorPot1CutValue) {
    
    //Serial.println("BOOST 1");
    
    // Apply boost mode 1 correction
    
    int tmpOut0 = map(sensorPot0Value, 0, 1023, MODE_1_MIN, MODE_1_MAX);
    int tmpOut1 = map(sensorPot1Value, 0, 1023, MODE_1_MIN, MODE_1_MAX);
    
    // Veryfies if the corrected value isnt too big or too small
    // If the value is more than ECU max value (or min value), the ECU can put the engine in safe mode
    if (tmpOut0 > sensorPot0ValueMax) {
      tmpOut0 = sensorPot0ValueMax;
    }
    if (tmpOut1 > sensorPot1ValueMax) {
      tmpOut1 = sensorPot1ValueMax;
    }
    if (tmpOut0 < sensorPot0IdleValue) {
      tmpOut0 = sensorPot0IdleValue;
    }
    if (tmpOut1 < sensorPot1IdleValue) {
      tmpOut1 = sensorPot1IdleValue;
    }
    
    // Prepare to PWM
    output0Value = map(tmpOut0, 0, 1023, 0, MAX_PWM);
    output1Value = map(tmpOut1, 0, 1023, 0, MAX_PWM);
    
    
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

    // Veryfies if the corrected value isnt too big or too small
    // If the value is more than ECU max value (or min value), the ECU can put the engine in safe mode
    if (tmpOut0 > sensorPot0ValueMax) {
      tmpOut0 = sensorPot0ValueMax;
    }
    if (tmpOut1 > sensorPot1ValueMax) {
      tmpOut1 = sensorPot1ValueMax;
    }
    if (tmpOut0 < sensorPot0IdleValue) {
      tmpOut0 = sensorPot0IdleValue;
    }
    if (tmpOut1 < sensorPot1IdleValue) {
      tmpOut1 = sensorPot1IdleValue;
    }
    
    // Prepare to PWM
    output0Value = map(tmpOut0, 0, 1023, 0, MAX_PWM);
    output1Value = map(tmpOut1, 0, 1023, 0, MAX_PWM);
    
  } 
  // Boost mode 0 (no boost) or pedal is in idle position
  else {
    
    //Serial.println("BOOST 0");
    
    output0Value = map(sensorPot0Value, 0, 1023, 0, MAX_PWM);
    output1Value = map(sensorPot1Value, 0, 1023, 0, MAX_PWM);
    
  }
  
  //TODO: debug only
  if(sensorPot0Value > sensorPot0CutValue && sensorPot1Value > sensorPot1CutValue) {
    showNotIdle();
  } else {
    showMode(mode);
  }
  
  // Normalizes output
  if(output0Value > MAX_PWM) {
    output0Value = MAX_PWM; 
  }
  if(output1Value > MAX_PWM) {
    output1Value = MAX_PWM; 
  }
  if(output0Value < 0) {
    output0Value = 0; 
  }
  if(output1Value < 0) {
    output1Value = 0; 
  }
  
  // PID tests
  if (false) {
    
    double correction0 = 1.0;
    double correction1 = 1.0;
  
    correction0 = fb_pid0.process(map(feedback0Value, 0, 1023, 0, MAX_PWM), output0Value); // have / want
    correction1 = fb_pid1.process(map(feedback1Value, 0, 1023, 0, MAX_PWM), output1Value); // have / want

    int tmp0 = output0Value;
    int tmp1 = output1Value;
    
    output0Value += correction0;
    output1Value += correction1;
    
    if(output0Value > MAX_PWM) {
      output0Value = MAX_PWM; 
    }
    if(output1Value > MAX_PWM) {
      output1Value = MAX_PWM; 
    }
    if(output0Value < 0) {
      output0Value = 0; 
    }
    if(output1Value < 0) {
      output1Value = 0; 
    }
    
    if (DEBUG && isShowTime()) {
      
      //char tbs[16];
      //sprintf(tbs, "P%4dR%4dT%4d", x, y, z);
      
      //Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
      
      /*
      char buf[127];
      sprintf(buf, "0:%d(%d)[%d]", 
                    map(feedback0Value, 0, 1023, 0, MAX_PWM), 
                    output0Value, 
                    correction0);
      
      Serial.println(buf);
      
      char buf2[127];
      sprintf(buf2, "1:%d(%d)[%d]", 
                    map(feedback1Value, 0, 1023, 0, MAX_PWM), 
                    output1Value, 
                    correction1);
      
      Serial.println(buf2);
      */
      
      Serial.print("CH0: ");
      Serial.print(tmp0);
      Serial.print("->");
      Serial.print(output0Value);
      Serial.print(" (");
      Serial.print(map(feedback0Value, 0, 1023, 0, MAX_PWM));
      Serial.print(" [");
      Serial.print(correction0);
      Serial.println("])");
      
      
      Serial.print("CH1: ");
      Serial.print(tmp1);
      Serial.print("->");
      Serial.print(output1Value);
      Serial.print(" (");
      Serial.print(map(feedback1Value, 0, 1023, 0, MAX_PWM));
      Serial.print(" [");
      Serial.print(correction1);
      Serial.println("])");
      
      
      //Serial.println("=================================");
    }
  }
  
  analogWrite10b(output0Pin, output0Value);
  analogWrite10b(output1Pin, output1Value);
  
  // Watchdog reset
  wdt_reset();
  
  // Debug loop counter
  tickDbgTime();
  
  //TODO: need delay?
  //delay(10);
}

// Verifies if show debug in this execution loop
bool isShowTime() {
  return dbg_time_count == dbg_time;
}

// Debug time count
void tickDbgTime() {
  dbg_time_count += 1;
  if (dbg_time_count > dbg_time) {
    dbg_time_count = 0;
  }
}

// Change to next mode and store in eeprom
int nextMode() {
  
  int newVal = mode + 1;
  if(newVal > 2) {
    newVal = 0;
  }
  EEPROM.write(0, newVal);
  
  return newVal;
}

// Read mode from eeprom
int readMode() {
  int value = EEPROM.read(0);
  
  if(value > 2) {
    value = 0;
  }
  
  return value;
}

// Blink at startup
void showBoot(int type) {

  switch(type) {
    case 0:
      digitalWrite(ledPin0, HIGH);   
      digitalWrite(ledPin1, HIGH);
      delay(150);
      digitalWrite(ledPin0, LOW);   
      digitalWrite(ledPin1, LOW);
      delay(150);
    break;
    
    case 1:
      digitalWrite(ledPin0, HIGH);   
      digitalWrite(ledPin1, HIGH);
      delay(100);
      digitalWrite(ledPin0, LOW);   
      digitalWrite(ledPin1, LOW);
      delay(100);
      digitalWrite(ledPin0, HIGH);   
      digitalWrite(ledPin1, HIGH);
      delay(100);
      digitalWrite(ledPin0, LOW);   
      digitalWrite(ledPin1, LOW);
      delay(100);
    break;
  }
}

// Show current mode with leds
void showMode(int mode) {
  
  // TODO: use switch
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

void showNotIdle() {
  digitalWrite(ledPin0, HIGH);   
  digitalWrite(ledPin1, HIGH);
}
