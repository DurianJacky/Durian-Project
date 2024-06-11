/*
  CH1 and CH2 is used for motion control;
  CH5 is E-stop switch;
  CH6 is the linear and angular motion switch.
*/

#include <math.h>

// Define Input Connections
#define CH1 3
#define CH2 5
#define CH3 6
#define CH4 9
#define CH5 10
#define CH6 11

// Integers to represent values from sticks and pots
int ch1Value;
int ch2Value;
int ch3Value;
int ch4Value; 


// Boolean to represent switch value
bool ch5Value;
bool ch6Value;

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

// when the input is reletively small, the value is set 0
// when the input is reletively high, a limit is set by 95
float filter(float x) {
    if (x > -8 && x < 8) {
        return 0;
    } else if (x <= -95) {
        return -95;
    } else if (x >= 95) {
        return 95;
    } else {
        return x;
    }
}


void setup(){
  // Set up serial monitor
  Serial.begin(115200);
  
  // Set all pins as inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
}


void loop() {
  
  // Get values for each channel
  ch1Value = readChannel(CH1, -100, 100, 0);
  ch2Value = readChannel(CH2, -100, 100, 0);
  ch3Value = readChannel(CH3, -100, 100, 0);
  ch4Value = readChannel(CH4, -100, 100, 0);
  ch5Value = readSwitch(CH5, false); // SWA --> E-stop
  ch6Value = readSwitch(CH6, false); // VRB --> linear / angular
  
  // Print to Serial Monitor
  Serial.print(" Status: ");
//  Serial.print("Ch1: ");
//  Serial.print(ch1Value);
//  Serial.print(" | Ch2: ");
//  Serial.print(ch2Value);
//  Serial.print(" | Ch3: ");
//  Serial.print(ch3Value);
//  Serial.print(" | Ch4: ");
//  Serial.print(ch4Value);
//  Serial.print(" | Ch5: ");
//  Serial.print(ch5Value);
//  Serial.print(" | Ch6: ");
//  Serial.print(ch6Value);
  
  

  // 逻辑
  // filter the channel value
  
//  float ch1 = filter(ch1Value);
//  float ch2 = filter(ch2Value);
  float ch1 = ch1Value;
  float ch2 = ch2Value;
  
  float Rx = filter(sqrt(2)/2*ch1 - sqrt(2)/2*ch2);
  float Ry = filter(sqrt(2)/2*ch1 + sqrt(2)/2*ch2);
  Serial.print(" Rx: ");
  Serial.print(Rx);
  Serial.print(" | Ry: ");
  Serial.print(Ry);

  float e_x = 0;
  float e_y = 0;
  float e_psi = 0;

  if (ch6Value == 0) {
    e_x = Rx;
    e_y = Ry;
  }
  else if (ch6Value == 1){
    e_psi = Rx;
  }
  Serial.print(" | E-stop: ");
  Serial.print(ch5Value);
  Serial.print(" | e_x: ");
  Serial.print(e_x);
  Serial.print(" | e_y: ");
  Serial.print(e_y);
  Serial.print(" | e_psi: ");
  Serial.println(e_psi);

  
  delay(500);
}
