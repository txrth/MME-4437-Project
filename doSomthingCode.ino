//
// MME 4487 project TCS34725 colour sensor testing code
//
//  Language: Arduino (C++)
//  Target:   ESP32
//  Author: Tirth Patel
//  Co-Author:   Michael Naish
//  Date:     2023 11 23
//

#define PRINT_COLOUR  // uncomment to turn on output of colour sensor data

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"


int servoAngle;
// Function declarations
void doHeartbeat();

// Constants
const int cHeartbeatLED = 2;          // GPIO pin of built-in LED for heartbeat
const int cHeartbeatInterval = 1000;  // heartbeat blink interval, in milliseconds
const int cTCSLED = 23;               // GPIO pin for LED on TCS34725
const int servoPin = 15;
const int servoChannel = 5;
// Variables
unsigned long lastHeartbeat = 0;  // time of last heartbeat state change
unsigned long lastTime = 0;       // last time of motor control was updated


/**
    Calibration 
      White background: R: 24 G:70 B:56 C: 176
      Natural background: R:42  G:70 B:63 C: 176// R: 20 G: 24 B: 20
      Black background: R: 23 G:33 B:28 C: 88
  
  */

const int cali[] = { 27, 36, 31 };  // r g b
const int tol = 2;
const int actDelay = 1000;

long lastActTime;






// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS3 4725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;  // TCS34725 flag: 1 = connected; 0 = not found



void setup() {
  Serial.begin(115200);            // Standard baud rate for ESP32 serial monitor
  pinMode(cHeartbeatLED, OUTPUT);  // configure built-in LED for heartbeat
  pinMode(cTCSLED, OUTPUT);        // configure GPIO for control of LED on TCS34725
  ledcAttachPin(servoPin, servoChannel);
  ledcSetup(servoChannel, 50, 16);
  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
    digitalWrite(cTCSLED, 1);  // turn on onboard LED
  } else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}

void loop() {
  Serial.begin(115200);  // Standard baud rate for ESP32 serial monitor
  uint16_t r, g, b, cor;   // RGBC values from TCS34725
  double c; 
  if (tcsFlag) {                     // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &cor);  // get raw RGBC values
                                     //#ifdef PRINT_COLOUR
    c=cor;
    Serial.printf("R: %f, G: %f, B: %f, C %f, A %d\n", r/c*100, g/c*100, b/c*100, c, servoAngle);
    //#endif
  }
    //Serial.printf("R: %d, %d  G: %d, %d B: %d, %d",r/c >= cali[0] - tol && r/c <= cali[0] + tol && g/c >= cali[1] - tol && g/c <= cali[1] + tol && b/c >= cali[2] - tol && b/c <= cali[2] + tol)
  if (r/c*100 >= cali[0] - tol && r/c*100 <= cali[0] + tol && g/c*100 >= cali[1] - tol && g/c*100 <= cali[1] + tol && b/c*100 >= cali[2] - tol && b/c*100 <= cali[2] + tol) {
      //servoAngle = 180;
      Serial.println("yes");
  }

  unsigned long curMillis = millis();
  
  if (digitalRead(cHeartbeatLED)){
    servoAngle= 180;
  }else{
    servoAngle=90;
  }
  //if ((curMillis - lastActTime) > actDelay) {
    lastActTime = curMillis;
    ledcWrite(servoChannel, degreesToDutyCycle(servoAngle));
     Serial.println("done");
    //servoAngle = 90;
  //}

  //Serial.print(degreesToDutyCycle(servoAngle));
  ledcWrite(servoChannel, degreesToDutyCycle(servoAngle));
  doHeartbeat();  // update heartbeat LED
}

// blink heartbeat LED
void doHeartbeat() {
  unsigned long curMillis = millis();  // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                                 // update the heartbeat toggle time for the next cycle
    digitalWrite(cHeartbeatLED, !digitalRead(cHeartbeatLED));  // toggle state of LED
  }
}

// Converts servo position in degrees into the required duty cycle for an RC servo motor control signal
// assuming 16-bit resolution (i.e., value represented as fraction of 65535).
// Note that the constants for minimum and maximum duty cycle may need to be adjusted for a specific motor

long degreesToDutyCycle(int deg) {
  const long cl_MinDutyCycle = 1650;  // duty cycle for 0 degrees
  const long cl_MaxDutyCycle = 8175;  // duty cycle for 180 degrees

  long l_DutyCycle = map(deg, 0, 180, cl_MinDutyCycle, cl_MaxDutyCycle);  // convert to duty cycle

#ifdef OUTPUT_ON
  float f_Percent = l_DutyCycle * 0.0015259;  // dutyCycle / 65535 * 100
  Serial.printf("Degrees %d, Duty Cycle Val: %ld = %f%%\n", i_ServoPos, l_DutyCycle, f_Percent);
#endif

  return l_DutyCycle;
}