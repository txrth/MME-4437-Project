//
// MME 4487 project Code
//
//  Language: Arduino (C++)
//  Target:   ESP32
//  Author: Tirth Patel
//  Co-Author:   Michael Naish
//  Date:     2023 11 25
//

#define SERIAL_STUDIO  // print formatted string, that can be captured and parsed by Serial-Studio
//#define PRINT_SEND_STATUS  // uncomment to turn on output packet send status
#define PRINT_INCOMING  // uncomment to turn on output of incoming data
//#define PRINT_COLOUR  // uncomment to turn on output of colour sensor data

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void ARDUINO_ISR_ATTR encoderISR(void *arg);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
long degreesToDutyCycle(int deg);

// Control data packet structure
struct ControlDataPacket {
  int dir;    // drive direction: 1 = forward, -1 = reverse, 0 = stop
  int speed;  // pot input for speed
  int turn;   // turn 1 for L -1 for R 1
  int dump;
  unsigned long time;  // time packet sent
};

// Drive data packet structure
struct DriveDataPacket {
  unsigned long time;  // time packet sent
  int data[14];
};

// Encoder structure
struct Encoder {
  const int chanA;  // GPIO pin for encoder channel A
  const int chanB;  // GPIO pin for encoder channel B
  long pos;         // current encoder position
};

// Constants
const int cHeartbeatLED = 2;              // GPIO pin of built-in LED for heartbeat
const int cStatusLED = 27;                // GPIO pin of communication status LED
const int cHeartbeatInterval = 500;       // heartbeat blink interval, in milliseconds
const int cNumMotors = 3;                 // Number of DC motors
const int cIN1Pin[] = { 17, 19, 33 };     // GPIO pin(s) for INT1
const int cIN1Chan[] = { 0, 1, 2 };       // PWM channe(s) for INT1
const int cIN2Pin[] = { 16, 18, 32 };     // GPIO pin(s) for INT2
const int cIN2Chan[] = { 3, 4, 5 };       // PWM channel(s) for INT2
const int cPWMRes = 8;                    // bit resolution for PWM
const int cMinPWM = 0;                    // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;  // PWM value for maximum speed
const int cPWMFreq = 20000;               // frequency of PWM signal
const int cCountsRev = 1096;              // encoder pulses per motor revolution
const int cMaxSpeedInCounts = 2500;       // maximum encoder counts/sec
const int cMaxChange = 14;                // maximum increment in counts/cycle
const int cMaxDroppedPackets = 20;        // maximum number of packets allowed to drop
const float kp = 1.5;                     //1.5                    // proportional gain for PID
const float ki = .2;                      //.2                     // integral gain for PID
const float kd = .8;                      //.3                    // derivative gain for PID
const int cTCSLED = 23;                   // GPIO pin for LED on TCS34725
const int servo1Pin = 14;                 // GPIO pin for servo1
const int servo1Channel = 6;              //
const int servo2Pin = 13;                 //
const int servo2Channel = 7;              //GPIO pin for servo2


/**
    Calibration 
        if c >70 == detection
        if [rgb] ./c = 28,36,29 == green
  */

const int cali[3] = { 28, 36, 29 };  // r g b
const int tol = 1;
const int actDelay = 1000;


// Variables
int servo1Angle = 0;
int servo2Angle = 0;
//int servo3Angle;
//int servo4Angle;
unsigned long lastHeartbeat = 0;      // time of last heartbeat state change
unsigned long lastTime = 0;           // last time of motor control was updated
unsigned int commsLossCount = 0;      // number of sequential sent packets have dropped
Encoder encoder[] = { { 26, 25, 0 },  // encoder 0 on GPIO 25 and 26, 0 position
                      { 35, 34, 0 },
                      { 36, 39, 0 } };  // encoder 1 on GPIO 32 and 33, 0 position
long target[] = { 0, 0, 0 };            // target encoder count for motor
long lastEncoder[] = { 0, 0, 0 };       // encoder count at last control cycle
float targetF[] = { 0.0, 0.0, 0.0 };    // target for motor as float
long lastActTime;

ControlDataPacket inData;   // control data packet from controller
DriveDataPacket driveData;  // data packet to send controller

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;  // TCS34725 flag: 1 = connected; 0 = not found

// REPLACE WITH MAC ADDRESS OF YOUR CONTROLLER ESP32
uint8_t receiverMacAddress[] = { 0xB8, 0xD6, 0x1A, 0x68, 0x16, 0x0C };  // MAC address of controller 0x08, 0xD1, 0xF9, 0x98, 0x8A, 0x38   // 0xFC, 0xB4, 0xB7, 0x50, 0xCD, 0x4C
esp_now_peer_info_t peerInfo = {};                                      // ESP-NOW peer information

void setup() {
  Serial.begin(115200);  // Standard baud rate for ESP32 serial monitor
  WiFi.mode(WIFI_STA);   // Use WiFi in station mode
  Serial.print("MAC address ");
  Serial.println(WiFi.macAddress());  // print MAC address of ESP32
  WiFi.disconnect();                  // disconnect from network

  pinMode(cHeartbeatLED, OUTPUT);  // configure built-in LED for heartbeat
  pinMode(cStatusLED, OUTPUT);     // configure GPIO for communication status LED as output
  pinMode(cTCSLED, OUTPUT);        // configure GPIO for control of LED on TCS34725

  //Servo
  ledcAttachPin(servo1Pin, servo1Channel);
  ledcSetup(servo1Channel, 50, 16);

  ledcAttachPin(servo2Pin, servo2Channel);
  ledcSetup(servo2Channel, 50, 16);




  // setup motors with encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttachPin(cIN1Pin[k], cIN1Chan[k]);     // attach INT1 GPIO to PWM channel
    ledcSetup(cIN1Chan[k], cPWMFreq, cPWMRes);  // configure PWM channel frequency and resolution
    ledcAttachPin(cIN2Pin[k], cIN2Chan[k]);     // attach INT2 GPIO to PWM channel
    ledcSetup(cIN2Chan[k], cPWMFreq, cPWMRes);  // configure PWM channel frequency and resolution
    pinMode(encoder[k].chanA, INPUT);           // configure GPIO for encoder channel A input
    pinMode(encoder[k].chanB, INPUT);           // configure GPIO for encoder channel B input
    // configure encoder to trigger interrupt with each rising edge on channel A
    attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
  }

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.printf("Error initializing ESP-NOW\n");
    return;
  } else {
    Serial.printf("Successfully initialized ESP-NOW\n");
  }
  esp_now_register_recv_cb(onDataRecv);  // register callback function for received data
  esp_now_register_send_cb(onDataSent);  // register callback function for data transmission

  // Set controller info
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);  // set address of peer
  peerInfo.channel = 0;                               // set peer channel
  peerInfo.encrypt = false;                           // no encryption of data

  // Add controller as ESP-NOW peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.printf("Failed to add peer\n");
    return;
  } else {
    Serial.printf("Successfully added peer %x:%x:%x:%x:%x:%x\n to list", receiverMacAddress[0], receiverMacAddress[1],
                  receiverMacAddress[2], receiverMacAddress[3],
                  receiverMacAddress[4], receiverMacAddress[5]);
  }

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
  float deltaT = 0;                  // time interval
  long pos[] = { 0, 0, 0 };          // current motor positions
  float velEncoder[] = { 0, 0, 0 };  // motor velocity in counts/sec
  float velMotor[] = { 0, 0, 0 };    // motor shaft velocity in rpm
  float posChange[] = { 0, 0, 0 };   // change in position for set speed
  long e[] = { 0, 0, 0 };            // position error
  float ePrev[] = { 0, 0, 0 };       // previous position error
  float dedt[] = { 0, 0, 0 };        // rate of change of position error (de/dt)
  float eIntegral[] = { 0, 0, 0 };   // integral of error
  float u[] = { 0, 0, 0 };           // PID control signal
  int pwm[] = { 0, 0, 0 };           // motor speed(s), represented in bit resolution
  int dir[] = { 1, 1, 1 };           // direction that motor should turn
  int speedV = 0;                    // var to hold speed value


  uint16_t r, g, b, cor;  // RGBC values from TCS34725
  double c;



  // if too many sequential packets have dropped, assume loss of controller, restart as safety measure
  if (commsLossCount > cMaxDroppedPackets) {
    delay(1000);    // okay to block here as nothing else should be happening
    ESP.restart();  // restart ESP32
  }



  unsigned long curTime = micros();                  // capture current time in microseconds
  if (curTime - lastTime > 10000) {                  // wait ~10 ms
    deltaT = ((float)(curTime - lastTime)) / 1.0e6;  // compute actual time interval in seconds
    lastTime = curTime;                              // update start time for next control cycle
    driveData.time = curTime;                        // update transmission time

    uint16_t r, g, b, cor;               // RGBC values from TCS34725
    if (tcsFlag) {                       // if colour sensor initialized
      tcs.getRawData(&r, &g, &b, &cor);  // get raw RGBC values
      c = cor;
#ifdef PRINT_COLOUR

      Serial.printf("R: %f, G: %f, B: %f, C %f, A %d\n", r / c * 100, g / c * 100, b / c * 100, c, servo2Angle);
#endif
    }
  }

  if (c >= 70) {                                                                                                                                                                                         // object detected
    if (r / c * 100 >= cali[0] - tol && r / c * 100 <= cali[0] + tol && g / c * 100 >= cali[1] - tol && g / c * 100 <= cali[1] + tol && b / c * 100 >= cali[2] - tol && b / c * 100 <= cali[2] + tol) {  // object is green gem
      servo1Angle = 90;
      ledcWrite(servo1Channel, degreesToDutyCycle(servo1Angle));  // open
      lastActTime = millis();
    }
  }

  if (inData.dump == 1) {
    //Serial.println("here");
    servo2Angle = 130;
  } else {
    //Serial.println("there");
    servo2Angle = 10;
  }

  ledcWrite(servo2Channel, degreesToDutyCycle(servo2Angle));

  unsigned long curMillis = millis();
  if ((curMillis - lastActTime) > actDelay) {
    lastActTime = curMillis;
    ledcWrite(servo1Channel, degreesToDutyCycle(servo1Angle));
    servo1Angle = 0;
  }
  // store encoder positions to avoid conflicts with ISR updates
  noInterrupts();  // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
    pos[k] = encoder[k].pos;  // read and store current motor position
  }
  interrupts();  // turn interrupts back on

  curTime = micros();
  deltaT = ((float)(curTime - lastTime)) / 1.0e6;  // compute actual time interval in seconds
  lastTime = curTime;                              // update start time for next control cycle
  for (int k = 0; k < cNumMotors; k++) {
    inData.speed = 50;
    velEncoder[k] = ((float)pos[k] - (float)lastEncoder[k]) / deltaT;  // calculate velocity in counts/sec
    lastEncoder[k] = pos[k];                                           // store encoder count for next control cycle
    velMotor[k] = velEncoder[k] / cCountsRev * 60;                     // calculate motor shaft velocity in rpm
    inData.speed = cMaxChange;
    //update target for set direction


    // for Fwd and rev ONLY
    posChange[k] = (float)(inData.dir * inData.speed);  // update with pot input speed

    //for turning
    if (inData.turn == 1 /*&& inData.dir != 0*/) {  //to turn left
      posChange[0] = 0;                             // set pos change to 0 for L motor
      posChange[1] = 1 * inData.speed;              // posChange[1] =  1 * inData.speed;

    } else if (inData.turn == -1 /* && inData.dir != 0*/) {  //to turn right
      posChange[1] = 0;                                      // set pos change to 0 for R motor
      posChange[0] = 1 * inData.speed;
    }
    if (inData.dir == 0 && inData.turn != 0) {  // for turn in place
      posChange[k] = (float)(inData.speed);     // set both to got forward
      if (inData.turn == 1) {                   // to turn left
        posChange[0] *= -1;                     // Left goes backwards

      } else {               // else turning right
        posChange[1] *= -1;  //right goes backwards
      }
    }

    posChange[2] = (float)(inData.speed);

    // changes
    targetF[k] = targetF[k] + posChange[k];  // set new target position


    if (k == 0) {                     // assume differential drive //apperently forword is rev
      target[k] = (long)-targetF[k];  // motor 1 spins one way
    } else if (k == 1) {
      target[k] = (long)targetF[k];  // motor 2 spins in opposite direction
    } else {
      target[k] = (long)targetF[k];  //motor 3 spins too
    }

    // use PID to calculate control signal to motor
    e[k] = target[k] - pos[k];                            // position error
    dedt[k] = ((float)e[k] - ePrev[k]) / deltaT;          // derivative of error
    eIntegral[k] = eIntegral[k] + e[k] * deltaT;          // integral of error (finite difference)
    u[k] = kp * e[k] + kd * dedt[k] + ki * eIntegral[k];  // compute PID-based control signal
    ePrev[k] = e[k];                                      // store error for next control cycle

    // set direction based on computed control signal
    dir[k] = 1;      // default to forward directon
    if (u[k] < 0) {  // if control signal is negative
      dir[k] = -1;   // set direction to reverse
    }

    // set speed based on computed control signal
    u[k] = fabs(u[k]);               // get magnitude of control signal
    if (u[k] > cMaxSpeedInCounts) {  // if control signal will saturate motor
      u[k] = cMaxSpeedInCounts;      // impose upper limit
    }
    //[k] = cMaxSpeedInCounts;                                     // impose upper limit
    pwm[k] = map(u[k], 0, cMaxSpeedInCounts, cMinPWM, cMaxPWM);  // convert control signal to pwm
    if (commsLossCount < cMaxDroppedPackets / 4) {
      setMotor(dir[k], pwm[k], cIN1Chan[k], cIN2Chan[k]);  // update motor speed and direction
    } else {
      setMotor(0, 0, cIN1Chan[k], cIN2Chan[k]);  // stop motor
    }
#ifdef SERIAL_STUDIO

    if (k == 0) {
      printf("/*");  // start of sequence for Serial Studio parsing
    }
    printf("%d,%d,%d,%0.4f", target[k], pos[k], e[k], velMotor[k]);  // target, actual, error, velocity
    if (k < cNumMotors - 1) {
      printf(",");  // data separator for Serial Studio parsing
    }
    if (k == cNumMotors - 1) {
      printf(" ,%d,%d,%d*/\r\n", servo1Angle, servo2Angle);  // end of sequence for Serial Studio parsing
      driveData.data[0] = target[0];
      driveData.data[1] = pos[0];
      driveData.data[2] = e[0];
      driveData.data[3] = (int)velMotor[1];
      driveData.data[4] = target[1];
      driveData.data[5] = pos[1];
      driveData.data[6] = e[1];
      driveData.data[7] = (int)velMotor[1];
      driveData.data[8] = target[2];
      driveData.data[9] = pos[2];
      driveData.data[10] = e[2];
      driveData.data[11] = (int)velMotor[2];
      driveData.data[12] = servo1Angle;
      driveData.data[13] = servo2Angle;
      // driveData.data [14] = servo3Angle;
    }
#endif
  }
  // send data from drive to controller
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&driveData, sizeof(driveData));
  if (result == ESP_OK) {         // if sent successfully
    digitalWrite(cStatusLED, 0);  // turn off communucation status LED
  } else {                        // otherwise
    digitalWrite(cStatusLED, 1);  // turn on communication status LED
  }



  //ledcWrite(servo2Channel, degreesToDutyCycle(servo2Angle));
  //ledcWrite(servo3Channel, degreesToDutyCycle(servo3Angle));
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

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {  // forward
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  } else if (dir == -1) {  // reverse
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  } else {  // stop
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
void ARDUINO_ISR_ATTR encoderISR(void *arg) {
  Encoder *s = static_cast<Encoder *>(arg);  // cast pointer to static structure

  int b = digitalRead(s->chanB);  // read state of channel B
  if (b > 0) {                    // high, leading channel A
    s->pos++;                     // increase position
  } else {                        // low, lagging channel A
    s->pos--;                     // decrease position
  }
}

// callback function for when data is received
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  if (len == 0)  // if empty packet
  {
    return;  // return
  }
  memcpy(&inData, incomingData, sizeof(inData));  // store drive data from controller
#ifdef PRINT_INCOMING
  Serial.printf("%d, %d, %d, %d, %d\n\n", inData.dir, inData.time, inData.speed, inData.turn, inData.dump);
#endif
}

// callback function for when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#ifdef PRINT_SEND_STATUS
  Serial.printf("Last packet send status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
#endif
  if (status != ESP_NOW_SEND_SUCCESS) {
    digitalWrite(cStatusLED, 1);  // turn on communication status LED
    commsLossCount++;             // increase lost packet count
  } else {
    commsLossCount = 0;  // reset communication loss counter
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