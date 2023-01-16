#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#include <Arduino.h>
#include "SPI.h"
/*
 * Project: Autonomous Vehicle Project Code
 * Description: The code will be the controller for the vehicle so that the vehicle can 
 * drive autonomously around a track using line-following from data gathered by a camera.
 * The car will be powered by a supercapacitors based power supply and both motors used in
 * the vehicle will be treated as servo motors. 
 * Date Created: January 7th, 2023
 * Last Edited: January 15th, 2023
 * Version: 1
 * Authors: Kevin Lin and Jason Su
 */

#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <AutoPID.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
// create two servo objects
Servo backMotor;
Servo frontMotor;

// These are all GPIO pins on the ESP32
// Recommended pins include 2,4,12-19,21-23,25-27,32-33
int backMotorPin = 33;
int frontMotorPin = 32;
// Published values for SG90 servos; adjust if needed
int minUs = 500;
int maxUs = 2400;

ESP32PWM pwm;

HUSKYLENS huskylens;
//HUSKYLENS green line >> SDA; blue line >> SCL

Adafruit_INA219 ina219;
bool startPressed;
String startMessage = "Please send the command 'start' to start the vehicle";

#define OUTPUT_MIN -50
#define OUTPUT_MAX 50

#define KP 0.5
#define KI 0.0
#define KD 0
#define GAIN 15

double Setpoint, Input, Output;
AutoPID HuskyPID(&Input, &Setpoint, &Output, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

double slope, oldslope1, oldslope2, blended_slope, x_offset;
double xOrigin, yOrigin, xTarget, yTarget;
int steer, motor;

void getCurrent();

void setup() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  Serial.begin(115200);
  backMotor.setPeriodHertz(50);   // Standard 50hz servo
  frontMotor.setPeriodHertz(50);  // Standard 50hz servo
  backMotor.attach(backMotorPin, minUs, maxUs);
  frontMotor.attach(frontMotorPin, minUs, maxUs);
  Setpoint = 0.0;
  HuskyPID.setTimeStep(20);
  startPressed = false;
  SerialBT.begin("Leopard 2 Bluetooth Module");  //Bluetooth device name
  Wire.begin();

  while (!huskylens.begin(Wire)) {
    String result = String() + F("Begin failed!\n") + F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)\n") + F("2.Please recheck the connection.\n");
    for (int i = 0; i < result.length(); i++) {
      SerialBT.write(result.charAt(i));
    }
    delay(100);
  }

  if (!ina219.begin()) {
    String result = String() + F("Failed to find INA219 chip");
    for (int i = 0; i < result.length(); i++) {
      SerialBT.write(result.charAt(i));
    }
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  String result = String() + F("Measuring voltage and current with INA219 ...\n");
  for (int i = 0; i < result.length(); i++) {
    SerialBT.write(result.charAt(i));
  }
  for (int i = 0; i < 180; i += 20) {
    backMotor.write(i);
    delay(20);
  }
  for (int i = 180; i > 0; i -= 20) {
    backMotor.write(i);
    delay(20);
  }
  frontMotor.write(100);
}

void Huskycontrol() {
  String stringResult = "";
  if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  else if (!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  else if (!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
  else {
    Serial.println(F("###########"));
    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      if (result.command == COMMAND_RETURN_BLOCK) {
        Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
      } else if (result.command == COMMAND_RETURN_ARROW) {
        // time2 = millis();
        xOrigin = result.xOrigin;
        yOrigin = result.yOrigin;
        xTarget = result.xTarget;
        yTarget = result.yTarget;
        if ((xTarget - xOrigin) == 0) slope = 0;
        else slope = 1 / ((yTarget - yOrigin) / (xTarget - xOrigin));
        x_offset = (150 - xOrigin) / 15;                                        // if the car is on one side or the other of the line, turn that offset into the equivalent of a slope
        blended_slope = x_offset + 10 * ((slope + oldslope1 + oldslope2) / 3);  // X offset plus moving average of past three slope reads
        Input = blended_slope;                                                  // take a moving average of the past three reading
        oldslope2 = oldslope1;                                                  // push the older readings down the stack
        oldslope1 = slope;
        HuskyPID.run();  //call every loop, updates automatically at certain time interval
        String temp = String() + Input + F("  ") + Output + F("\n");
        for(int i = 0; i < temp.length(); i++){
          SerialBT.write(temp.charAt(i));
        }
        steer = -Output * GAIN + 100;
        steer = constrain(steer, 45, 180);
        // motor = constrain(motor, 60, 120);
        frontMotor.write(steer);  // send values to output
        backMotor.write(60);
        // lasttime2 = time2;
        // stringResult = String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID;
        // Serial.println(stringResult);
      } else {
        Serial.println("Object unknown!");
      }
    }
  }
  delay(10);
}

void loop() {
  if (startPressed == true) {
    // time = millis();
    // if (time > lasttime + 1000) {
    //   lasttime = time;
    // }
    Huskycontrol();

    // for (int i = 0; i < stringResult.length(); i++) {
    //   SerialBT.write(stringResult.charAt(i));
    // }
    // SerialBT.write('\n');

    // double maxYValue = 240.0;
    // double maxSpeed = 120.0;
    // double minSpeed = 50.0;
    // double maxXValue = 320.0;
    // double maxAngle = 45.0;
    // double minAngle = 0.0;
    // double middlePoint = 160.0;
    // int speed = 0;
    // int angle = 0;

    // if ((yOrigin - yTarget) > 0) {
    //   // speed = (yOrigin - yTarget) / maxYValue * (maxSpeed - minSpeed) + minSpeed;  // Max value of 240
    //   // String speedString = String() + F("Speed: ") + speed + "\n";
    //   // for (int i = 0; i < speedString.length(); i++) {
    //   //   SerialBT.write(speedString.charAt(i));
    //   // }
    //   backMotor.write(100);
    // }
    // String angleString;
    // if ((xOrigin - xTarget) > 20) {
    //   //Go left //Max value of 320
    //   angle = map((middlePoint - xTarget), 0, 200, 100, 180);
    //   // angle = (xOrigin - xTarget) / maxXValue * (maxAngle - minAngle) + middleAngle;
    //   angleString = String() + F("Angle: ") + angle + F("\n");
    //   frontMotor.write(angle);
    // } else if ((xOrigin - xTarget) < -20) {
    //   //Go right //Min value of -320
    //   angle = map((middlePoint - xTarget), -170, -1, 10, 90);
    //   // angle = middleAngle - (xTarget - xOrigin) / maxXValue * (maxAngle - minAngle);
    //   angleString = String() + F("Angle: ") + angle + F("\n");
    //   frontMotor.write(angle);
    // } else {
    //   angleString = String() + F("Angle: ") + middleAngle + F("\n");
    //   frontMotor.write(90);
    // }
    // for (int i = 0; i < angleString.length(); i++) {
    //   SerialBT.write(angleString.charAt(i));
    // }

    // getCurrent();
  } else {
    for (int i = 0; i < startMessage.length(); i++) {
      SerialBT.write(startMessage.charAt(i));
    }
    SerialBT.write('\n');
    delay(2000);
    if (SerialBT.available()) {
      String read = SerialBT.readString();
      read.toLowerCase();
      if (read.compareTo("start")) {
        startPressed = 1;
      }
    }
  }
}

void getCurrent() {
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  String result = String() + F("Bus Voltage: ") + busvoltage + F(" V\n") + F("Shunt Voltage: ") + shuntvoltage + F(" mV\n") + F("Load Voltage: ") + loadvoltage + F(" V\n") + F("Current: ") + current_mA + F(" mA\n") + F("Power: ") + power_mW + F(" mW\n");

  for (int i = 0; i < result.length(); i++) {
    SerialBT.write(result.charAt(i));
  }
}
