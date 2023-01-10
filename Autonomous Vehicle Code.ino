/*
 * Project: Autonomous Vehicle Project Code
 * Description: The code will be the controller for the vehicle so that the vehicle can 
 * drive autonomously around a track using line-following from data gathered by a camera.
 * The car will be powered by a supercapacitors based power supply and both motors used in
 * the vehicle will be treated as servo motors. 
 * Date Created: January 7th, 2023
 * Last Edited: January 8th, 2023
 * Version: 1
 * Authors: Kevin Lin and Jason Su
 */

#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

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

void getCurrent();

void setup() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  Serial.begin(115200);
  backMotor.setPeriodHertz(50);      // Standard 50hz servo
	frontMotor.setPeriodHertz(50);      // Standard 50hz servo
  backMotor.attach(backMotorPin, minUs, maxUs);
  frontMotor.attach(frontMotorPin, minUs, maxUs);

  SerialBT.begin("Leopard 2 Bluetooth Module");  //Bluetooth device name
  // Serial.println("The device started, now you can pair it with bluetooth!");

  Wire.begin();

  while (!huskylens.begin(Wire)) {
    String result = String() + F("Begin failed!\n") + F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)\n") + 
                    F("2.Please recheck the connection.\n");
    for(int i = 0; i < result.length(); i++) {
      SerialBT.write(result.charAt(i));
    }
    delay(100);
  }

  if (!ina219.begin()) {
    String result = String() + F("Failed to find INA219 chip");
    for(int i = 0; i < result.length(); i++) {
      SerialBT.write(result.charAt(i));
    }
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  String result = String() + F("Measuring voltage and current with INA219 ...\n");
  for(int i = 0; i < result.length(); i++) {
    SerialBT.write(result.charAt(i));
  }
}

void loop() {
  int xOrigin = 0;
  int yOrigin = 0;
  int xTarget = 0;
  int yTarget = 0;
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
        xOrigin = result.xOrigin;
        yOrigin = result.yOrigin;
        xTarget = result.xTarget;
        yTarget = result.yTarget;
        // getHuskyResult(result)
        stringResult = String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID;
        Serial.println(stringResult);
      } else {
        Serial.println("Object unknown!");
      }
    }
  }

  // for (int i = 0; i < stringResult.length(); i++){
  //   SerialBT.write(stringResult.charAt(i));
  // }
  // SerialBT.write('\n');

  double maxYValue = 240.0;
  double maxSpeed = 120.0;
  double minSpeed = 50.0;
  int speed = 0;
  Serial.println("yOrigin: " + yOrigin);
  Serial.println("yTarget: " + yTarget);
  frontMotor.write(90);
  if ((yOrigin - yTarget) > 0) {
    speed = (yOrigin-yTarget)/maxYValue*(maxSpeed-minSpeed) + minSpeed; // Max value of 240
    speed = round((speed+5)/10) * 10; //Rounding to nearest 10's place (THIS SHOULD WORK BUT DOUBLE CHECK)
    Serial.println("Speed: " + speed);
    for(int pos = 50; pos < 120; pos += 10) { // Goes from 50 to speed ideally
      backMotor.write(pos);
      delay(500);
    }
  }
  // if ((xOrigin - xTarget) > 20) {
  //   //Go left //Max value of 320
  // } else if ((xOrigin - xTarget) < 20) {
  //   //Go right //Min value of -320
  // } else {
  //   frontMotor.write(90);
  // }
  getCurrent();

  delay(300);
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

  String result = String() + F("Bus Voltage: ") + busvoltage + F(" V\n") + 
                  F("Shunt Voltage: ") + shuntvoltage + F(" mV\n") +
                  F("Load Voltage: ") + loadvoltage + F(" V\n") + 
                  F("Current: ") + current_mA + F(" mA\n") +
                  F("Power: ") + power_mW + F(" mW\n");

  for(int i = 0; i < result.length(); i++) {
    SerialBT.write(result.charAt(i));
  }
}

// void loop() {
//   for (pos = 50; pos < 120; pos += 10) {  // sweep from 0 degrees to 180 degrees
//     // in steps of 1 degree
//     backMotor.write(pos);
//     delay(100);  // waits 20ms for the servo to reach the position
//   }
//   delay(2000);
//   for (pos = 120; pos > 50; pos -= 10) {  // sweep from 180 degrees to 0 degrees
//     backMotor.write(pos);
//     delay(100);
//   }
//   backMotor.write(0);
//   delay(2000);
//   for (pos = 20; pos < 160; pos += 10) {  // sweep from 0 degrees to 180 degrees
//     // in steps of 1 degree
//     frontMotor.write(pos);
//     delay(100);  // waits 20ms for the servo to reach the position
//   }
//   for (pos = 160; pos > 20; pos -= 10) {  // sweep from 180 degrees to 0 degrees
//     frontMotor.write(pos);
//     delay(100);
//   }
//   backMotor.detach();
//   frontMotor.detach();

//   delay(5000);
// }
