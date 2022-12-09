#include <Wire.h>
#include <Zumo32U4.h>

// For this code to work, the library proximitysensors must be edited, 
// such the prepareRead() func does not turn emittersOff.

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4OLED display;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4IMU imu;

#include "TurnSensor.h"

unsigned int lineSensorValues[5];
int brightnessLevels[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
char TrashDistance;
int currentAngle = 0;

void setup() {
  proxSensors.initFrontSensor();
  proxSensors.setBrightnessLevels(brightnessLevels, 20);
  lineSensors.initFiveSensors();
  display.clear();
   
  // Play a little welcome song
  buzzer.play(">g32>>c32");

  // Wait for button A to be pressed and released.
  display.print(F("Press A"));
  display.gotoXY(0, 1);
  display.print(F("to calib"));
  buttonA.waitForButton();

  calibrateSensors();
  buttonA.waitForButton();

  // Play music and wait for it to finish before we start driving.
  display.clear();
  display.print(F("Go!"));
  buzzer.play("L16 cdegreg4");
  while(buzzer.isPlaying());

  findLine();
  moveOntoLine(clockwise=true);
  followLine();
  turnSensorSetup(); 
}

void loop() {
  turnSensorReset(); 
  TrashDistance = scanTrash();
  sortTrash();
}

void calibrateSensors() {
  delay(1000);
  // Rotates in place to sweep sensors over line
  for(uint16_t i = 0; i < 120; i++) {
    if(i > 30 && i <= 90) motors.setSpeeds(-150, 150);
    else motors.setSpeeds(150, -150);
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

void findLine() {
  motors.setSpeeds(300, 300);
  lineSensors.readCalibrated(lineSensorValues);
  while(!detectLine(100)) lineSensors.readCalibrated(lineSensorValues);
  motors.setSpeeds(0, 0);
}

bool detectLine(const int lineValue) {
  return lineSensorValues[0] < lineValue || \
         lineSensorValues[4] < lineValue || \
         lineSensorValues[2] < lineValue;
}

void moveOntoLine(const bool clockwise) {
  const int baseSpeed = 100;
  // Move a bit past line
  motors.setSpeeds(baseSpeed, baseSpeed);
  delay(550);

  // Turn until middle sensor hits the line.
  if (clockwise) motors.setSpeeds(baseSpeed, -baseSpeed);
  else motors.setSpeeds(-baseSpeed, baseSpeed);
  while(lineSensorValues[2] >= 150) lineSensors.readCalibrated(lineSensorValues);
  motors.setSpeeds(0, 0); 
}

void followLine() {
  const int setpoint = 2000, baseSpeed = 100;
  float kp = 0.8, kd = 1;
  int sensorInput, error, lastError = 0, errorRate, regulatedSpeed;
  
  lineSensors.readCalibrated(lineSensorValues);
  while(!(lineSensorValues[0] < 300 && lineSensorValues[4] < 300 && lineSensorValues[2] > 200)) {
    sensorInput = lineSensors.readLine(lineSensorValues);
    error = sensorInput - setpoint;
    errorRate = error - lastError;
    regulatedSpeed = baseSpeed + (error*kp + errorRate*kd);
    motors.setSpeeds(baseSpeed, regulatedSpeed);
    lastError = error;
  }
  motors.setSpeeds(0, 0);
}

char scanTrash() {
  int cLeftSensor;
  int cRightSensor;

  lineSensors.emittersOn();
  while(true) {
    proxSensors.read();
    cLeftSensor = proxSensors.countsFrontWithLeftLeds();
    cRightSensor = proxSensors.countsFrontWithRightLeds();
    if(cLeftSensor == cRightSensor && cRightSensor == 20) {
      delay(600);
      lineSensors.emittersOff();
      return 'c';
    }
    if((cRightSensor == cLeftSensor && cRightSensor > 17 && cRightSensor < 20)) {
      lineSensors.emittersOff();
      return 'f';
    }
  }
}

void sortTrash() {
  switch(TrashDistance) {
    case 'f': sortTrashFar(); break;
    case 'c': sortTrashClose(); break;
    default: break;
  }
}

void sortTrashFar(){
  const int baseSpeed = 300, baseDuration = 200;
  // move away from start line
  motors.setSpeeds(baseSpeed, baseSpeed);
  delay(baseDuration);

  // move forward until a new line is found
  findLine();
  buzzer.play("L16 cdegreg4");
  TrashDistance = 'n';

  // return to start line
  motors.setSpeeds(-baseSpeed, -baseSpeed);
  delay(baseDuration);
  lineSensors.readCalibrated(lineSensorValues);
  while(!detectLine(100)) lineSensors.readCalibrated(lineSensorValues);
  delay(baseDuration);
  followLine();
}

void sortTrashClose() {

  // go around trash and push
  rotateToTarget(-30);
  motors.setSpeeds(300, 300);
  delay(400);
  motors.setSpeeds(0, 0);
  rotateToTarget(90);  
  lineSensors.readCalibrated(lineSensorValues);
  findLine();
  buzzer.play("L16 cdegreg4");
  
  // go back
  motors.setSpeeds(100, 100);
  delay(120);
  motors.setSpeeds(0, 0);
  turnSensorReset();
  rotateToTarget(90);
  motors.setSpeeds(200, 200);
  delay(900);
  motors.setSpeeds(0, 0);
  rotateToTarget(169);

  findLine();
  moveOntoLine(clockwise=false);
  followLine();
  TrashDistance = 'n';
}

void rotateToTarget(int target) {  // span {-180 -- 0 -- 180}
  int currentAngle = getAngle(); 

  while(-target != currentAngle && target != currentAngle)) {
    turnSensorUpdate();
    currentAngle = getAngle();

    if(target != currentAngle && target < (currentAngle)) motors.setSpeeds(100, -100);
    else if(target != currentAngle && target > (currentAngle)) motors.setSpeeds(-100, 100);

    display.clear();
    display.print(currentAngle);
  }
  motors.setSpeeds(0,0);
}

int getAngle() {
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}

void showLineSensorValues(const int a, const int b, const int c) {
  /* Reads linesensor values. Values span from 0-1000. Higher value = more color contrast. */
  lineSensors.readCalibrated(lineSensorValues);
  display.clear();
  display.gotoXY(0, 0); display.println(lineSensorValues[a]);
  display.gotoXY(4,0); display.println(lineSensorValues[b]);
  display.gotoXY(0,1); display.println(lineSensorValues[c]);
}
