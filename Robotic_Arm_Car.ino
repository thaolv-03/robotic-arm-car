#include <Ps3Controller.h>
#include <ESP32Servo.h>

#define SERVO_FORWARD_STEP_ANGLE 1
#define SERVO_BACKWARD_STEP_ANGLE -1

struct ServoPins {
  Servo servo;
  int servoPin;
  String servoName;
  int initialPosition;  
};

std::vector<ServoPins> servoPins = {
  { Servo(), 27 , "Base", 90},
  { Servo(), 26 , "Shoulder", 0},
  { Servo(), 25 , "Elbow", 0},
  { Servo(), 33 , "Gripper", 90},
};

bool gripperSwitch = false;

// Right motor
int enableRightMotor=23; 
int rightMotorPin1=19;
int rightMotorPin2=18;
// Left motor
int enableLeftMotor=22;
int leftMotorPin1=17;
int leftMotorPin2=16;

#define MAX_MOTOR_SPEED 255  // 255 is maximum speed

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int PWMSpeedChannel = 4;

void writeServoValues(int servoIndex, int servoMoveStepSize, bool servoStepSizeIsActualServoPosition = false) {
  int servoPosition;

  if (servoStepSizeIsActualServoPosition) {
    servoPosition = servoMoveStepSize; 
  }
  else {
    servoPosition = servoPins[servoIndex].servo.read();
    servoPosition = servoPosition + servoMoveStepSize;
  }

  if (servoPosition > 180 || servoPosition < 0) {
    return;
  }

  servoPins[servoIndex].servo.write(servoPosition);
  
}

void notify() {
  int ry = (Ps3.data.analog.stick.ry);  // Shoulder =>  Right stick  - y axis
  int ly = (Ps3.data.analog.stick.ly);  // Elbow    =>  Left stick  - y axis
  int square = (Ps3.data.analog.button.square);  // Đóng gripper
  int circle = (Ps3.data.analog.button.circle);  // Mở gripper
  int triangle = (Ps3.data.analog.button.triangle); // Xoay base qua trái
  int cross = (Ps3.data.analog.button.cross); // Xoay base qua phải

  if (ry > 50) {
    writeServoValues(1, 3.5);  
  } else if (ry < -50) {
    writeServoValues(1, SERVO_BACKWARD_STEP_ANGLE);  
  }

  if (ly > 50) { // 
    writeServoValues(2, SERVO_BACKWARD_STEP_ANGLE);  
  } else if (ly < -50) {
    writeServoValues(2, 3.5);  
  }

  if (square) { // Đóng gripper
    writeServoValues(3, 3.5);  
  } else if (circle) { // Mở gripper
    writeServoValues(3, SERVO_BACKWARD_STEP_ANGLE);
  }

  if (triangle) { // Base -> Left
    writeServoValues(0, SERVO_BACKWARD_STEP_ANGLE);    
  } else if (cross) { // Base -> Right
    writeServoValues(0, 3.5); 
  }

  if (Ps3.data.button.up) { 
    rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED); // Tiến xe
  }
  else if (Ps3.data.button.down) {
    rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED); // Lùi xe
  }
  else if (Ps3.data.button.right) {
    rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED); // Rẽ phải
  }
  else if (Ps3.data.button.left) {
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED); // Rẽ trái
  }
  else {
    rotateMotor(0, 0); // Dừng xe
  } 
    
  delay(10);
  
}

void onConnect() {
  Serial.println("Connected!.");
}

void onDisConnect() {
  Serial.println("Disconnected!.");    
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  } else {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }
  
  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  } else {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }  
}

void setUpPinModes() {
  for (int i = 0; i < servoPins.size(); i++) {
    servoPins[i].servo.attach(servoPins[i].servoPin);
    servoPins[i].servo.write(servoPins[i].initialPosition);    
  }

  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  
  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);

  // Thiết lập PWM cho động cơ
  ledcSetup(PWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, PWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, PWMSpeedChannel);  
  ledcWrite(PWMSpeedChannel, MAX_MOTOR_SPEED);
  
  rotateMotor(0, 0);  
}


void setup() {
  setUpPinModes();
  Serial.begin(115200);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisConnect);
  Ps3.begin("c0:49:ef:cb:73:4a");
  Serial.println("Ready.");
}

void loop() {
}
