# Roboyic-car
<code> 
#include <Servo.h>
#include <AFMotor.h>
#define Echo A1
#define Trig A0
#define motor 10
#define Speed 170
#define spoint 103
char value;
int mode = 0;
int distance;
int Left;
int Right;
int L = 0;
int R = 0;
int L1 = 0;
int R1 = 0;
Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);
void Bluetoothcontrol() {
  while (1) {

    if (Serial.available() > 0) {
      value = Serial.read();
      Serial.println(value);
    }
    if (value == 'F') {
      forward();
    } else if (value == 'B') {
      backward();
    } else if (value == 'R') {
      left();
    } else if (value == 'L') {
      right();
    } else if (value == 'S') {
      Stop();
    }
  }
}
void Obstacle() {
  while (1)
  {
    distance = ultrasonic();
    if (distance <= 12) {
      Stop();
      backward();
      delay(100);
      Stop();
      L = leftsee();
      servo.write(spoint);
      delay(800);
      R = rightsee();
      servo.write(spoint);
      if (L < R) {
        right();
        delay(500);
        Stop();
        delay(200);
      } else if (L > R) {
        left();
        delay(500);
        Stop();
        delay(200);
      }
    } else {
      forward();
    }
  }
}
int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  long cm = t / 29 / 2;
  return cm;
}
void forward() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}
void backward() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}
void right() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}
void left() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}
void Stop() {
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}
int rightsee() {
  servo.write(20);
  delay(800);
  Left = ultrasonic();
  return Left;
}
int leftsee() {
  servo.write(180);
  delay(800);
  Right = ultrasonic();
  return Right;
}
void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  servo.attach(motor);
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
}
void loop() {
  while (Serial.available()) {

    if (Serial.available() > 0) {
      char input = Serial.read();
      Serial.println(input);
      if (input =='O'&&mode==0) {
        Serial.println("Obstacle mode");
        mode = 1;
        Obstacle();
      }

     else if (input =='B'&&mode==0) {
        Serial.println("Bluetooth mode");
        mode = 2;
        Bluetoothcontrol();
      }
    }
  }
}
