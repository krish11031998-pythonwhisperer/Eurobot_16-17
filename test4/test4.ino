#include <Servo.h>

Servo myservo;  // create servo object to control a servo
Servo myservo1;  // create servo object to control a servo
Servo myservo2;
Servo myservo3;
Servo myservo4;
// a maximum of eight servo objects can be created

int pos = 50;    // variable to store the servo position
int pos1 = 135;    // variable to store the servo position
int pos2 = 100;
int pos3 = 100;
int pos4 = 135;

//#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_PWMServoDriver.h"
#include <Wire.h>                                              // Use the 'Wire' library for I2C.
//#include <Servo.h>
//#include <Timer.h>
#define MD25ADDRESS         0x58                               // Address of the MD25 on the I2C bus.
#define SPEED               0x00                               // Byte to send speed to both motors.
#define TURN                0x01                               // Byte to send speed for turn amount.
#define ENCODER1            0x02                               // Byte to read motor encoder 1.
#define ENCODER2            0x06                               // Byte to read motor encoder 2.
#define ACCELERATION        0x0E                               // Byte to define motor acceleration.
#define CMD                 0x10                               // Byte to reset encoder values.
#define MODE                0x0F                               // Byte to change between control MODES.
//Timer t ;
//Adafruit_MotorShield AFMS = Adafruit_MotorShield();
//Adafruit_DCMotor *leftcarpet = AFMS.getMotor(1);
//Adafruit_DCMotor *rightcarpet = AFMS.getMotor(2);
const int pullrope = 4;
const int startpoint = 3;
int thresh = 17;
const int trigpin = 12;
const int echo = 13;
const int echo1 = 10;
const int trigpin1 = 11;
long pulse = 0;
long cm = 0;
int speed = 0;
int acceleration = 0;
int turnspeed = 0;
float wd1 = 0;
float wd2 = 0;
long duration = 0;
int distance = 0;
long durationback = 0;
int distanceback = 0;
float tempEncoder1 = 0;
float tempEncoder2 = 0;

void setup() {
  myservo.attach(2);  // attaches the servo on pin 2 on arduino
  myservo1.attach(5);  // attaches the servo on pin 5 on arduino
  myservo2.attach(6);  // attaches the servo on pin 6 on arduino
  myservo3.attach(7);  // attaches the servo on pin 7 on arduino
  myservo4.attach(9);  // attaches the servo on pin 9 on arduino

  //AFMS.begin();
  Wire.begin();
  Serial.begin(9600);
  delay(100);
  Wire.beginTransmission(MD25ADDRESS);                         // Open I2C bus communication line with the MD25.
  Wire.write(MODE);                                            // Write to the MODE register.
  Wire.write(3);                                               // Set mode to 3. Single speed value, turn value. Signed speed value, -127<x<128.
  Wire.endTransmission();                                      // Send commands and close the I2C bus.
  pinMode(startpoint, INPUT);
  pinMode( , INPUT);
  encodeReset();                                               // Calls a function that resets the encoder values to 0.
  pinMode(trigpin, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(trigpin1, OUTPUT);
  pinMode(echo1, INPUT);

  Wire.beginTransmission(MD25ADDRESS);                         // Open I2C bus communication line with the MD25.
  Wire.write(CMD);                                             // Write to the command register.
  Wire.write(0x32);                                            // Disable motor timeout.
  Wire.endTransmission();
  // t.after(90000,autostop);
  myservo.write(pos);
  myservo1.write(pos1);
  myservo2.write(pos2);
  myservo3.write(pos3);
  myservo4.write(pos4);


}

void encodeReset() {                                                // Create function to reset the encoder values to 0.
  Wire.beginTransmission(MD25ADDRESS);                              // Open I2C bus communication line with the MD25.
  Wire.write(CMD);                                                  // Write to the command register.
  Wire.write(0x20);                                                 // Reset the encoder values to 0.
  Wire.endTransmission();                                           // Send commands and close the I2C bus.
  delay(50);                                                       // Delay to allow registers to reset.
}

void autostop() {
  while (true) {
        motorStop();
  }

}

float encoder1() {                                                  // Creat function to read value of encoder 1.
  Wire.beginTransmission(MD25ADDRESS);                              // Open I2C bus communication line with the MD25.
  Wire.write(ENCODER1);                                             // Send byte to get a reading from encoder 1.
  Wire.endTransmission();                                           // Send commands and close the I2C bus.

  Wire.requestFrom(MD25ADDRESS, 4);                          // Request 4 bytes from MD25.
  while (Wire.available() < 4);                              // Wait for 4 bytes to arrive.
  long poss1 = Wire.read();                                  // First byte for encoder 1, HH.
  poss1 <<= 8;                                               // Bitshift the first byte.
  poss1 += Wire.read();                                      // Second byte for encoder 1, HL.
  poss1 <<= 8;
  poss1 += Wire.read();                                      // Third byte for encoder 1, LH
  poss1 <<= 8;                                               // Bitshift for the third byte.
  poss1  += Wire.read();                                     // Fourth byte for encoder 1, LLalue.
  delay(5);                                                  // Wait for everything to make sure everything is sent.
  return (poss1 * 0.0785);                                     // Convert encoder value to cm.
}


float encoder2() {                                                  // Creat function to read value of encoder 1.
  Wire.beginTransmission(MD25ADDRESS);                              // Open I2C bus communication line with the MD25.
  Wire.write(ENCODER2);                                             // Send byte to get a reading from encoder 2.
  Wire.endTransmission();                                           // Send commands and close the I2C bus.

  Wire.requestFrom(MD25ADDRESS, 4);                          // Request 4 bytes from MD25.
  while (Wire.available() < 4);                              // Wait for 4 bytes to arrive.
  long poss2 = Wire.read();                                  // First byte for encoder 1, HH.
  poss2 <<= 8;                                               // Bitshift the first byte.
  poss2 += Wire.read();                                      // Second byte for encoder 1, HL.
  poss2 <<= 8;                                               // Bitshift for the second byte.
  poss2 += Wire.read();                                      // Third byte for encoder 1, LH
  poss2 <<= 8;                                               // Bitshift for the third byte.
  poss2  += Wire.read();                                     // Fourth byte for encoder 1, LLalue.
  delay(5);                                                  // Wait for everything to make sure everything is sent.
  return (poss2 * 0.0785);                                     // Convert encoder value to cm.
}


bool isBlocked() {
  int distancefwd = ping();
  int distanceback = ping1();
  if (distancefwd < thresh || distanceback < thresh) {
    return true;
  }
  else {
    return false;
  }
}

void forward(int speed, int acceleration, float wd1) {
  wd1 = abs(wd1);
  while ((abs(encoder1()) <= wd1) && (abs(encoder2()) <= wd1) ) {
    Wire.beginTransmission(MD25ADDRESS);                         // Open I2C bus communication line with the MD25.
    Wire.write(ACCELERATION);                                    // Set the acceleration value.
    Wire.write(acceleration);                                              // Maximum acceleration.
    Wire.endTransmission();


    Wire.beginTransmission(MD25ADDRESS);                                   // Open I2C bus communication line with the MD25.
    Wire.write(SPEED);                                                     // Set the speed value.
    Wire.write(speed);                                                         // Speed of 'x'.
    Wire.endTransmission();                                                // Send commands and close the I2C bus.
    while (isBlocked()) {
      motorStop();
    }
  }
  motorStop();
}

void rightTurn(int turnspeed, int acceleration) {
  while ((abs(encoder1()) <= -15) && (abs(encoder2()) <= 15)) {

    Wire.beginTransmission(MD25ADDRESS);                         // Open I2C bus communication line with the MD25.
    Wire.write(ACCELERATION);                                    // Set the acceleration value.
    Wire.write(acceleration);                                              // Maximum acceleration.
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                            // Open I2C bus communication line with the MD25.
    Wire.write(TURN);                                               // Set the turn value.
    Wire.write(turnspeed);                                                  // Turn of 't'.
    Wire.endTransmission();                                         // Send commands and close the I2C bus.
    while (isBlocked()) {
      motorStop();
    }
  }
  motorStop();
}

void leftTurn(int turnspeed, int acceleration) {
  while ((abs(encoder1()) <= 15) && (abs(encoder2()) <= -15)) {

    Wire.beginTransmission(MD25ADDRESS);                         // Open I2C bus communication line with the MD25.
    Wire.write(ACCELERATION);                                    // Set the acceleration value.
    Wire.write(acceleration);                                              // Maximum acceleration.
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                            // Open I2C bus communication line with the MD25.
    Wire.write(TURN);                                               // Set the turn value.
    Wire.write(turnspeed);                                                  // Turn of 't'.
    Wire.endTransmission();                                         // Send commands and close the I2C bus.
    while (isBlocked()) {
      motorStop();
    }
  }
  motorStop();
}
void turn180(int turnspeed, int acceleration) {
  while ((abs(encoder1()) <= 30) && (abs(encoder2()) <= -30)) {

    Wire.beginTransmission(MD25ADDRESS);                         // Open I2C bus communication line with the MD25.
    Wire.write(ACCELERATION);                                    // Set the acceleration value.
    Wire.write(acceleration);                                              // Maximum acceleration.
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                            // Open I2C bus communication line with the MD25.
    Wire.write(TURN);                                               // Set the turn value.
    Wire.write(turnspeed);                                                  // Turn of 't'.
    Wire.endTransmission();                                         // Send commands and close the I2C bus.
    while (isBlocked()) {
      motorStop();
    }
  }
  motorStop();
}
void driv(int speed, int turnspeed, float wd1, float wd2, int acceleration) {
  wd1 = abs(wd1);                                                     // Take the magnitude of wheel distance 1.
  wd2 = abs(wd2);                                                     // Take the magnitude of wheel distance 2.


  while ((abs(encoder1()) <= wd1) && (abs(encoder2()) <= wd2)) {           // While statement that waits for the encoder values to be equal to the required distances.


    Wire.beginTransmission(MD25ADDRESS);                         // Open I2C bus communication line with the MD25.
    Wire.write(ACCELERATION);                                    // Set the acceleration value.
    Wire.write(acceleration);                                              // Maximum acceleration.
    Wire.endTransmission();                                      // Send commands and close the I2C bus.

    Wire.beginTransmission(MD25ADDRESS);                                   // Open I2C bus communication line with the MD25.
    Wire.write(SPEED);                                                     // Set the speed value.
    Wire.write(speed);                                                         // Speed of 'x'.
    Wire.endTransmission();                                                // Send commands and close the I2C bus.

    Wire.beginTransmission(MD25ADDRESS);                            // Open I2C bus communication line with the MD25.
    Wire.write(TURN);                                               // Set the turn value.
    Wire.write(turnspeed);                                                  // Turn of 't'.
    Wire.endTransmission();
  }
  motorStop();
  encodeReset();
}
void drive(int speed, int turnspeed, float wd1, float wd2, int acceleration) {
  wd1 = abs(wd1);                                                     // Take the magnitude of wheel distance 1.
  wd2 = abs(wd2);                                                     // Take the magnitude of wheel distance 2.
  tempEncoder1 = abs(tempEncoder1);
  tempEncoder2 = abs(tempEncoder2);

  while ((abs(encoder1()) <= wd1) && (abs(encoder2()) <= wd2)) {          // While statement that waits for the encoder values to be equal to the required distances.


    Wire.beginTransmission(MD25ADDRESS);                         // Open I2C bus communication line with the MD25.
    Wire.write(ACCELERATION);                                    // Set the acceleration value.
    Wire.write(acceleration);                                              // Maximum acceleration.
    Wire.endTransmission();                                      // Send commands and close the I2C bus.

    Wire.beginTransmission(MD25ADDRESS);                                   // Open I2C bus communication line with the MD25.
    Wire.write(SPEED);                                                     // Set the speed value.
    Wire.write(speed);                                                         // Speed of 'x'.
    Wire.endTransmission();                                                // Send commands and close the I2C bus.

    Wire.beginTransmission(MD25ADDRESS);                            // Open I2C bus communication line with the MD25.
    Wire.write(TURN);                                               // Set the turn value.
    Wire.write(turnspeed);                                                  // Turn of 't'.
    Wire.endTransmission();
    while (isBlocked()) {
      motorStop();
    }
  }
  motorStop();
  delay(250);
  tempEncoder1 = (abs(encoder1()) - wd1);
  Serial.println("WD1");
  Serial.println(wd1);
  Serial.println("WD2");
  Serial.println(wd2);
  tempEncoder2 = (abs(encoder2()) - wd2);
  Serial.println("TEMPENCODER1");
  Serial.println(tempEncoder1);
  Serial.println("TEMPENCODER2");
  Serial.println(tempEncoder2);
  encodeReset();
  driv(speed, turnspeed, tempEncoder1, tempEncoder2, 1);
}

void motorStop() {                                          // Function to stop motors

  Wire.beginTransmission(MD25ADDRESS);                      // Sets the acceleration to register 10 (0.65s)
  Wire.write(ACCELERATION);
  Wire.write(7);
  Wire.endTransmission();
  Wire.beginTransmission(MD25ADDRESS);                         // Open communications with the MD25 via I2C.
  Wire.write(SPEED);                                           // Write the next value to the SPEED1 register.
  Wire.write(0);                                               // Set average speed to 0.
  Wire.endTransmission();                                      // Send commands and free I2C line.
  Wire.beginTransmission(MD25ADDRESS);                         // Open communication with the MD25 via I2C.
  Wire.write(TURN);                                            // Write the next value to the TURN/SPEED2 register.
  Wire.write(0);                                               // Set turn factor to 0. Robot should now stop.
  Wire.endTransmission();                                      // Send commands and free I2C line.
  delay(60);                                                   // Delay for a twentieth of a second.
}

int ping()
{
  digitalWrite(trigpin, LOW);
  delay(2);
  digitalWrite(trigpin, HIGH);
  delay(10);
  digitalWrite(trigpin, LOW);
  //Used to read in the pulse that is being sent by the MaxSonar device. //Pulse Width


  duration = pulseIn(echo, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}
int ping1()
{
  digitalWrite(trigpin1, LOW);
  delay(2);
  digitalWrite(trigpin1, HIGH);
  delay(10);
  digitalWrite(trigpin1, LOW);
  //Used to read in the pulse that is being sent by the MaxSonar device. //Pulse Width


  durationback = pulseIn(echo1, HIGH);
  distanceback = durationback * 0.034 / 2;
  return distanceback;
}


bool switchSide() {
  if (analogRead(A3) > 650) {
    return 1;
  }
  else {
    return 0;
  }
}

void loop() {
  delay(3000);
  Serial.println(switchSide());
  Serial.println(analogRead(A3));
  while (millis() < 90000) {
    if (switchSide()==1) {  // right side start code
      drive(-10, 0, 69, 69, 1);
      delay(100);
      for (pos1 = 135; pos1 >= 85; pos1 -= 1) // goes from 180 degrees to 0 degrees
      {
        myservo1.write(pos1);              // tell servo to go to position in variable 'pos'
        delay(20);                       // waits 15ms for the servo to reach the position
      }
      drive(0, -25, 5, -5, 0);
      delay(50);
      drive(-35, 0, 35.7, 35.7, 1);
      delay(100);
      drive(0, 25, 23, -23, 0);
      delay(50);
      pos3 = 20;
      myservo3.write(pos3);
      drive(-20, 0, 11.5, 11.5, 0);
      delay(50);
      for (pos4 = 130; pos4 >= 60; pos4 -= 1) // goes from 180 degrees to 0 degrees
      {
        myservo4.write(pos4);              // tell servo to go to position in variable 'pos'
        delay(20);                       // waits 15ms for the servo to reach the position
      }
      drive(30, 0, 96.5, 96.5, 1);
      delay(100);
      pos = 135;
      myservo.write(pos);
      pos3 = 85;
      myservo3.write(pos3);
      drive(0, -20, 16.8, -16.8, 0);
      delay(50);
      drive(-8, 0, 4, 4, 0);
      delay(50);
      pos1 = 135;
      myservo1.write(pos1);
      pos4 = 130;
      myservo4.write(pos4);
      delay(100);
      drive(7.5, 0, 7, 7, 0);
      delay(100);
      drive(0, 20, 16.4, -16.4, 0);
      delay(100);
      pos = 50;
      myservo.write(pos);
      pos3 = 20;
      myservo3.write(pos3);
      drive(-30, 0, 95.8, 95.8, 1);
      delay(100);
      for (pos4 = 130; pos4 >= 60; pos4 -= 1) // goes from 180 degrees to 0 degrees
      {
        myservo4.write(pos4);              // tell servo to go to position in variable 'pos'
        delay(20);                       // waits 15ms for the servo to reach the position
      }
      drive(30, 0, 82, 82, 0);
      delay(50);
      pos3 = 85;
      myservo3.write(pos3);
      drive(0, 20, 11, -11, 0);
      delay(50);
      drive(-30, 0, 87, 87, 1);
      delay(50);
      for (pos1 = 120; pos1 >= 85; pos1 -= 1) // goes from 180 degrees to 0 degrees
      {
        myservo1.write(pos1);              // tell servo to go to position in variable 'pos'
        delay(20);                       // waits 15ms for the servo to reach the position
      }
      drive(20, 0, 20, 20, 0);
      delay(50);
      drive(0, 20, 4.7, 4.7, 0);
      delay(50);
      pos = 135;
      myservo.write(pos);
      delay(20);
      drive(-20, 0, 22.5, 22.5, 0);
      delay(50);
      pos1 = 135;
      myservo1.write(pos1);
      pos4 = 130;
      myservo4.write(pos4);
      delay(20);
      drive(20, 0, 8, 8, 1);
      delay(50);
      drive(0, 20, 22, -22, 0);
      delay(23000);
      myservo2.write(0);
      delay(400000);
      autostop();


    }

    else
    {       //left side start code blue
      pos = 140;
      myservo.write(pos);
      pos3 = 20;
      myservo3.write(pos3);
      drive(-10, 0, 69, 69, 1);
      delay(100);
      for (pos4 = 135; pos4 >= 60; pos4 -= 1) // goes from 180 degrees to 0 degrees
      {
        myservo4.write(pos4);              // tell servo to go to position in variable 'pos'
        delay(20);                       // waits 15ms for the servo to reach the position
      }
      drive(0, 25, 5, -5, 0);
      delay(50);
      drive(-35, 0, 35.7, 35.7, 1);
      delay(100);
      drive(0, -25, 23, -23, 0);
      delay(50);
      pos = 50;
      myservo.write(pos);
      drive(-20, 0, 11.5, 11.5, 0);
      delay(50);
      for (pos1 = 135; pos1 >= 60; pos1 -= 1) // goes from 180 degrees to 0 degrees
      {
        myservo1.write(pos1);              // tell servo to go to position in variable 'pos'
        delay(20);                       // waits 15ms for the servo to reach the position
      }
      drive(30, 0, 96.5, 96.5, 1);
      delay(100);
      pos = 135;
      myservo.write(pos);
      pos3 = 85;
      myservo3.write(pos3);
      drive(0, 20, 16.8, -16.8, 0);
      delay(50);
      drive(-8, 0, 4, 4, 0);
      delay(50);
      pos1 = 135;
      myservo1.write(pos1);
      pos4 = 135;
      myservo4.write(pos4);
      delay(100);
      drive(7.5, 0, 7, 7, 0);
      delay(100);
      drive(0, -20, 16.4, -16.4, 0);
      delay(100);
      pos = 135;
      myservo.write(pos);
      pos3 = 20;
      myservo3.write(pos3);
      drive(-30, 0, 95.8, 95.8, 1);
      delay(100);
      for (pos1 = 135; pos1 >= 60; pos1 -= 1) // goes from 180 degrees to 0 degrees
      {
        myservo1.write(pos1);              // tell servo to go to position in variable 'pos'
        delay(20);                       // waits 15ms for the servo to reach the position
      }
      drive(30, 0, 82, 82, 0);
      delay(50);
      pos = 50;
      myservo.write(pos);
      drive(0, -20, 11, -11, 0);
      delay(50);
      drive(-30, 0, 87, 87, 1);
      delay(50);
      for (pos4 = 130; pos4 >= 60; pos4 -= 1) // goes from 180 degrees to 0 degrees
      {
        myservo4.write(pos4);              // tell servo to go to position in variable 'pos'
        delay(20);                       // waits 15ms for the servo to reach the position
      }
      drive(20, 0, 20, 20, 0);
      delay(50);
      drive(0, -20, 4.7, 4.7, 0);
      delay(50);
      pos = 50;
      myservo.write(pos);
      pos3 = 100;
      myservo3.write(pos3);
      delay(20);
      drive(-20, 0, 22.5, 22.5, 0);
      delay(50);
      pos1 = 135;
      myservo1.write(pos1);
      pos4 = 135;
      myservo4.write(pos4);
      delay(20);
      drive(20, 0, 8, 8, 1);
      delay(50);
      drive(0, -20, 22, -22, 0);
      delay(50);
      delay(23000);
      myservo2.write(0);
      delay(400000);
      autostop();
    }
  }
  delay(28000);
  myservo2.write(0);
  delay(400000);
  autostop();



}
