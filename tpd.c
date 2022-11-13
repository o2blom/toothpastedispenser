//www.elegoo.com
//2018.10.25

/*
  Stepper Motor Control - one revolution

  This program drives a unipolar or bipolar stepper motor.
  The motor is attached to digital pins 8 - 11 of the Arduino.

  The motor should revolve one revolution in one direction, then
  one revolution in the other direction.

*/

#include <Stepper.h>
#include <Servo.h>
#include <AccelStepper.h>

const int forwardSteps = 5;  // change this to fit the number of steps per revolution
const int retractionSteps = 100;
const int rolePerMinute = 2;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm
const int closedPos = 101; 
const int openPos = 0;

const int inPin = 4;
const int servoPin = 3;


// initialize the stepper library on pins 8 through 11:
Stepper myStepper(2048, 8, 10, 9, 11);
Servo myservo;  // create servo object to control a servo

int pos;

void setup() {
  myStepper.setSpeed(rolePerMinute);
  myservo.attach(servoPin);  // attaches the servo on pin 3 to the servo object
  myservo.write(closedPos); 
//  delay(5000);
//  myservo.write(openPos);
  pinMode(inPin, INPUT_PULLUP);    // sets the digital pin 4 as input
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {  
  int active;
  static int firstTime = 1;

//  myservo.write(closedPos); 
//  delay(5000);
//  myservo.write(openPos);
//  delay(5000);
//
//  return;

  active = !digitalRead(inPin);

  if (firstTime)  //If button is pressed upon powerup, its maintenance mode - Run in reverse
  {
      firstTime = 0;
      while(active) 
      {
        active = !digitalRead(inPin);   
        myStepper.step(forwardSteps);
      }  
  }
     
  if (active)
  {
    myservo.write(openPos); 
    myStepper.step(-forwardSteps);
    while(active) 
    {
      active = !digitalRead(inPin);   
      myStepper.step(-forwardSteps);
    }

    myStepper.step(retractionSteps);  //Back off a little to reduce pressure
    
 //   delay(700); //Wait for pressure to go down
    myservo.write(closedPos); 
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
  }


}
 
