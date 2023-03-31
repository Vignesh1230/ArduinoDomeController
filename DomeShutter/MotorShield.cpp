/*******************************************************************************
Monster Motor Shield library

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/

#include <Arduino.h>
#include "MonsterMotorShield.h"


/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)


/* Motor driver pin definitions */
int pwmPin = 5; // PWM input
int dirPin = 7; // Direction input

const int PWMFreq = 5000; /* 5 KHz */
const int PWMChannel = 0;
const int PWMResolution = 10;

Motor::Motor(uint8_t n)
{
    // _nmotor can be only 0 or 1
    _nmotor = (n > 0);
    pinMode(dirPin, OUTPUT);
    
    ledcSetup(PWMChannel, PWMFreq, PWMResolution);
    /* Attach the LED PWM Channel to the GPIO Pin */
    ledcAttachPin(pwmPin, PWMChannel);
    stop();
}

void Motor::run(bool dir, int pwm)
{
    digitalWrite(dirPin, dir); // Set direction to stop
    ledcWrite(PWMChannel, pwm); // Set PWM value to running val
}

void Motor::stop()
{
    digitalWrite(dirPin, LOW); // Set direction to stop
    ledcWrite(PWMChannel, 0); // Set PWM value to running val
}

void Motor::brake()
{
    digitalWrite(dirPin, LOW); // Set direction to stop
    ledcWrite(PWMChannel, 0); // Set PWM value to running val