/**
  hind_brain.h
  Purpose: Hindbrain libraries, constants, tuning parameters

  @author Connor Novak
  @email olingravl@gmail.com
  @version 0.0.1 19/02/24
*/

#ifndef HIND_BRAIN_H
#define HIND_BRAIN_H

// Libraries
#include <Servo.h>                          // ESC and Servo PWM control
#include <Arduino.h>                        // Used for Arduino functions

// Arduino Pins
const byte ESTOP_BUTTON_PIN = 3;
const byte STEER_SERVO_PIN = 5;
const byte VEL_SERVO_PIN   = 6;
const byte ESTOP_LED_PIN   = 13;

// General Constants
#define DEBUG True
#define WATCHDOG_TIMEOUT 250        // milliseconds
#define LED_BLINK_DELAY 750         // milliseconds
#define BAUD_RATE 9600              // Hz
#define DEBOUNCE_DELAY 500          // milliseconds

// Velocity Motor Ranges
const int VEL_CMD_MIN = 60;         // ESC cmd for min speed
const int VEL_CMD_STOP = 91;        // ESC cmd for 0 speed
const int VEL_CMD_MAX = 110;        // ESC cmd for max speed (actual max 140)
const int VEL_MSG_MIN = -2;         // Ackermann msg min speed
const int VEL_MSG_STOP = 0;         // Ackermann msg 0 speed
const int VEL_MSG_MAX = 2;          // Ackermann msg max speed

// Steering Motor Ranges
const int STEER_CMD_LEFT = 160;        // Servo library cmd for max left turn
const int STEER_CMD_CENTER = 85;     // Servo library cmd for straight
const int STEER_CMD_RIGHT = 35;      // Servo library cmd for max right turn
const int STEER_MSG_LEFT = 45;      // Ackermann msg min steering angle
const int STEER_MSG_CENTER = 0;     // Ackermann msg center steering angle
const int STEER_MSG_RIGHT = -45;       // Ackermann msg max steering angle

#endif
