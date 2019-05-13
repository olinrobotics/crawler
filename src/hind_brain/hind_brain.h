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
const byte Z_SERVO_PIN     = 9;
const byte PITCH_SERVO_PIN = 10;
const byte PITCH_SERVO2_PIN = 11;
const byte ESTOP_LED_PIN   = 13;

// General Constants
#define DEBUG True
#define WATCHDOG_TIMEOUT 500        // milliseconds
#define LED_BLINK_DELAY 500         // milliseconds
#define BAUD_RATE 9600              // Hz
#define DEBOUNCE_DELAY 500          // milliseconds

// Velocity Motor Ranges
const int VEL_CMD_MIN  = 60;        // ESC cmd for min speed
const int VEL_CMD_STOP = 91;        // ESC cmd for 0 speed
const int VEL_CMD_MAX  = 110;       // ESC cmd for max speed (actual max 140)
const int VEL_MSG_MIN  = 0;         // Ackermann msg min speed
const int VEL_MSG_STOP = 50;        // Ackermann msg 0 speed
const int VEL_MSG_MAX  = 100;       // Ackermann msg max speed

// Steering Motor Ranges
const int STEER_CMD_LEFT   = 160;   // Servo library cmd for max left turn
const int STEER_CMD_CENTER = 85;    // Servo library cmd for straight
const int STEER_CMD_RIGHT  = 35;    // Servo library cmd for max right turn
const int STEER_MSG_LEFT   = 100;     // Ackermann msg min steering angle
const int STEER_MSG_CENTER = 50;    // Ackermann msg center steering angle
const int STEER_MSG_RIGHT  = 0;   // Ackermann msg max steering angle

// Blade Z Motor Ranges
const int Z_CMD_TOP = 60;           // Servo cmd for highest blade height
const int Z_CMD_CENTER = 105;       // . . .         middle . . .
const int Z_CMD_BOTTOM = 150;       // . . .         lowest . . .
const int Z_MSG_TOP = 100;          // Pose msg highest blade height
const int Z_MSG_CENTER = 50;        // . . .    middle . . .
const int Z_MSG_BOTTOM = 0;         // . . .    lowest . . .

// Blade Pitch Motor Ranges
const int PITCH_CMD_UP = 60;         // Servo cmd for maximum upward   blade pitch
const int PITCH_CMD_CENTER = 30;    // . . .         central          . . .
const int PITCH_CMD_DOWN = 0;     // . . .         maximum downward . . .
const int PITCH_MSG_UP = 100;         // Pose msg maximum upward   blade pitch
const int PITCH_MSG_CENTER = 50;    // . . .    central          . . .
const int PITCH_MSG_DOWN = 0;     // . . .    maximum downward . . .

// Blade Z Motor PWM Tuning
const int Z_PWM_MIN = 1000;
const int Z_PWM_MAX = 2450;
#endif
