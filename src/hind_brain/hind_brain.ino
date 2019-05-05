/**
  hind_brain.ino
  Purpose: Arduino Uno hindbrain firmware for Koda

  @author Connor Novak
  @email olingravl@gmail.com
  @version 0.0.1 19/02/24
*/

#include "hind_brain.h"

// Servos
Servo steerServo;
Servo velServo;

// Global Variables
float velCmd   = VEL_CMD_STOP;
float steerCmd = STEER_CMD_CENTER;
int incomingByte = 0;
unsigned long watchdogTimer;
unsigned long ledTimer;
unsigned long buttonTimer;
char serial_buffer[10];          // Buffer from serial

// States
boolean isEStopped = false;
boolean isLEDOn = false;

void setup() {

  // Wait for serial connection
  Serial.begin(BAUD_RATE);
  while (!Serial){;}

  // Setup pins
  steerServo.attach(STEER_SERVO_PIN);
  velServo.attach(VEL_SERVO_PIN);
  pinMode(ESTOP_LED_PIN, OUTPUT);
  pinMode(ESTOP_BUTTON_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP_BUTTON_PIN), onEStopPress, RISING);

  // Initialize timers
  watchdogTimer = millis();
  ledTimer = millis();
  buttonTimer = millis();
}

void loop() {

  checkWatchdog();
  if (Serial.available() > 0) {
    watchdogTimer = millis();
    parseSerialMsg();
  }

  updateLED();
  if (!isEStopped) updateState(velCmd, steerCmd);
}

void checkWatchdog() {
  // estops if watchdog has timed out

  if(millis() - watchdogTimer >= WATCHDOG_TIMEOUT) {
    if(!isEStopped) {
      eStop();
    }
  }
}

float steerAckToCmd(float ack_steer){
  //  Given ackermann steering message, returns corresponding ESC command

  // Convert from input message to output command
  if (ack_steer > STEER_MSG_CENTER) {ack_steer = map(ack_steer, STEER_MSG_CENTER, STEER_MSG_RIGHT, STEER_CMD_CENTER, STEER_CMD_RIGHT);}
  else if (ack_steer < STEER_MSG_CENTER) {ack_steer = map(ack_steer, STEER_MSG_LEFT, STEER_MSG_CENTER, STEER_CMD_LEFT, STEER_CMD_CENTER);}
  else { ack_steer = STEER_CMD_CENTER;}

  // Safety limits for signal
  if (ack_steer > STEER_CMD_LEFT) {
    ack_steer = STEER_CMD_LEFT;
  }
  else if (ack_steer < STEER_CMD_RIGHT) {
    ack_steer = STEER_CMD_RIGHT;
  }

  return ack_steer;
}

float steerCmdToAck(float cmd_steer) {
  // Given steering ESC command, return corresponding msg
  if (cmd_steer > STEER_CMD_CENTER) cmd_steer = mapPrecise(cmd_steer, STEER_CMD_LEFT, STEER_CMD_CENTER, STEER_MSG_LEFT, STEER_MSG_CENTER);
  else if (cmd_steer < STEER_CMD_CENTER) cmd_steer = mapPrecise(cmd_steer, STEER_CMD_CENTER, STEER_CMD_RIGHT, STEER_MSG_CENTER, STEER_MSG_RIGHT);
  else cmd_steer = STEER_MSG_CENTER;

  return cmd_steer;
}

float velCmdToAck(float cmd_vel) {
  // Given steering ESC command, return corresponding msg
  if (cmd_vel < VEL_CMD_STOP) cmd_vel = mapPrecise(cmd_vel, VEL_CMD_MIN, VEL_CMD_STOP, VEL_MSG_MIN, VEL_MSG_STOP);
  else if (cmd_vel > VEL_CMD_STOP) cmd_vel = mapPrecise(cmd_vel, VEL_CMD_STOP, VEL_CMD_MAX, VEL_MSG_STOP, VEL_MSG_MAX);
  else cmd_vel = VEL_MSG_STOP;

  return cmd_vel;
}

float velAckToCmd(float ack_vel){
  // given ackermann velocity, returns corresponding ESC command

  // Convert from range of input signal to range of output signal
  if (ack_vel > VEL_MSG_STOP) {ack_vel = mapPrecise(ack_vel, VEL_MSG_STOP, VEL_MSG_MAX, VEL_CMD_STOP, VEL_CMD_MAX);}
  else if (ack_vel < VEL_MSG_STOP) {ack_vel = ack_vel = mapPrecise(ack_vel, VEL_MSG_MIN, VEL_MSG_STOP, VEL_CMD_MIN, VEL_CMD_STOP);}
  else { ack_vel = VEL_CMD_STOP;}

  // Safety limits for signal
  if (ack_vel > VEL_CMD_MAX) {ack_vel = VEL_CMD_MAX;}
  else if(ack_vel < VEL_CMD_MIN) {ack_vel = VEL_CMD_MIN;}

  return ack_vel;
}

int parseSerialMsg() {
  // Reads msg from serial buffer - based on format, calls necessary functions

  // Store msg in serial_buf, up to newline char
  serial_buffer[0] = '\0';
  Serial.readBytesUntil('\n', serial_buffer, 10);

  // If ser_buf successfully overwritten, parse msg
  if (serial_buffer[0] != '\0') {

    const char* info = strtok(serial_buffer, ":"); // Remove info part of msg from strtok
    switch(serial_buffer[0]) {    // Check 1st char for command type
      case '?': { // Query cmd
        String message = "~";
        switch(serial_buffer[1]) {  // Check 2nd char for msg type
            case 'a': case 'A': // Ackermann msg
              message += String("a:");
              message += String(int(steerCmdToAck(steerCmd)));
              message += ":";
              message += String(int(velCmdToAck(velCmd)));
              break;
            case 'e': case 'E': // Estop msg
              message += String("e:");
              message += String(isEStopped);
        }

        // Convert msg to char array and send over serial
        char result[message.length()];
        message.toCharArray(result, message.length()+3);
        Serial.write(result, sizeof(result));
        Serial.write("\n");
        Serial.flush();
        break;
      }
      case '!': {// Command cmd
        switch(serial_buffer[1]) {
          case 'a': case 'A': {// Ackermann msg
            const char* s_char = strtok(NULL, ":"); // Read steer
            const char* v_char = strtok(NULL, ":");// Read vel
            steerCmd = steerAckToCmd(atof(s_char));
            velCmd = velAckToCmd(atof(v_char));
            break;
          }
          case 'e': case 'E': {// Estop msg
            const char* e_char = strtok(NULL, "\n");  // Read only val
            if(atoi(e_char)) eStop();
            else eStart();
            break;
          }
          case 'w': case 'W': {// Watchdog msg
          watchdogTimer = millis();
          }
        }
      }
    return 1;
    }
  } else return 0;
}

void updateState(int vel_cmd, int steer_cmd) {
  // Given velocity and steering message, sends vals to ESC & Servo

  velServo.write(vel_cmd);
  steerServo.write(steer_cmd);

}

void updateLED() {
  // Updates LED State

  if (isEStopped) {
    digitalWrite(ESTOP_LED_PIN, HIGH);
    isLEDOn = true;
  } else {
    if (millis() - ledTimer > LED_BLINK_DELAY) {

      // Switch state
      if (isLEDOn) {digitalWrite(ESTOP_LED_PIN, LOW);}
      else {digitalWrite(ESTOP_LED_PIN, HIGH);}

      isLEDOn = !isLEDOn;
      ledTimer = millis();
    }
  }
}

void eStop() {
  // Estops vehicle

  isEStopped = true;
  velCmd = VEL_CMD_STOP;
  updateState(velCmd, steerCmd);
}

void eStart() {
  // Disactivates isEStopped state

  isEStopped = false;
}

float mapPrecise(float x, float inMin, float inMax, float outMin, float outMax) {
  // Emulates Arduino map() function, but uses floats for precision
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

}

void onEStopPress() {
  // Toggles estopped state after debouncing
  if (millis() - buttonTimer >= DEBOUNCE_DELAY) {
    isEStopped = !isEStopped;
    if(isEStopped) { eStop();}
    else {eStart();}
    buttonTimer = millis();
  }
}
