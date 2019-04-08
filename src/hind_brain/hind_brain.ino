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
int velCmd = VEL_CMD_STOP;
int steerCmd = STEER_CMD_CENTER;
int incomingByte = 0;
unsigned long watchdogTimer;
unsigned long ledTimer;
unsigned long buttonTimer;

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
    readAckMsg(&velCmd, &steerCmd);
    watchdogTimer = millis();
  }

  updateLED();

  if (!isEStopped) {updateState(velCmd, steerCmd);}

}

void checkWatchdog() {
  // estops if watchdog has timed out

  if(millis() - watchdogTimer >= WATCHDOG_TIMEOUT) {
    if(!isEStopped) {
      eStop();
    }
  }
}

int steerAckToCmd(float ack_steer){
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

int velAckToCmd(float ack_vel){
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

bool isCommand() {
// Returns true if message is CMD, false if QUE
auto msg = Serial.read();
if (msg == '!') return true;
else if (msg == '?') return false;
}



void readAckMsg(int *vel, int *steer){
  // Reads Serial port, returns msg attributes

  if (Serial.read() != '[') return; // Checks for message start char

  *vel = velAckToCmd(Serial.readStringUntil(':').toFloat());
  *steer = steerAckToCmd(Serial.readStringUntil('\n').toFloat());

  while(Serial.available()) {Serial.read();} // clears out extra chars
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
