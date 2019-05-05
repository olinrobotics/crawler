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
Servo pitchServo;
Servo zServo;

// Global Variables
float velCmd   = VEL_CMD_STOP;
float steerCmd = STEER_CMD_CENTER;
float pitchCmd = PITCH_CMD_CENTER;
float zCmd     = Z_CMD_TOP;
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
  pitchServo.attach(PITCH_SERVO_PIN);
  zServo.attach(Z_SERVO_PIN, Z_PWM_MIN, Z_PWM_MAX);
  pinMode(ESTOP_LED_PIN, OUTPUT);
  pinMode(ESTOP_BUTTON_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP_BUTTON_PIN), onEStopPress, RISING);

  // Initialize timers
  watchdogTimer = millis();
  ledTimer = millis();
  buttonTimer = millis();
}

void loop() {

  updateWatchdog();

  // Parse msg if data on serial port
  if (Serial.available() > 0) {
    watchdogTimer = millis();
    parseSerialMsg();
  }

  // Update motor states if running
  if (!isEStopped) {
    updateState(velCmd, steerCmd);
    updateBladeState(zCmd, pitchCmd);
  }

  updateLED();
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
            case 'a': case 'A': {// Ackermann msg
              message += String("a:");
              message += String(int(cmdToMsg(steerCmd,
                                             STEER_CMD_LEFT,STEER_CMD_CENTER,STEER_CMD_RIGHT,
                                             STEER_MSG_LEFT,STEER_MSG_CENTER,STEER_MSG_RIGHT)));
              message += ":";
              message += String(int(cmdToMsg(velCmd,
                                             VEL_CMD_MIN,VEL_CMD_STOP,VEL_CMD_MAX,
                                             VEL_MSG_MIN,VEL_MSG_STOP,VEL_MSG_MAX)));
              break;
            }
            case 'b': case 'B': {// Blade msg
              message += String("b:");
              message += String(int(cmdToMsg(zCmd,
                                             Z_CMD_BOTTOM,Z_CMD_CENTER,Z_CMD_TOP,
                                             Z_MSG_BOTTOM,Z_MSG_CENTER,Z_MSG_TOP)));
              message += ":";
              message += String(int(cmdToMsg(pitchCmd,
                                             PITCH_CMD_DOWN,PITCH_CMD_CENTER,PITCH_CMD_UP,
                                             PITCH_MSG_DOWN,PITCH_MSG_CENTER,PITCH_MSG_UP)));
              break;
            }
            case 'e': case 'E': {// Estop msg
              message += String("e:");
              message += String(isEStopped);
            }
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
            steerCmd = msgToCmd(atof(s_char),
                                STEER_MSG_LEFT,STEER_MSG_CENTER,STEER_MSG_RIGHT,
                                STEER_CMD_LEFT,STEER_CMD_CENTER,STEER_CMD_RIGHT);
            velCmd = msgToCmd(atof(v_char),
                              VEL_MSG_MIN,VEL_MSG_STOP,VEL_MSG_MAX,
                              VEL_CMD_MIN,VEL_CMD_STOP,VEL_CMD_MAX);
            break;
          }
          case 'b': case 'B': {// Blade msg
            const char* z_char = strtok(NULL, ":"); // Read z
            const char* p_char = strtok(NULL, ":"); // Read pitch
            zCmd = msgToCmd(atof(z_char),
                            Z_MSG_BOTTOM,Z_MSG_CENTER,Z_MSG_TOP,
                            Z_CMD_BOTTOM,Z_CMD_CENTER,Z_CMD_TOP);
	          pitchCmd = msgToCmd(atof(p_char),
                                PITCH_MSG_DOWN,PITCH_MSG_CENTER,PITCH_MSG_UP,
                                PITCH_CMD_DOWN,PITCH_CMD_CENTER,PITCH_CMD_UP);
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

void updateWatchdog() {
  // estops if watchdog has timed out

  if(millis() - watchdogTimer >= WATCHDOG_TIMEOUT) {
    if(!isEStopped) {
      eStop();
    }
  }
}

void updateState(int vel_cmd, int steer_cmd) {
  // Given velocity and steering message, sends vals to ESC & Servo

  velServo.write(vel_cmd);
  steerServo.write(steer_cmd);

}

void updateBladeState(int z_cmd, int pitch_cmd) {
  // Given z and pitch commands, sends commands to blade servos

  zServo.write(z_cmd);
  pitchServo.write(pitch_cmd);

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

void onEStopPress() {
  // Toggles estopped state after debouncing
  if (millis() - buttonTimer >= DEBOUNCE_DELAY) {
    isEStopped = !isEStopped;
    if(isEStopped) { eStop();}
    else {eStart();}
    buttonTimer = millis();
  }
}

float msgToCmd(float m_in, float m_low, float m_mid, float m_hi, float c_low, float c_mid, float c_hi) {
  //  Given msg and msg low-mid-high range, returns corresponding cmd within cmd low-mid-high range
  float c_out;

  // Convert from input message to output command
  if      (m_in > m_mid) c_out = map(m_in, m_mid, m_hi, c_mid, c_hi);
  else if (m_in < m_mid) c_out = map(m_in, m_low, m_mid, c_low, c_mid);
  else                   c_out = c_mid;

  // Safety limits for signal
  if (c_low < c_hi) { // Ensure correct direction of comparison - is c_high < c_low?
    if      (c_out > c_hi)  c_out = c_hi;
    else if (c_out < c_low) c_out = c_low;
  } else if (c_low > c_hi) {
    if      (c_out > c_low) c_out = c_low;
    else if (c_out < c_hi) c_out = c_hi;
  }

  return c_out;
}

float cmdToMsg(float c_in, float c_low, float c_mid, float c_hi, float m_low, float m_mid, float m_hi) {
  //  Given cmd and cmd low-mid-high range, returns corresponding msg within msg low-mid-high range
  float m_out;

  // Convert from input command to output message
  if (c_low < c_hi) { // Ensure correct direction of comparison - is c_high < c_low?
    if (c_in < c_mid) m_out = mapPrecise(c_in, c_low, c_mid, m_low, m_mid);
    else if (c_in > c_mid) m_out = mapPrecise(c_in, c_mid, c_hi, m_mid, m_hi);
    else m_out = m_mid;
  } else if (c_low > c_hi) {
    if (c_in > c_mid) m_out = mapPrecise(c_in, c_low, c_mid, m_low, m_mid);
    else if (c_in < c_mid) m_out = mapPrecise(c_in, c_mid, c_hi, m_mid, m_hi);
    else m_out = m_mid;
  }

  return m_out;
}

float mapPrecise(float x, float inMin, float inMax, float outMin, float outMax) {
  // Emulates Arduino map() function, but uses floats for precision
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

}
