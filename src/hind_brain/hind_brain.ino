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
Servo pitchServo2;
Servo zServo;

// Global Variables
float velCmd   = VEL_CMD_STOP;
float steerCmd = STEER_CMD_CENTER;
float pitchCmd = PITCH_CMD_CENTER;
float zCmd     = Z_CMD_TOP;
int incomingByte = 0;

// Serial Reading Variables
bool newMsg = false;    // Flag for new msg
char rawMsg[SERIAL_BUFFER_LENGTH];       // Storage for new msg
int loopTime;           // Time per loop() cycle

// Timers
unsigned long loopTimer;
unsigned long watchdogTimer;
unsigned long ledTimer;
unsigned long buttonTimer;

// States
boolean isEStopped = false;
boolean isLEDOn = false;

void setup() {
  /** Sets up platform to run

  Starts serial connection, attaches motors, sensors, indicators to pins, sets
  hitch to neutral position, updates all timers

  :param[out] watchdogTimer: updates timer at end of setup
  :param[out] ledTimer:      . . .
  :param[out] loopTimer:     . . .
  :param[out] buttonTimer:   . . .
  **/

  // Wait for serial connection
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT);
  while (!Serial){;}

  // Setup pins
  steerServo.attach(STEER_SERVO_PIN);
  velServo.attach(VEL_SERVO_PIN);
  pitchServo.attach(PITCH_SERVO_PIN);
  pitchServo2.attach(PITCH_SERVO2_PIN);
  zServo.attach(Z_SERVO_PIN, Z_PWM_MIN, Z_PWM_MAX);
  pinMode(ESTOP_LED_PIN, OUTPUT);
  pinMode(ESTOP_BUTTON_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP_BUTTON_PIN), onEStopPress, RISING);

  // Set blade to init pose
  updateBladeState(zCmd, pitchCmd);

  // Initialize timers
  watchdogTimer = millis();
  ledTimer = millis();
  loopTimer = millis();
  buttonTimer = millis();
}

void loop() {
  /** Main update loop of platform

    Updates the watchdog, checks for serial messages, sends motor commands if not
    estopped, updates led state, and logs it's loop time.

    :param[out] loopTimer: updates loopTimer at beginning of loop
    :param[out] loopTime: updats loopTime at end of loop
  **/

  loopTimer = millis(); // Start loop timer

  updateWatchdog();

  // Parse msg if data on serial port
  if (Serial.available() > 0) {
    watchdogTimer = millis();
    readSerialMsg();
    if (newMsg) parseSerialMsg();
  }

  // Update motor states if running
  if (!isEStopped) {
    updateState(velCmd, steerCmd);
    updateBladeState(zCmd, pitchCmd);
  }

  updateLED();

  loopTime = millis() - loopTimer;
}

void readSerialMsg() {
  /** Read msg from serial port into global var

  Function replaces & augments Serial.readBytesUntil() to prevent blocking of
  program. Reads from start msg char ('\s') to end msg char ('\n'), stores msg
  in msg_raw char array if the mesage fits, otherwise dumps the message. Flips
  the newMsg flag if msg successfully stored.
  Source: https://forum.arduino.cc/index.php?topic=396450.0

  :param[out] newMsg: flips newMsg flag if successfully reads a msg
  :param[out] rawMsg: populates rawMsg[] with read chars
  **/

  static boolean readingMsg = false;
  static byte index = 0;
  char startChar = '\s';
  char   endChar = '\n';
  char rc;

  // info on serial port and no message waiting for parsing
  while (Serial.available() > 0 && newMsg == false) {
    rc = Serial.read();

    if (readingMsg) { // Deal with msg valid char
      if (rc != endChar) { // Save char to buffer
        rawMsg[index] = rc;
        index++;
        if (index >= SERIAL_BUFFER_LENGTH) index = SERIAL_BUFFER_LENGTH - 1; // Overwrite last char if overflow
      } else { // Finish writing buffer and flip flag
        rawMsg[index] = '\0';
        readingMsg = false;
        index = 0;
        newMsg = true;
      }
    }

    else if (rc == startChar) readingMsg = true;
  }

}

int parseSerialMsg() {
  /** Reads serial message and responds accordingly

    Reads command from newMsg
    Sorts into command ('!') or query ('?'), and then into steering ('a'), estop
    ('e'), watchdog ('w'), hitch ('b'), or loop timer ('l'). Based on subsequent values, updates
    respective global state variables ('!') or sends response msg with queried
    information ('?')

    :param[out] steerCmd: Updates steering command value if receiving proper command
    :param[out] velCmd:   . . .   velocity . . .
    :param[out] steerCmd: . . .   steering . . .
    :param[out] zCmd:     . . .   hitch height . . .
    :param[out] watchdogTimer: Resets timer if called
    :return int: successful read bool value
  **/

  char tempMsg[SERIAL_BUFFER_LENGTH]; // Buffer from serial
  strcpy(tempMsg, rawMsg); // temporary copy is necessary to protect original data
                           // because strtok() replaces spacers with \0

  // If ser_buf successfully overwritten, parse msg
  const char* info = strtok(tempMsg, ":"); // Remove info part of msg from strtok
  switch(tempMsg[0]) {    // Check 1st char for command type
    case '?': { // Query cmd
      String message = "~";
      switch(tempMsg[1]) {  // Check 2nd char for msg type
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
          case 'l': case 'L': {// Loop timer query
            message += String("l");
            message += String(loopTime);
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
      switch(tempMsg[1]) {
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
}

void updateWatchdog() {
  /** estops if watchdog timeout

    Compares watchdog timer to timeout value, and estops if past the threshold.
  **/

  if(millis() - watchdogTimer >= WATCHDOG_TIMEOUT && !isEStopped) eStop();
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
  pitchServo2.write(PITCH_CMD_UP - pitch_cmd);

}

void updateLED() {
  // Updates LED state

  if (isEStopped) {
    digitalWrite(ESTOP_LED_PIN, HIGH);
    isLEDOn = true;
  } else {
    if (millis() - ledTimer > LED_BLINK_DELAY) {

      // Switch state
      if (isLEDOn) digitalWrite(ESTOP_LED_PIN, LOW);
      else digitalWrite(ESTOP_LED_PIN, HIGH);

      isLEDOn = !isLEDOn;
      ledTimer = millis();
    }
  }
}

void eStop() {
  // Estops vehicle

  Serial.write(isEStopped);
  isEStopped = true;
  velCmd = VEL_CMD_STOP;
  updateState(velCmd, steerCmd);
  Serial.write("~e:1:\n");
  Serial.flush();
}

void eStart() {
  // Disactivates isEStopped state

  isEStopped = false;
  Serial.write("~e:0:\n");
  Serial.flush();
}

void onEStopPress() {
  // Toggles estopped state after debouncing
  if (millis() - buttonTimer >= DEBOUNCE_DELAY) {
    if (!isEStopped) eStop();
    else             eStart();
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
    else if (c_out < c_hi)  c_out = c_hi;
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

  // Threshold outgoing msg
  if (m_low < m_hi) {
    if      (m_out < m_low) m_out = m_low;
    else if (m_out > m_hi)  m_out = m_hi;
  } else if (m_low > m_hi) {
    if      (m_out > m_low) m_out = m_low;
    else if (m_out < m_hi)  m_out = m_hi;
  }
  return m_out;
}

float mapPrecise(float x, float inMin, float inMax, float outMin, float outMax) {
  // Emulates Arduino map() function, but uses floats for precision
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

}
