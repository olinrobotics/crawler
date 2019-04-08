/**
* serial_msgs.h
* Purpose: Serial protocol msgs
*
* @author Connor Novak
* @email olingravl@gmail.com
* @version 0.0.2 19/03/07
**/

#ifndef SERIAL_MSGS_H
#define SERIAL_MSGS_H

// Command Set
#define C_WDG "!W"    // Watchdog - 0
#define C_STP "!E"    // Estop - 1
#define C_ACK "!A"    // Ackermann - 2
#define C_BLD "!B"    // Blade - 3

// Query Set
#define Q_ACK "?AK"   // Ackermann
#define Q_WDG "?W"    // Watchdog
#define Q_STP "?STP"  // Estop

// Answer Set
#define A_ACK "~A"    // Ackermann
#define A_WDG "~W"    // Watchdog
#define A_BLN "~B"    // Boolean
#define A_INT "~I"    // Integer
#define A_FLT "~F"    // Float

#endif
