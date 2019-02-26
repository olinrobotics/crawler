# crawler
**A Codebase for the Koda GRAVL Mini-platform**

This package contains software and firmware specific to koda, GRAVL's miniature
prototyping ground vehicle

<img src="" width=300/>

_Image 1: Koda, Iteration 0_

## documentation

### How to Run Koda

Currently, Koda is a tethered platform. The Arduino must be connected to a computer
via USB in order to send commands, as the ROScore is run offboard of the Arduino.

1. Connect a USB cable between Koda and a laptop with ROS and the `crawler` repository set up.

2. In a new terminal, launch a new roscore with the command `roscore`

* Launch files can start their own roscore, but having a separate core ensures that your ROS instances stay connected if any nodes in the launch file crash.

3. Plug in an XBox controller, or another controller compatible with the Teleop node.

4.  In a new terminal, run the basic nodes required for teleoperating Koda with the command `roslaunch crawler bringup_minimal.launch port:=/dev/ttyACM0`

* The port argument represents which port the Arduino is plugged into. To determine the port, check the Arduino IDE, or run the command `ls /dev/ttyACM*`
* The default for the port argument is `/dev/ttyACM0`

5. Turn on Koda's main power using the power switch.

* If Koda is E-stopped (LED is solid ON), press the EStop button or reset the Arduino onboard.

6. Press the 'A' button on the XBox controller to activate the controls.

### Authors

This package was built by Connor Novak (20'), Nathan Estill (21'), and Kawin Nikomborirak (21') for the Ground Robotic Autonomous Vehicle Laboratory
([link](https://github.com/olinrobotics/gravl/wiki)).
