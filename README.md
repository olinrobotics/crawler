# crawler
**A Codebase for the Koda GRAVL Mini-platform**

This package contains software and firmware specific to koda, GRAVL's miniature
prototyping ground vehicle

<img src="https://lh3.googleusercontent.com/Jx1OQg4AAH1TizGKpdFJIWJzOM1vo685XgYTWU95Av1HF5BUWwxS8cGzk8nvpuet6Gna2lvbTxpi2G6mdRw2R_2TdJpQuo_9b9-MwRltptuHaXnXMHdIBqKRYLPDcgkr1N1zLTyi2vOLsDmTJvDfJbxp70PfpnkkFRH41RxhJDAhgZ3w6NtVWjFsZlS_mik2-1JKsx0_eyUGYM5lwi-rbeaDPIIwna_nReINlVVLEPGuM7SrccHoSi1tur4QrrEq43-ADs8lF78j-3EalirCbOvOkU3bgqljQYcVaIfQpFkQU_1FNaIts122rshhFGtB8WuzIWW2euLrsOZrn2ilZ-4J3sIizO0bxluezjKNUrjIEZpkMbMb-72mGM3Q7DdFiGdPKtKW9MUqrFOMBIf31hgKjXnBKgzHsiLZ44s9Xb25_i9CDOeblnVeyTMPs6G904XoXEi6DHid_sbyPHk2lGAfdNYPi0Wmd5R9euZxRj56PmZM_hl6B_KHBJ65Wh-R1pvU_16DcuwfAvir787EtqMjBeJMLEcZOAnQ27JrVjFp36MqFnzCQQ_IbCV1BaWu-uu-nRHzgzNBmGCp3zqSrOZEdBKebARhyxHZq4MuFXRiKqGEMl_aZDV_Um_mEg976Ia8muaGFcwJ072vikmc35y0EfDFsSo=w1279-h959-no" width=400/>

_Image 1: Koda, Iteration 0_

## documentation

### Setup/Installation
+ Dependencies: https://github.com/olinrobotics/state_controller.git

### How to Run Koda

Currently, Koda has no onboard computer. The onboard microprocessor must be connected to a computer via XBee in order to send commands, as the ROScore is run offboard of the Arduino.

1. Connect an XBee radio USB breakout board to a laptop with ROS and the `crawler` repository set up.

2. In a new terminal, launch a new roscore with the command `roscore`

* Launch files can start their own roscore, but having a separate core ensures that your ROS instances stay connected if any nodes in the launch file crash.

3. Plug in an XBox controller, or another controller compatible with the Teleop node.. Test

4.  In a new terminal, run the basic nodes required for teleoperating Koda with the command `roslaunch crawler bringup_minimal.launch port:=/dev/ttyUSB0`

* The port argument represents which port the Arduino is plugged into. To determine the port, check the Arduino IDE, or run the command `ls /dev/ttyUSB*`
* The default for the port argument is `/dev/ttyUSB0`

5. Turn on Koda's main power using the power switch.

* If Koda is E-stopped (LED is solid ON), press the EStop button or reset the Arduino onboard.

6. Press the 'A' button on the XBox controller to activate the controls.

### Uploading New Code
1. Remove the Xbee from Koda
2. Flip the Serial Select switch on the red sparkfun board under the Xbee from HW_SER to SW_SER
3. Upload code using the Arduino IDE
4. Flip the Serial Select switch on the red sparkfun board under the Xbee from SW_SER back to HW_SER
5. Re-plug in the Xbee
6. Test code

### Authors

This package was built by Connor Novak (20'), Nathan Estill (21'), and Kawin Nikomborirak (21') for the Ground Robotic Autonomous Vehicle Laboratory
([link](https://github.com/olinrobotics/gravl/wiki)).
