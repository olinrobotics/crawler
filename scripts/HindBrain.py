#!/usr/bin/env python

import serial
import rospy as rp
import sys
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool

class UnoNode():

    def __init__(self):

        rp.init_node('hind_brain')
        self.ack_sub = rp.Subscriber('/cmd_vel', AckermannDrive, self.ackermann_cb)
        self.estop_sub = rp.Subscriber('/softestop', Bool, self.estop_cb)
        self.rate = rp.Rate(5)
        self.prev_command = ''

        self.vel_range = [-1,0,1]
        self.steer_range = [-45,0,45]
        self.cmd_range = [0,50,100]

        # Setup serial port
        try:
            port = rp.get_param('/hind_brain/port')
            self.uno_port = serial.Serial(port, 9600, write_timeout=1)
        except KeyError as e:
            rp.logerr("Cannot find /hind_brain/port param - did you use the launch file?")
            rp.signal_shutdown(e)

    def steerAckToMsg(ack_steer):
        # Given ackermann steering cmd, returns corresponding Serial msg

        # Convert from input message to output command
        if ack_steer > self.ack_range[1]: ack_steer = mapPrecise(ack_steer, self.ack_range[1], self.ack_range[2], self.cmd_range[1], self.cmd_range[2])
        elif ack_steer < self.ack_range[1]: ack_steer = mapPrecise(ack_steer, self.ack_range[0], self.ack_range[1], self.cmd_range[0], self.cmd_range[1])
        else ack_steer = self.ack_range[1]: ack_steer = self.cmd_range[1]
        return ack_steer

    def ackermann_cb(self, msg):

        speed_cmd = steerAckToMsg(msg.speed)
        steer_cmd = mapPrecise(msg.steering_angle, ack_range)
        command = "!a:" + str(speed_cmd) + ":" + str(msg.steering_angle) + ":"
        if (command != self.prev_command):
            print("Transmitted: " + command)
            self.send(command)
            self.prev_command = command

    def estop_cb(self, msg):

        command = "!e:" + str(msg.data) + ":"
        if (command != self.prev_command):
            self.send(command)
            self.prev_command = command

    def mapPrecise(x, inMin, inMax, outMin, outMax):
      # Emulates Arduino map() function, but uses floats for precision
      return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

    def send(self, msg):

        try:
            self.uno_port.write(msg.encode('utf-8'))
        except Exception as e:
            rp.logerr("Lost connectivity with rover")
            rp.logerr(e)

    def run(self):
        while not rp.is_shutdown():
            self.send('!w') # Satisfy watchdog
            self.rate.sleep()

if __name__ == '__main__':
    unode = UnoNode()
    unode.run()
