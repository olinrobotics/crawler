#!/usr/bin/env python

import serial
import rospy as rp
import sys
from ackermann_msgs.msg import AckermannDrive

class UnoNode():

    def __init__(self):

        rp.init_node('hind_brain')
        self.ack_sub = rp.Subscriber('/cmd_vel', AckermannDrive, self.ackermann_cb)
        self.rate = rp.Rate(5)
        self.prev_command = ''

        # Setup serial port
        try:
            port = rp.get_param('/hind_brain/port')
            self.uno_port = serial.Serial(port, 9600, write_timeout=1)
        except KeyError as e:
            rp.logerr("Cannot find /hind_brain/port param - did you use the launch file?")
            rp.signal_shutdown(e)

    def ackermann_cb(self, msg):

        command = "[" + str(msg.speed) + "|" + str(msg.steering_angle) + "]"
        if (command != self.prev_command):
            print("Trasmitted: " + command)
            self.send(command)
            self.prev_command = command

    def send(self, msg):

        try:
            self.uno_port.write(msg.encode('utf-8'))
        except Exception as e:
            rp.logerr("Lost connectivity with rover")
            rp.logerr(e)

    def run(self):
        while not rp.is_shutdown():
            self.send('.') # Satisfy watchdog
            self.rate.sleep()

if __name__ == '__main__':
    unode = UnoNode()
    unode.run()
