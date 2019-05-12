#!/usr/bin/env python

import serial
import rospy as rp
import sys
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from helper_functions import *

class UnoNode():

    def __init__(self):
        """initializes class

        Creates ros constructs, msg/cmd attribute ranges, etc.
        """

        rp.init_node('hind_brain')
        self.ack_sub = rp.Subscriber('/cmd_vel', AckermannDrive, self.ackermann_cb)
        self.estop_sub = rp.Subscriber('/softestop', Bool, self.estop_cb)
        self.pose_sub = rp.Subscriber('/cmd_hitch', Pose, self.pose_cb)
        self.rate = rp.Rate(5)
        self.prev_command = ''
        self.name = 'hb'

        self.vel_range = (-1,0,1)
        self.steer_range = (-45,0,45)
        self.z_range = (-0.3,0,0.3)
        self.pitch_range = (-45,0,45)
        self.msg_range = (0,50,100)

        # Setup serial port
        try:
            port = rp.get_param('/hind_brain/port')
            self.uno_port = serial.Serial(port, 9600, write_timeout=1)
        except KeyError as e:
            rp.logerr("Cannot find /hind_brain/port param - did you use the launch file?")
            rp.signal_shutdown(e)

    def ackermann_cb(self, msg):
        """callback function for ackermann messages

        Converts ackermann msg into a serial msg, sends if msg is different than
        last msg sent

        :param[in] msg: ackermann msg provided by ROS subscriber
        :param[out] self.prev_command: Updates previous command attribute
        """

        speed_cmd = cmd2msg(msg.speed, self.vel_range, self.msg_range)
        steer_cmd = cmd2msg(msg.steering_angle, self.steer_range, self.msg_range)
        command = "!a:" + str(int(speed_cmd)) + ":" + str(int(steer_cmd)) + ":"
        if (command != self.prev_command):
            self.send(command)
            self.prev_command = command

    def estop_cb(self, msg):
        """callback function for estop messages

        Converts estop msg into serial msg, sends if msg is different than
        last msg sent

        :param[in] msg: boolean msg provided by ROS subscriber
        :param[out] self.prev_command: Updates previous command attribute
        """
        command = "!e:" + str(msg.data) + ":"
        if (command != self.prev_command):
            self.send(command)
            self.prev_command = command

    def pose_cb(self, msg):
        """callback function for pose messages

        Converts pose msg into serial msg, sends if msg is different than
        last msg sent

        :param[in] msg: pose msg provided by ROS subscriber
        :param[out] self.prev_command: Updates previous command attribute
        """
        z_cmd = cmd2msg(msg.position.z, self.z_range, self.msg_range)
        pitch_cmd = cmd2msg(msg.orientation.y, self.pitch_range, self.msg_range)
        command = "!b:" + str(int(z_cmd)) + ":" + str(int(pitch_cmd)) + ":\n"
        if (command != self.prev_command):
            self.send(command)
            self.prev_command = command

    def send(self, msg):
        """sends msg over serial

        Tries to write msg to serial, assumes lost connectivity if failed

        :param[in] msg: string to send over serial
        :return: failure boolean
        """
        try:
            self.uno_port.write(msg.encode('utf-8'))
            rp.loginfo("%s - Sent message: %s", self.name, msg)
            return True
        except Exception as e:
            rp.logerr("%s - lost connectivity with rover", self.name)
            rp.logerr(e)
            return False

    def run(self):
        """runs node mainloop

        sends watchdog msg at rate attr. frequency if ros is ok
        """
        while not rp.is_shutdown():
            #self.send('!w') # Satisfy watchdog
            self.rate.sleep()

if __name__ == '__main__':
    unode = UnoNode()
    unode.run()
