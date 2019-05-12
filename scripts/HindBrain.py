#!/usr/bin/env python

import serial
import rospy as rp
import sys
from ackermann_msgs.msg import AckermannDrive
<<<<<<< Updated upstream
=======
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from helper_functions import *
>>>>>>> Stashed changes

# Hello
class UnoNode():

    def __init__(self):

        rp.init_node('hind_brain')
        self.ack_sub = rp.Subscriber('/cmd_vel', AckermannDrive, self.ackermann_cb)
        self.rate = rp.Rate(5)
        self.prev_command = ''
        self.name = 'hb'

        # Setup serial port
        try:
            port = rp.get_param('/hind_brain/port')
            self.uno_port = serial.Serial(port, 9600, write_timeout=1)
        except KeyError as e:
            rp.logerr("Cannot find /hind_brain/port param - did you use the launch file?")
            rp.signal_shutdown(e)

    def ackermann_cb(self, msg):

<<<<<<< Updated upstream
        command = "[" + str(msg.speed) + "|" + str(msg.steering_angle) + "]"
=======
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
>>>>>>> Stashed changes
        if (command != self.prev_command):
            print("Trasmitted: " + command)
            self.send(command)
            self.prev_command = command

    def send(self, msg):

        try:
            self.uno_port.write(msg.encode('utf-8'))
<<<<<<< Updated upstream
=======
            rp.loginfo("%s - Sent message: %s", self.name, msg)
            return True
>>>>>>> Stashed changes
        except Exception as e:
            rp.logerr("%s - lost connectivity with rover", self.name)
            rp.logerr(e)
<<<<<<< Updated upstream
=======
            return False
>>>>>>> Stashed changes

    def run(self):
        while not rp.is_shutdown():
<<<<<<< Updated upstream
            self.send('.') # Satisfy watchdog
=======
            #self.send('!w') # Satisfy watchdog
>>>>>>> Stashed changes
            self.rate.sleep()

if __name__ == '__main__':
    unode = UnoNode()
    unode.run()
