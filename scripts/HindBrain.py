#!/usr/bin/env python

import serial
import rospy as rp
import sys
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from helper_functions import *
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
        self.rate = rp.Rate(3)
        self.name = 'hb'

        # Attributes to check for repeat msgs
        self.prev_ack_cmd =''
        self.prev_estop_cmd=''
        self.prev_pose_cmd=''

        self.vel_range = (-1,0,1)
        self.steer_range = (-45,0,45)
        self.z_range = (-0.3,0,0.3)
        self.pitch_range = (-45,0,45)
        self.msg_range = (0,50,100)

        if not self.serial_setup(): rp.signal_shutdown('Serial port connection failure')

    def ackermann_cb(self, msg):
        """callback function for ackermann messages

        Converts ackermann msg into a serial msg, sends if msg is different than
        last msg sent

        :param[in] msg: ackermann msg provided by ROS subscriber
        :param[out] self.prev_command: Updates previous command attribute
        """

        speed_cmd = cmd2msg(msg.speed, self.vel_range, self.msg_range)
        steer_cmd = cmd2msg(msg.steering_angle, self.steer_range, self.msg_range)
        command = "!a:" + str(int(speed_cmd)) + ":" + str(int(steer_cmd)) + ":\n"
        if (command != self.prev_ack_cmd):
            self.send(command)
            self.prev_ack_cmd = command

    def estop_cb(self, msg):
        """callback function for estop messages

        Converts estop msg into serial msg, sends if msg is different than
        last msg sent

        :param[in] msg: boolean msg provided by ROS subscriber
        :param[out] self.prev_command: Updates previous command attribute
        """
        command = "!e:" + str(int(msg.data)) + ":\n"
        if (command != self.prev_estop_cmd):
            self.send(command)
            self.prev_estop_cmd = command

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
        if (command != self.prev_pose_cmd):
            self.send(command)
            self.prev_pose_cmd = command

    def serial_setup(self):
        """Setup serial port

        Read /hind_brain/port parameter from rosserver, attempt to connect to
        obtained port, log success/failure to screen

        :param[out] self.uno_port: Creates Serial object on port
        :return bool: True if success, else False
        """
        try:
            port = rp.get_param('/hind_brain/port')
            self.uno_port = serial.Serial(port, 9600, write_timeout=1)
            self.uno_port.close()
            self.uno_port.open()
            rp.loginfo("%s - successfully set up serial communication on port %s", self.name, port)
            return True

        except KeyError as e:
            rp.logerr("%s - Cannot find /hind_brain/port param - did you use the launch file?", self.name)
            return False

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

    def read(self):
        """shows incoming serial msgs

        Checks serial port for msg, prints msg to terminal if exists

        :return:  bool if info on serial port
        """
        bytes_waiting = self.uno_port.inWaiting()
        read_val = self.uno_port.read(size=bytes_waiting)
        if read_val is not '':
            rp.loginfo(read_val)
            return True
        else: return False

    def run(self):
        """runs node mainloop

        sends watchdog msg at rate attr. frequency if ros is ok
        """
        while not rp.is_shutdown():
            self.read()
            self.send('!w:\n') # Satisfy watchdog
            self.rate.sleep()

if __name__ == '__main__':
    unode = UnoNode()
    unode.run()
