#!/usr/bin/python
# -*- coding:utf-8 -*-
import time
import sys
import rospy
import io_interface as io
from ur_msgs.srv import *
from ur_msgs.msg import *
GRIPPER_DIGITAL_OUT = 4

class Gripper:
    def __init__(self,service):
        self.digit_pin = GRIPPER_DIGITAL_OUT
        io.set_states()
    def close(self):
        io.set_digital_out(self.digit_pin,True)
        time.sleep(2)
    def open(self):
        io.set_digital_out(self.digit_pin,False)
        time.sleep(2)

if __name__ == "__main__":
    gripper = Gripper('ur_hardware_interface/set_io')
    while True:
        gripper.open()
        gripper.close()