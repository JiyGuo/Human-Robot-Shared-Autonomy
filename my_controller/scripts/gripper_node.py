#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from gripper import Gripper
from std_msgs.msg import String
from ur_msgs.srv import *
from ur_msgs.msg import *
class GripperNode:
    def __init__(self,service='ur_hardware_interface/set_io'):
        rospy.init_node('gripper_node', anonymous=True)
        self.subscriber = rospy.Subscriber('/gripper/command', String, self.callback, queue_size=1)
        self.gripper = Gripper(service)
        print('Gripper node is running')
        rospy.spin()
    def callback(self, data):
        if data.data == 'open':
            self.gripper.open()
        elif data.data == 'close':
            self.gripper.close()
        else:
            rospy.ERROR("Command should input 'open' or 'close'")

if __name__ == '__main__':
    node = GripperNode()