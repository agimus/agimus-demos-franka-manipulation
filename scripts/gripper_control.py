#!/usr/bin/python

# Copyright 2022 CNRS
# Authors: Florent Lamiraux
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import actionlib
from std_msgs.msg import Int32
from dynamic_graph_bridge_msgs.msg import Vector
from franka_gripper.msg import GraspAction, MoveAction, GraspActionGoal, \
    MoveActionGoal

class GripperControl(object):

    def __init__(self):
        rospy.init_node ('gripper_control')
        self.sub_open = rospy.Subscriber('/agimus/sot/open_gripper',
                                         Vector, self.openGripper)
        self.sub_close = rospy.Subscriber('/agimus/sot/close_gripper',
                                          Vector, self.closeGripper)
        self.pub_success = rospy.Publisher('/agimus/gripper/feedback',
                                           Int32, queue_size = 1)
    def openGripper(self, msg):
        """
        Convert data from SoT signal to gripper command
        msg.data: vector with the following values
          - width,
          - speed.
        """
        # publish that action is starting
        self.pub_success.publish(0)
        client = actionlib.SimpleActionClient('/franka_gripper/move',
                                              MoveAction)
        res = client.wait_for_server()
        if not res:
            rospy.loginfo(
                "Failed to connect to action server '/franka_gripper/move'")
            self.pub_success.publish(-1)
        goal = MoveActionGoal()
        goal.goal.width = msg.data[0]
        goal.goal.speed = msg.data[1]
        client.send_goal(goal.goal)
        res = client.wait_for_result()
        if res:
            # publish success
            self.pub_success.publish(1)
        else:
            # publish failure
            self.pub_success.publish(-1)

    def closeGripper(self,msg):
        """
        Convert data from SoT signal to gripper command
        msg.data: vector with the following values
          - width,
          - speed.
        """
        self.pub_success.publish(0)
        client = actionlib.SimpleActionClient('/franka_gripper/grasp',
                                              GraspAction)
        res = client.wait_for_server()
        if not res:
            rospy.loginfo(
                "Failed to connect to action server '/franka_gripper/grasp'")
            self.pub_success.publish(-1)

        goal = GraspActionGoal()
        goal.goal.width = msg.data[0]
        goal.goal.epsilon.inner = msg.data[1]
        goal.goal.epsilon.outer = msg.data[2]
        goal.goal.speed = msg.data[3]
        goal.goal.force = msg.data[4]
        client.send_goal(goal.goal)
        res = client.wait_for_result()
        if res:
            # publish success
            self.pub_success.publish(1)
        else:
            # publish failure
            self.pub_success.publish(-1)
    
try:
    gc = GripperControl()
    rospy.spin()
except rospy.ROSInterruptException:
    pass
