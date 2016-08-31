#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import copy
import actionlib
import rospy
import tf
from math import sin, cos

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        self._is_ar_marker = False
        self._ar_marker_pose = []
        self._listener = tf.TransformListener()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()
        
        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

    def is_ar_marker(self):
        # Get current pose.
        try:
            (robot_trans, rot) = self._listener.lookupTransform('/map',
                                                                '/base_link',
                                                                rospy.Time(0))
            angles = [0, 1.57, 3.14, -1.57]
            for angle in angles:
                self.goto(robot_trans[0], robot_trans[1], angle)
                try:
                    (ar_trans,rot) = self._listener.lookupTransform('/map',
                                                                    '/ar_marker_0',
                                                                    rospy.Time(0))
                    self._is_ar_marker = True
                    self._ar_marker_pose = ar_trans
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    self._is_ar_marker = False
                    continue

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self._is_ar_marker = False


    def goto_ar_marker(self):
        if self._is_ar_marker:
            self.goto(self._ar_marker_pose[0],
                      self._ar_marker_pose[1],
                      0)

if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()

    # Move the base to be in front of the table
    # Demonstrates the use of the navigation stack
    # rospy.loginfo("Moving to table...")
    point_near_table = [4.31071949005, 0.690907716751]
    point_near_intersection = [3.65533733368, -0.7061499382]
    point_near_table = [2.30365657806, -0.100080490112]
    point_under_table = [0.673938751221, 0.430013656616]

    waypoints = []
    waypoints.append(point_near_table)
    waypoints.append(point_near_intersection)
    waypoints.append(point_near_table)
    waypoints.append(point_under_table)

    for waypoint in waypoints:
        # Goto waypoint.
        move_base.goto(waypoint[0], waypoint[1], 0.0)
        # Do spin.
        if move_base.is_ar_marker():
            move_base.goto_ar_marker()
            break

