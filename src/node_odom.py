#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, Ocean Systems Laboratory, Heriot-Watt University, UK.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Heriot-Watt University nor the names of
#     its contributors may be used to endorse or promote products
#     derived from this software without specific prior written
#     permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  Original authors:
#   Valerio De Carolis, Marian Andrecki, Corina Barbalata, Gordon Frost

"""Navigation Odometry

    This node processes the nav data and publish the ROS odometry message together with TF broadcasting to make
    the vehicle working with RVIZ visualizations.
"""

from __future__ import division

import numpy as np
import traceback

import rospy
import roslib
roslib.load_manifest('vehicle_core')

import tf
import tf.transformations as tft

# Odometry related (needs nav_msgs and geometry_msgs)
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, TwistWithCovariance, Twist, Vector3

# Nessie VII related (previous software stack, also needs geometry_msgs)
from auv_msgs.msg import NavSts, NED, RPY


# CONFIG
TOPIC_NAV = 'nav/nav_sts'
TOPIC_ODM = 'nav/odometry'
FRAME_PARENT = 'odom'
FRAME_CHILD = 'base_link'

# REFERENCES:
#	to publish the odometry this node needs to broadcast the tf tranform and then publish the odometry message
#
#	[1]: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
#	[2]: http://wiki.ros.org/navigation/Tutorials
#	[3]: http://wiki.ros.org/tf/Overview/Transformations


class OdometryNode(object):

    def __init__(self, name, topic_in, topic_out, frame_parent, frame_child):
        self.name = name

        # config
        self.topic_in = topic_in
        self.topic_out = topic_out
        self.frame_parent = frame_parent
        self.frame_child = frame_child

        # tf related
        self.br = tf.TransformBroadcaster()

        # ros interface
        self.sub_nav = rospy.Subscriber(self.topic_in, NavSts, self.handle_nav, tcp_nodelay=True, queue_size=1)
        self.pub_odom = rospy.Publisher(self.topic_out, Odometry, tcp_nodelay=True, queue_size=1)

        # convert to xyz
        # c = np.cos(np.deg2rad(180))
        # s = np.sin(np.deg2rad(180))
        #
        # rot_mat = np.array([
        #     [c, s, 0, 0, 0, 0],
        #     [-s, c, 0, 0, 0, 0],
        #     [0, 0, 1, 0, 0, 0],
        #     [0, 0, 0, c, s, 0],
        #     [0, 0, 0, -s, c, 0],
        #     [0, 0, 0, 0, 0, 1],
        # ])

        rot_mat_z = np.zeros((6,6))
        rot_mat_z[0:3, 0:3] = tft.rotation_matrix(np.deg2rad(180), np.array([0, 0, 1]))[:3, :3]
        rot_mat_z[3:6, 3:6] = tft.rotation_matrix(np.deg2rad(180), np.array([0, 0, 1]))[:3, :3]

        rot_mat_y = np.zeros((6,6))
        rot_mat_y[0:3, 0:3] = tft.rotation_matrix(np.deg2rad(180), np.array([0, 1, 0]))[:3, :3]
        rot_mat_y[3:6, 3:6] = tft.rotation_matrix(np.deg2rad(180), np.array([0, 1, 0]))[:3, :3]

        # new rotation matrix
        self.rot_mat = np.dot( rot_mat_z, rot_mat_y )




    def handle_nav(self, data):
        # parse data
        self.pos = np.array([
            data.position.north,
            data.position.east,
            data.position.depth,
            data.orientation.roll,
            data.orientation.pitch,
            data.orientation.yaw
        ])

        self.vel = np.array([
            data.body_velocity.x,
            data.body_velocity.y,
            data.body_velocity.z,
            data.orientation_rate.roll,
            data.orientation_rate.pitch,
            data.orientation_rate.yaw
        ])

        # convert to xyz
        self.pos = np.dot( self.rot_mat, self.pos.reshape((6,1)) ).flatten()
        self.vel = np.dot( self.rot_mat, self.vel.reshape((6,1)) ).flatten()

        # tf broadcast
        translation = (self.pos[0], self.pos[1], self.pos[2])
        rotation = tft.quaternion_from_euler(self.pos[3], self.pos[4], self.pos[5], axes='sxyz')

        self.br.sendTransform(translation, rotation, rospy.Time.now(), self.frame_child, self.frame_parent)

        # odometry message
        od = Odometry()
        od.header.stamp = rospy.Time.now()
        od.header.frame_id = self.frame_parent
        od.child_frame_id = self.frame_child

        od.pose.pose.position.x = self.pos[0]		# north
        od.pose.pose.position.y = self.pos[1]		# east
        od.pose.pose.position.z = self.pos[2]		# depth

        od.pose.pose.orientation.x = rotation[0]
        od.pose.pose.orientation.y = rotation[1]
        od.pose.pose.orientation.z = rotation[2]
        od.pose.pose.orientation.w = rotation[3]

        od.twist.twist.linear.x = self.vel[0]
        od.twist.twist.linear.y = self.vel[1]
        od.twist.twist.linear.z = self.vel[2]
        od.twist.twist.angular.x = self.vel[3]
        od.twist.twist.angular.y = self.vel[4]
        od.twist.twist.angular.z = self.vel[5]

        self.pub_odom.publish(od)



def main():
    rospy.init_node('nav_odometry')
    name = rospy.get_name()
    rospy.loginfo('%s initializing ...', name)

    # load parameters
    topic_input = rospy.get_param('~input', TOPIC_NAV)
    topic_output = rospy.get_param('~output', TOPIC_ODM)
    frame_parent = rospy.get_param('~parent', FRAME_PARENT)
    frame_child = rospy.get_param('~child', FRAME_CHILD)

    # console info
    rospy.loginfo('%s: input topic: %s', name, topic_input)
    rospy.loginfo('%s: output topic: %s', name, topic_output)
    rospy.loginfo('%s: parent frame: %s', name, frame_parent)
    rospy.loginfo('%s: child frame: %s', name, frame_child)

    # start odometry node
    on = OdometryNode(name, topic_input, topic_output, frame_parent, frame_child)
    rospy.spin()


if __name__ == '__main__':
    main()