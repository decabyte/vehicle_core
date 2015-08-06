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

from __future__ import division

import math
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('vehicle_core')

import tf.transformations as tft

from vehicle_core.path import trajectory_tools as tt

from auv_msgs.msg import NavSts
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import PoseStamped, WrenchStamped
from visualization_msgs.msg import Marker, MarkerArray

from vehicle_interface.msg import PathRequest, PathStatus, PilotRequest, PilotStatus, Vector6Stamped, Vector6

# topics
TOPIC_NAV = 'nav/nav_sts'
TOPIC_PILOT_POS = 'pilot/position_req'
TOPIC_PILOT_STS = 'pilot/status'
TOPIC_FORCES = 'pilot/forces'
TOPIC_WRENCH = 'pilot/wrench'

TOPIC_PATH_REQ = 'path/request'
TOPIC_PATH_STS = 'path/status'

TOPIC_POSE2D = '/move_base_simple/goal'
TOPIC_MARKER = 'hmi/marker'

# modes
MODE_POSITION = 'position'
MODE_PATH = 'path'

# config
DEFAULT_SPACING = 4.0       # meters
DEFAULT_PROXIMITY = 8.0     # meters
DEFAULT_REFRESH = 0.5       # seconds

TEXT_STATUS = """
Pilot: {} ({})
Path: {} ({})
"""


class RVizInterface(object):
    def __init__(self, name, **kwargs):
        self.name = name

        # internal state
        self.mode = MODE_PATH
        self.pos = np.zeros(6)
        self.vel = np.zeros(6)

        self.status_pilot = None
        self.status_path = None

        # ros interface
        self.sub_poses = rospy.Subscriber(TOPIC_POSE2D, PoseStamped, self.handle_poses, queue_size=1)
        self.pub_marker = rospy.Publisher(TOPIC_MARKER, Marker, queue_size=1)

        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_nav, queue_size=10)
        self.sub_pilot = rospy.Subscriber(TOPIC_PILOT_STS, PilotStatus, self.handle_pilot_status, queue_size=1)
        self.sub_path = rospy.Subscriber(TOPIC_PATH_STS, PathStatus, self.handle_path_status, queue_size=1)

        #self.pub_wrench = rospy.Publisher(TOPIC_WRENCH, WrenchStamped, queue_size=1)
        #self.sub_forces = rospy.Subscriber(TOPIC_FORCES, Vector6Stamped, self.handle_forces, queue_size=1, tcp_nodelay=True)

        self.pub_pilot = rospy.Publisher(TOPIC_PILOT_POS, PilotRequest, queue_size=1)
        self.pub_path = rospy.Publisher(TOPIC_PATH_REQ, PathRequest, queue_size=1)

        # timers
        self.t_hmi = rospy.Timer(rospy.Duration(DEFAULT_REFRESH), self.send_text_marker)

        # user log
        rospy.loginfo('%s: started in %s mode ...', self.name, self.mode)


    def handle_nav(self, data):
        # parse navigation data
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

    def handle_pilot_status(self, data):
        self.status_pilot = data

    def handle_path_status(self, data):
        self.status_path = data

    # def handle_forces(self, data):
    #     # send wrench for rviz visualizer
    #     ws = WrenchStamped()
    #     ws.header.stamp = rospy.Time.now()
    #     ws.header.frame_id = 'base_link'
    #     ws.wrench.force.x = data.values[0]
    #     ws.wrench.force.y = data.values[1]
    #     ws.wrench.force.z = data.values[2]
    #     ws.wrench.torque.x = data.values[3]
    #     ws.wrench.torque.y = data.values[4]
    #     ws.wrench.torque.z = data.values[5]
    #
    #     self.pub_wrench.publish(ws)

    def handle_poses(self, data):
        # parse data
        pose = np.zeros(6)

        # transform quaternions to euler angles
        quat = np.zeros(4)
        quat[0] = data.pose.orientation.x
        quat[1] = data.pose.orientation.y
        quat[2] = data.pose.orientation.z
        quat[3] = data.pose.orientation.w
        orientation = tft.euler_from_quaternion(quat)

        # conversion from XYZ to NED
        pose[0] = data.pose.position.x
        pose[1] = -data.pose.position.y
        pose[2] = -data.pose.position.z
        pose[3] = orientation[0]
        pose[4] = -orientation[1]
        pose[5] = -orientation[2]

        if self.mode == MODE_POSITION:
            self.send_position_req(pose)
        else:
            self.send_path_req(pose)


    def send_position_req(self, position):
        # user log
        rospy.loginfo('%s: sending position request: %s', self.name, position)

        # send pilot request
        msg = PilotRequest()
        msg.header.stamp = rospy.Time.now()
        msg.position = position
        self.pub_pilot.publish(msg)

    def send_path_req(self, goal):
        # path info
        distance = tt.distance_between(self.pos, goal)

        if distance <= DEFAULT_PROXIMITY:
            # generate linear path
            mode = 'lines'
            wps = tt.interpolate_leg(self.pos, goal, face_goal=True, spacing=DEFAULT_SPACING, dimensions=2)
        else:
            # generate smooth path
            mode = 'fast'

            p1 = (6.0, self.pos[5])
            p2 = (6.0, goal[5])
            steps = max(math.floor(distance / DEFAULT_SPACING), 100)

            points = tt.format_bezier_input(self.pos, p1, p2, goal, degrees=False)
            wps = tt.interpolate_bezier_cubic(points, steps=steps)

        # user log
        rospy.loginfo('%s: sending %s path request: %s', self.name, mode, goal)

        # send new path
        msg = PathRequest()
        msg.header.stamp = rospy.Time.now()
        msg.command = 'path'
        msg.points = [Vector6(wp) for wp in wps]
        msg.options = [
            KeyValue('mode', mode),
            KeyValue('target_speed', '0.75'),
            KeyValue('look_ahead', '5.0'),
        ]

        self.pub_path.publish(msg)

    def send_text_marker(self, event=None):
        if self.status_pilot is None or self.status_path is None:
            return

        mm = Marker()
        mm.header.stamp = rospy.Time.now()
        mm.header.frame_id = 'map'
        mm.ns = 'hmi'
        mm.id = 0
        mm.action = Marker.ADD
        mm.lifetime = rospy.Duration(1, 0)

        mm.type = Marker.TEXT_VIEW_FACING
        mm.scale.z = 0.20
        mm.color.r = 1.0
        mm.color.g = 1.0
        mm.color.b = 1.0
        mm.color.a = 1.0

        mm.pose.position.x = self.pos[0]
        mm.pose.position.y = -(self.pos[1])
        mm.pose.position.z = -(self.pos[2] + 1.0)

        mm.text = TEXT_STATUS.format(
            self.status_pilot.status, self.status_pilot.mode,
            self.status_path.path_status, self.status_path.navigation_status
        )

        self.pub_marker.publish(mm)


if __name__ == '__main__':
    rospy.init_node('rviz_interface')

    #topic_input = rospy.get_param('~topic_input', TOPIC_CMDS)
    #config = rospy.get_param('rviz/interface', dict())

    rospy.loginfo('%s: node init ... ', rospy.get_name())
    thruster = RVizInterface(rospy.get_name())

    rospy.spin()
