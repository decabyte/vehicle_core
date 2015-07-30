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

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('vehicle_core')

import vehicle_core.model.thruster_model as tm
import vehicle_core.config.thrusters_config as tc

from sensor_msgs.msg import Joy
from vehicle_interface.msg import ThrusterCommand, Vector6Stamped, PilotRequest
from vehicle_interface.srv import BooleanService


# config
TOPIC_JOY = 'joy'
TOPIC_CMD = 'thrusters/commands'
TOPIC_FORCES = 'user/forces'
TOPIC_STAY = 'pilot/body_req'
SRV_THRUSTERS = 'thrusters/switch'
SRV_CONTROLLER = 'pilot/switch'

# joystick sensitivity mapping
#   use 0.1 for a linear response
#   use 0.5 for a mild exponential response
#   use 1.0 for a heavy exponential response
K_EXP = 0.5

# thrust standard request
#   in default operation the joystick interface request only a fraction of maximum thrust
#   this is helping in small tank operations and it produces only 65% of full thrust
#   use the boost mode to squeeze all the juice out of the vehicle! :)
K_REDUCED = 0.65


AXIS = {
    'x': 1,     # left stick up/down
    'y': 0,     # left stich left/right
    'z': 3,     # right stick up/down
    'pitch': 1,   # left stick up/down
    'yaw': 2,    # right stick left/right
}

BTNS = {
    'sw_stay': 0,       # square
    'sw_controller': 1, # cross
    'sw_thrusters': 2,  # circle
    'sw_mode': 3,       # triangle
    'depth': 7,         # R2
    'pitch': 6,         # L2
    'boost': 5          # R1
    #'': 4               # L1
}

MODE_FORCE = 'force'
MODE_SPEED = 'speed'
MODE_DIRECT = 'direct'

VEHICLE_AUV = 'auv'
VEHICLE_EMILY = 'emily'


class JoystickInterface(object):

    def __init__(self, name, speed_limit, topic_input, topic_throttle, topic_stay, topic_forces, srv_thrusters, srv_controller, vehicle, k_exp=K_EXP):
        self.name = name
        self.speed_limit = speed_limit
        self.input_topic = topic_input

        # joystick state
        self.mode = MODE_FORCE
        self.vehicle = vehicle
        self.k_exp = np.clip(k_exp, 0.1, 2)     # clip exponential term (zero will produce errors)

        self.mode_controller = False
        self.mode_thrusters = False

        self.btn_switch = np.zeros(5)
        self.btn_thrusters = np.zeros(5)
        self.btn_controller = np.zeros(5)
        self.btn_stay = np.zeros(5)

        # ros interface
        self.sub_joy = rospy.Subscriber(self.input_topic, Joy, self.handle_joystick, tcp_nodelay=True, queue_size=1)
        self.pub_thr = rospy.Publisher(topic_throttle, ThrusterCommand, tcp_nodelay=True, queue_size=1)
        self.pub_for = rospy.Publisher(TOPIC_FORCES, Vector6Stamped, tcp_nodelay=True, queue_size=1)
        self.pub_stay = rospy.Publisher(topic_stay, PilotRequest, tcp_nodelay=True, queue_size=1)

        # services
        self.srv_thrusters = rospy.ServiceProxy(srv_thrusters, BooleanService)
        self.srv_controller = rospy.ServiceProxy(srv_controller, BooleanService)

        # rosinfo
        rospy.loginfo('%s started in mode: %s', self.name, self.mode)


    def handle_joystick(self, data):
        # parse joystick
        surge = data.axes[AXIS['x']]            # normal joystick axis
        sway = -1 * data.axes[AXIS['y']]        # reverse joystick axis
        heave = -1 * data.axes[AXIS['z']]       # normal joystick axis

        roll = 0                                # not used
        pitch = data.axes[AXIS['pitch']]        # normal joystick axis
        yaw = -1 * data.axes[AXIS['yaw']]       # normal joystick axis

        # parse buttons
        self.btn_switch[-1] = data.buttons[BTNS['sw_mode']]
        self.btn_thrusters[-1] = data.buttons[BTNS['sw_thrusters']]
        self.btn_controller[-1] = data.buttons[BTNS['sw_controller']]
        self.btn_stay[-1] = data.buttons[BTNS['sw_stay']]


        # ask the current position to the pilot
        if self.btn_stay[-1] == 1 and self.btn_stay[-2] == 0:
            try:
                pr = PilotRequest()
                pr.header.stamp = rospy.Time.now()
                pr.priority = PilotRequest.HIGH
                pr.position = np.zeros(6)

                self.pub_stay.publish(pr)
                rospy.loginfo('%s requesting position hold', self.name)
            except Exception:
                rospy.logerr('%s: publish error', self.name)

        self.btn_stay = np.roll(self.btn_stay, -1)


        # switch controller (on/off)
        if self.btn_controller[-1] == 1 and self.btn_controller[-2] == 0:
            self.mode_controller = not self.mode_controller
            try:
                response = self.srv_controller.call(self.mode_controller)
                rospy.loginfo('%s switching controller: %s', self.name, response)
            except rospy.ServiceException:
                rospy.logerr('%s: controller service error', self.name)

        self.btn_controller = np.roll(self.btn_controller, -1)


        # switch thrusters (on/off)
        if self.btn_thrusters[-1] == 1 and self.btn_thrusters[-2] == 0:
            self.mode_thrusters = not self.mode_thrusters
            try:
                response = self.srv_thrusters.call(self.mode_thrusters)
                rospy.loginfo('%s switching thrusters: %s', self.name, response)
            except rospy.ServiceException:
                rospy.logerr('%s: thrusters service error', self.name)

        self.btn_thrusters = np.roll(self.btn_thrusters, -1)


        # # switch joystick mode
        # if self.btn_switch[-1] == 1 and self.btn_switch[-2] == 0:
        #     if self.mode == MODE_FORCE:
        #         self.mode = MODE_DIRECT
        #     else:
        #         self.mode = MODE_FORCE
        #
        #     rospy.loginfo('%s joystick now in mode: %s', self.name, self.mode)
        #
        # self.btn_switch = np.roll(self.btn_switch, -1)


        # parse buttons and select joystick mode
        depth_mode = data.buttons[BTNS['depth']]
        pitch_mode = data.buttons[BTNS['pitch']]
        boost_mode = data.buttons[BTNS['boost']]

        if depth_mode == 1:
            yaw = 0
        else:
            heave = 0

        if pitch_mode == 1:
            surge = 0
        else:
            pitch = 0

        if boost_mode == 1:
            k_scaling = 1               # request full thrust
        else:
            k_scaling = K_REDUCED       # request only a fraction of max thrust

        if self.vehicle == VEHICLE_AUV:
            # process joystick input
            forces = k_scaling * np.array([surge, sway, heave, 0, pitch, yaw])

            # exponential mapping
            forces = np.sign(forces) * (np.exp(self.k_exp * np.abs(forces)) - 1) / (np.exp(self.k_exp) - 1)

            forces = np.clip(forces, -1, 1)

            # forces clipping
            forces *= tc.MAX_U

            # FORCES command mode default
            uf = Vector6Stamped()
            uf.header.stamp = rospy.Time.now()
            uf.values = forces.flatten().tolist()
            self.pub_for.publish(uf)

        elif self.vehicle == VEHICLE_EMILY:
            # send only if controller is enabled
            if not self.mode_controller:
                # map joystick to Emily's controls
                throttle = np.array([surge, -yaw, 0, 0, 0, 0])

                # exponential mapping
                throttle = np.sign(throttle) * (np.exp(self.k_exp * np.abs(throttle)) - 1) / (np.exp(self.k_exp) - 1)

                throttle = np.clip(throttle, -1, 1)
                # forward thrust greater than 0
                throttle[0] = np.maximum(throttle[0], 0)

                # scale up the range
                throttle *= 100

                thc = ThrusterCommand()
                thc.header.stamp = rospy.Time.now()
                thc.throttle = throttle
                self.pub_thr.publish(thc)

        # # MULTIPLE MODES (this should be disabled after initial testing and debugging)
        # # FORCES command mode
        # #   send forces to the pilot
        # if self.mode == MODE_FORCE:
        #     uf = Vector6Stamped()
        #     uf.header.stamp = rospy.Time.now()
        #     uf.values = forces.flatten().tolist()
        #     self.pub_for.publish(uf)
        #
        # # DIRECT command mode
        # #   bypass the pilot and sends direct requests to thruster driver
        # #   this is using the linearization model to transform forces to throttle
        # #   and the thruster allocation matrix to distribute forces on vehicle axis
        # elif self.mode == MODE_DIRECT:
        #     # throttle mapping using the linearization and limits
        #     tau = np.dot(tc.inv_TAM, forces)
        #     throttle = tm.estimate_throttle(tau, tc.THRUST_TO_THROTTLE, tc.LINEAR_THROTTLE, tc.THRUST_THRESHOLD, tc.MAX_FORCE)
        #     throttle = np.clip(throttle, -self.speed_limit, self.speed_limit)
        #
        #     # send throttle
        #     thc = ThrusterCommand()
        #     thc.header.stamp = rospy.Time.now()
        #     thc.throttle = throttle.astype(int).flatten().tolist()
        #     self.pub_thr.publish(thc)
        #
        # # SPEED command mode
        # #   bypass the pilot and sends direct requests to thruster driver
        # #   by applying a simple transformation (maybe useful for testing)
        # elif self.mode == MODE_SPEED:
        #     lat_rear = sway + -(yaw)
        #     lat_front = sway + yaw
        #
        #     # create the speed vector
        #     throttle = np.array([surge, surge, lat_rear, lat_front, heave, heave]) * 100
        #     throttle = np.clip(throttle, -self.speed_limit, self.speed_limit)
        #
        #     # send throttle
        #     thc = ThrusterCommand()
        #     thc.header.stamp = rospy.Time.now()
        #     thc.throttle = throttle.astype(int).flatten().tolist()
        #     self.pub_thr.publish(thc)
        #
        # else:
        #     pass


if __name__ == '__main__':
    rospy.init_node('node_joystick')
    rospy.loginfo('%s initializing ...', rospy.get_name())

    # load parameters
    throttle_limit = int(rospy.get_param('thrusters/throttle_limit', tc.MAX_THROTTLE))
    topic_input = rospy.get_param('~input_topic', TOPIC_JOY)
    topic_throttle = rospy.get_param('~topic_throttle', TOPIC_CMD)
    topic_stay = rospy.get_param('~topic_stay', TOPIC_STAY)
    topic_forces = rospy.get_param('~topic_forces', TOPIC_FORCES)
    srv_thrusters = rospy.get_param('~srv_thrusters', SRV_THRUSTERS)
    srv_controller = rospy.get_param('~srv_controller', SRV_CONTROLLER)
    vehicle = rospy.get_param('~vehicle', VEHICLE_AUV)
    wait_services = bool(rospy.get_param('~wait_services', False))

    # check valid limit
    throttle_limit = np.clip(throttle_limit, -100, 100)

    # console output
    rospy.loginfo('%s throttle limit: %d%%', rospy.get_name(), throttle_limit)
    rospy.loginfo('%s topic input: %s', rospy.get_name(), topic_input)
    rospy.loginfo('%s topic throttle: %s', rospy.get_name(), topic_throttle)
    rospy.loginfo('%s topic stay: %s', rospy.get_name(), topic_stay)
    rospy.loginfo('%s topic forces: %s', rospy.get_name(), topic_forces)
    rospy.loginfo('%s service thrusters: %s', rospy.get_name(), srv_thrusters)
    rospy.loginfo('%s service controller: %s', rospy.get_name(), srv_controller)
    rospy.loginfo('%s vehicle: %s', rospy.get_name(), vehicle)
    rospy.loginfo('%s wait services: %s', rospy.get_name(), wait_services)

    if wait_services:
        try:
            rospy.wait_for_service(SRV_THRUSTERS, timeout=1)
            rospy.wait_for_service(SRV_CONTROLLER, timeout=1)
        except rospy.ROSException as re:
            rospy.logerr('%s: services not available: %s', rospy.get_name(), re)

    js = JoystickInterface(rospy.get_name(), throttle_limit, topic_input, topic_throttle, topic_stay, topic_forces,
                           srv_thrusters, srv_controller, vehicle)

    rospy.spin()
