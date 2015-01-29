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

from vehicle_core.config import  thrusters_config as tc
from vehicle_core.model import thruster_model as tm
from vehicle_core.model import throttle_model as th

import rospy
import roslib
roslib.load_manifest('vehicle_core')

from vehicle_interface.msg import ThrusterCommand, ThrusterFeedback, Vector6Stamped

# topics
TOPIC_MODEL = 'thrusters/model'
TOPIC_CMDS = 'thrusters/commands'
TOPIC_FORCES = 'forces/body'


class SimulatedThrusters(object):
    def __init__(self, name, topic_input, topic_feedback, topic_forces, thruster_limit, **kwargs):
        self.name = name

        # data arrays
        self.throttle_request = np.zeros((6, tc.LPF_WINDOW))
        self.throttle_predicted = np.zeros(6)
        self.throttle_last = np.zeros(6)
        self.current_predicted = np.zeros(6)
        self.forces_predicted = np.zeros(6)
        self.body_forces = np.zeros(6)
        self.throttle_limit = thruster_limit

        self.limit_rate = bool(kwargs.get('limit_rate', False))
        self.rising_limit = float(kwargs.get('rising_limit', tc.THROTTLE_RISING_LIMIT))
        self.falling_limit = float(kwargs.get('falling_limit', tc.THROTTLE_FALLING_LIMIT))
        self.model_delay = int(kwargs.get('model_delay', 0))

        self.rising_limit = np.clip(self.rising_limit, 0, 100)
        self.falling_limit = np.clip(self.falling_limit, 0, 100)
        self.model_delay  = np.clip(self.model_delay, 0, tc.LPF_WINDOW-1)

        # subscribers
        self.sub_cmd = rospy.Subscriber(topic_input, ThrusterCommand, self.handle_commands, tcp_nodelay=True, queue_size=10)
        self.pub_feedback = rospy.Publisher(topic_feedback, ThrusterFeedback, tcp_nodelay=True, queue_size=10)
        self.pub_forces = rospy.Publisher(topic_forces, Vector6Stamped, tcp_nodelay=True, queue_size=10)


    def handle_commands(self, data):
        self.throttle_request[:, -1] = np.array(data.throttle[0:6])

        # apply thruster model (current estimation)
        self.throttle_last = np.copy(self.throttle_predicted)
        self.throttle_predicted = th.predict_throttle(
            self.throttle_request, b=tc.LPF[0], a=tc.LPF[1], offset=self.model_delay, limit=self.throttle_limit
        )

        if self.limit_rate is True:
            # new thrusters filter
            self.throttle_predicted = th.rate_limiter(
                self.throttle_predicted, self.throttle_last, rising_limit=self.rising_limit, falling_limit=self.falling_limit
            )

        self.current_predicted = tm.estimate_current(self.throttle_predicted, tc.THROTTLE_TO_CURRENT)

        # apply thruster model (thrust estimation)
        self.forces_predicted = tm.estimate_forces( self.throttle_predicted, self.current_predicted, tc.CURRENT_TO_THRUST )

        # converting from thruster domain to body forces using the thruster allocation matrix
        self.forces_predicted = self.forces_predicted.reshape(6, 1)
        self.body_forces = np.dot(tc.TAM, self.forces_predicted)

        # zero the array in case the messages are not received
        self.throttle_request = np.roll(self.throttle_request, -1, axis=1)
        self.throttle_request[:, -1] = np.zeros(6)

        # send thruster feedback
        msg = ThrusterFeedback()
        msg.header.stamp = rospy.Time.now()
        msg.throttle = self.throttle_predicted
        msg.current = self.current_predicted
        msg.temp = np.zeros(6)
        msg.status = np.zeros(6)
        msg.errors = np.zeros(6)
        self.pub_feedback.publish(msg)

        # send body forces
        msg = Vector6Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.values = self.body_forces
        self.pub_forces.publish(msg)


if __name__ == '__main__':
    rospy.init_node('thrusters_model')

    topic_input = rospy.get_param('~topic_input', TOPIC_CMDS)
    topic_feedback = rospy.get_param('~topic_feedback', TOPIC_MODEL)
    topic_forces = rospy.get_param('~topic_forces', TOPIC_FORCES)
    lim = int(rospy.get_param('thrusters/throttle_limit', tc.MAX_THROTTLE))
    lim = int(np.clip(lim, 0, 100).astype(int))

    config = rospy.get_param('thruster_model', dict())

    rospy.loginfo('%s: model init ... ', rospy.get_name())
    rospy.loginfo('%s: thrusters input topic: %s', rospy.get_name(), topic_input)
    rospy.loginfo('%s: thrusters feedback topic: %s', rospy.get_name(), topic_feedback)
    rospy.loginfo('%s: thrusters forces topic: %s', rospy.get_name(), topic_forces)
    rospy.loginfo('%s: thrusters throttle limit: %s', rospy.get_name(), lim)
    rospy.loginfo('%s: thrusters simulator config:\n%s', rospy.get_name(), config)

    thruster = SimulatedThrusters(rospy.get_name(), topic_input, topic_feedback, topic_forces, lim, **config)
    rospy.spin()
