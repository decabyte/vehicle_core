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

"""This node wraps the NavigationSimulator providing a ROS interface that publish the navigation messages at a given rate.

It consumes a force input in body-frame coordinates and updates the position of the vehicle in the simulated environment.
The input forces can be generated using the thrusters_simulator provided in the vehicle_core package. This two aspects
have been separated during the implementation of the nav_sim node to reduce coupling and allow the implementation of a real
thruster simulator using the thruster model developed during experiments conducted in the OSL tank.
"""
from __future__ import division

import traceback
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('vehicle_core')

import tf
import tf.transformations as tft

from vehicle_core.sim import navigation as sim
from vehicle_core.util import conversions as cnv

from nav_msgs.msg import Odometry
from auv_msgs.msg import NavSts
from std_srvs.srv import Empty, EmptyResponse

from vehicle_interface.msg import FloatArrayStamped, Vector6Stamped
from vehicle_interface.srv import Vector6Service, Vector6ServiceResponse

# params
RATE_SIM = 20       # simulation rate (Hz) (this is limited by the current implementation)
RATE_PUB = 10       # publisher rate (Hz) (one should publish messages at a slower rate)

TOPIC_NAV = 'nav/nav_sts'
TOPIC_THR = 'thrusters/commands'
TOPIC_FRC = 'forces/sim/body'
TOPIC_NET = 'forces/sim/net'
TOPIC_WATER = 'nav/sim/water'
TOPIC_CURRENTS = 'nav/sim/currents'

SRV_RESET = 'nav/reset'
SRV_OFFSET = 'nav/offset'

TOPIC_ODM = 'nav/odometry'
FRAME_PARENT = 'world'
FRAME_ODOM = 'odom'
FRAME_CHILD = 'base_link'


class NodeSimulator(object):
    """NavigationSimulator is part of the vehicle_core module. it provides interfacing with the ROS system.

    This class handles force inputs and publish navigation messages simulating the behaviour of the navigation modules
    running inside the vehicle during real operations. It also offers extra topics useful for monitoring the vehicle during
    simulation, like the total net force acting on the vehicle or the status of the internal navigation simulator.

    References:
        [1] http://www.ros.org/reps/rep-0103.html
    """

    def __init__(self, name, sim_rate, pub_rate, **kwargs):
        self.name = name
        self.verbose = kwargs.get('verbose', False)

        # state
        self.dt = 1.0 / sim_rate
        self.tau = np.zeros(6, dtype=np.float64)                                # input forces (N)
        self.offset_pos = np.zeros(6, dtype=np.float64)                         # navigation offset (meters, rad)
        self.depth_bottom = kwargs.get('depth_bottom', sim.DEFAULT_SEA_DEPTH)   # sea bottom (meters)

        # initial conditions
        self.offset_pos = np.array(kwargs.get('pos', self.offset_pos.tolist()))
        rospy.loginfo('%s setting initial position to zero ...', self.name)

        # dynamic model
        self.model_config = rospy.get_param('sim/model', dict())

        # init navigation simulator
        self.navsim = sim.NavigationSimulator(self.dt, self.model_config, depth_bottom=self.depth_bottom)

        # topics
        self.input_forces = kwargs.get('input_forces', TOPIC_FRC)
        self.output_nav = kwargs.get('output_nav', TOPIC_NAV)
        self.output_forces = kwargs.get('output_forces', TOPIC_NET)

        # odometry
        self.br = tf.TransformBroadcaster()
        self.topic_odom = kwargs.get('topic_odom', TOPIC_ODM)
        self.frame_parent = kwargs.get('frame_parent', FRAME_PARENT)
        self.frame_odom = kwargs.get('frame_odom', FRAME_ODOM)
        self.frame_child = kwargs.get('frame_child', FRAME_CHILD)

        # odometry rotation matrix
        rot_mat_z = np.zeros((6,6), dtype=np.float64)
        rot_mat_z[0:3, 0:3] = tft.rotation_matrix(np.deg2rad(180), np.array([0, 0, 1]))[:3, :3]
        rot_mat_z[3:6, 3:6] = tft.rotation_matrix(np.deg2rad(180), np.array([0, 0, 1]))[:3, :3]

        rot_mat_y = np.zeros((6,6), dtype=np.float64)
        rot_mat_y[0:3, 0:3] = tft.rotation_matrix(np.deg2rad(180), np.array([0, 1, 0]))[:3, :3]
        rot_mat_y[3:6, 3:6] = tft.rotation_matrix(np.deg2rad(180), np.array([0, 1, 0]))[:3, :3]

        self.rot_mat = np.dot( rot_mat_z, rot_mat_y )

        # ros interface
        self.sub_forces = rospy.Subscriber(self.input_forces, Vector6Stamped, self.handle_forces, queue_size=1, tcp_nodelay=True)
        self.sub_water = rospy.Subscriber(TOPIC_WATER, FloatArrayStamped, self.handle_water, queue_size=1, tcp_nodelay=True)

        self.pub_navs = rospy.Publisher(self.output_nav, NavSts, queue_size=1, tcp_nodelay=True)
        self.pub_odom = rospy.Publisher(self.topic_odom, Odometry, queue_size=1, tcp_nodelay=True)
        self.pub_curr = rospy.Publisher(TOPIC_CURRENTS, FloatArrayStamped, queue_size=1, tcp_nodelay=True)
        #self.pub_force = rospy.Publisher(self.output_forces, WrenchStamped, queue_size=1, tcp_nodelay=True)

        self.srv_zero = rospy.Service(SRV_RESET, Empty, self.handle_reset)
        self.srv_offset = rospy.Service(SRV_OFFSET, Vector6Service, self.handle_offset)

        # timers
        self.r_loop = rospy.Rate(sim_rate)
        self.t_pub = rospy.Timer(rospy.Duration(1 / pub_rate), self.publish_navigation)

        # optional interfaces
        if self.verbose:
            self.t_pri = rospy.Timer(rospy.Duration(1), self.print_status)
            #self.t_net = rospy.Timer(rospy.Duration(1 / pub_rate), self.send_forces)

    def handle_reset(self, req):
        """Resets the status of the navigation simulator."""
        rospy.logwarn('%s: resetting the nav simulator ...', self.name)
        self.navsim.reset()

        return EmptyResponse()

    def handle_offset(self, req):
        """Sets the nav offset using the user request."""
        rospy.logwarn('%s: offsetting the nav origin with %s ...', self.name, self.offset_pos)
        self.offset_pos = np.array(req.request)

        return Vector6ServiceResponse(True, self.offset_pos)

    def handle_water(self, data):
        """Sets the water current using the user request. This assumes the data input to be an array of floats where:
            a[0] = water surface speed (maximum) (m/s)                      (default: 0.0)
            a[1] = water surface speed variance (m/s)^2                     (default: 0.001)
            a[2] = water surface speed process coeff [0.0, 1.0]             (default: 0.0)

            a[3] = water angle of attack (azimuth, radians)                 (default: 0.0)
            a[4] = water angle of attack variance (azimuth, radians^2)      (default: 0.001)
            a[5] = water angle of attack (elevation, radians)               (default: 0.0)
            a[6] = water angle of attack variance (elevation, radians^2)    (default: 0.001)
        """
        params = len(data.values)

        if params < 2:
            return

        v = np.clip(data.values[0], 0.0, sim.MAX_CURRENT)
        sigma_v = np.clip(data.values[1], 0.001, sim.MAX_CURRENT)
        mu = 0.0

        if params >= 3:
            mu = np.clip(data.values[2], 0.0, 1.0)

        # set default orientation if not provided
        b = 0.0
        sigma_b = 0.001
        a = 0.0
        sigma_a = 0.001

        if params >= 5:
            b = cnv.wrap_pi(data.values[3])
            sigma_b = np.clip(data.values[4], 0.001, np.pi)

        if params >= 7:
            a = cnv.wrap_pi(data.values[5])
            sigma_a = np.clip(data.values[6], 0.001, np.pi)

        rospy.loginfo('%s updating water current (v: %.3f, vs: %.3f, mu: %.3f, b: %.3f, bs: %.3f, a: %.3f, as: %.3f)',
                      self.name, v, sigma_v, mu, b, sigma_b, a, sigma_a)

        self.navsim.update_water_current(v, sigma_v, mu, b, sigma_b, a, sigma_a)


    def handle_forces(self, data):
        """Sets the forces acting on the vehicle (e.g. net force produced by thrusters)"""
        self.tau = np.array(data.values, dtype=np.float64)

    def publish_navigation(self, event=None):
        """This function is used for publishing sending data outside of this node.

        It applies the navigation offset, used for indoor operations without the use of the global positioning system.
        """
        # current state
        pos = np.copy(self.navsim.pos)
        vel = np.copy(self.navsim.vel)
        depth = self.navsim.depth_bottom

        # apply offsets (simple strategy)
        pos[0:3] += self.offset_pos[0:3]

        # send ROS messages
        self.send_nav_sts(pos, vel, depth)

        # send TF messages
        self.send_tf_odom(pos, vel)
        #self.send_tf_ned(pos, vel)

        # publish water currents
        self.send_currents()

    def print_status(self, event=None):
        print(self.navsim)

    def send_tf_ned(self, pos, vel):
        """This is publishing the correct TF transformation with respect to vehicle frame

        It broadcast the transform using the same convention NED used for the position and velocity vectors.
        """
        translation = (pos[0], pos[1], pos[2])
        rotation = tft.quaternion_from_euler(pos[3], pos[4], pos[5], axes='sxyz')
        child = '{}_{}'.format(self.frame_child, 'ned')

        self.br.sendTransform(translation, rotation, rospy.Time.now(), child, self.frame_odom)

    def send_tf_odom(self, pos, vel):
        """This is publishing the correct TF transformation with respect to RViz visualizer

        It applies rotations to the current position and velocity vectors from NED to XYZ frames.
        """
        # # convert to xyz
        # odom_pos = np.dot( self.rot_mat, pos.reshape((6,1)) ).flatten()
        # odom_vel = np.dot( self.rot_mat, vel.reshape((6,1)) ).flatten()

        # conversion from old auv_nav implementation (please check it twice!)
        odom_pos = (pos[0], -pos[1], -pos[2], pos[3], -pos[4], -pos[5])
        odom_vel = (vel[0], vel[1], vel[2], vel[3], vel[4], vel[5])

        # tf broadcast
        translation = (odom_pos[0], odom_pos[1], odom_pos[2])
        rotation = tft.quaternion_from_euler(odom_pos[3], odom_pos[4], odom_pos[5], axes='sxyz')

        self.br.sendTransform(translation, rotation, rospy.Time.now(), self.frame_child, self.frame_odom)

        # odometry message
        od = Odometry()
        od.header.stamp = rospy.Time.now()
        od.header.frame_id = self.frame_odom
        od.child_frame_id = self.frame_child

        od.pose.pose.position.x = odom_pos[0]		# north
        od.pose.pose.position.y = odom_pos[1]		# east
        od.pose.pose.position.z = odom_pos[2]		# depth

        od.pose.pose.orientation.x = rotation[0]
        od.pose.pose.orientation.y = rotation[1]
        od.pose.pose.orientation.z = rotation[2]
        od.pose.pose.orientation.w = rotation[3]

        od.twist.twist.linear.x = odom_vel[0]
        od.twist.twist.linear.y = odom_vel[1]
        od.twist.twist.linear.z = odom_vel[2]
        od.twist.twist.angular.x = odom_vel[3]
        od.twist.twist.angular.y = odom_vel[4]
        od.twist.twist.angular.z = odom_vel[5]

        self.pub_odom.publish(od)

    def send_nav_sts(self, pos, vel, depth):
        ns = NavSts()
        ns.header.stamp = rospy.Time.now()
        ns.header.frame_id = self.frame_child
        ns.position.north = pos[0]
        ns.position.east = pos[1]
        ns.position.depth = pos[2]
        ns.orientation.roll = pos[3]
        ns.orientation.pitch = pos[4]
        ns.orientation.yaw = pos[5]

        # NOTE: altitude implementation may be improved using external topics for instance uwsim ones
        #   its implementation is left outside the core part of the navigation simulator in dynamics_simulator.py
        ns.altitude = np.clip(depth - pos[2], 0.0, sim.MAX_SENSED_DEPTH)

        ns.body_velocity.x = vel[0]
        ns.body_velocity.y = vel[1]
        ns.body_velocity.z = vel[2]
        ns.orientation_rate.roll = vel[3]
        ns.orientation_rate.pitch = vel[4]
        ns.orientation_rate.yaw = vel[5]

        self.pub_navs.publish(ns)

    # def send_forces(self, event=None):
    #     # net force acting on the vehicle (useful for rviz)
    #     ws = WrenchStamped()
    #     ws.header.stamp = rospy.Time.now()
    #     ws.header.frame_id = 'base_link'
    #     ws.wrench.force.x = self.sim_nav.F_net[0]
    #     ws.wrench.force.y = -self.sim_nav.F_net[1]
    #     ws.wrench.force.z = -self.sim_nav.F_net[2]
    #     ws.wrench.torque.x = self.sim_nav.F_net[3]
    #     ws.wrench.torque.y = -self.sim_nav.F_net[4]
    #     ws.wrench.torque.z = -self.sim_nav.F_net[5]
    #     self.pub_force.publish(ws)

    def send_currents(self):
        fa = FloatArrayStamped()
        fa.header.stamp = rospy.Time.now()
        fa.values = [
            self.navsim.water_spd,
            #self.navsim.water_b,
            #self.navsim.water_a
        ]

        self.pub_curr.publish(fa)

    def run(self):
        # run simulation
        while not rospy.is_shutdown():
            self.navsim.update(self.tau)

            try:
                self.r_loop.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo('%s shutdown requested ...', self.name)


def main():
    rospy.init_node('nav_sim')
    name = rospy.get_name()
    rospy.loginfo('%s initializing ...', name)

    # parse args
    #args = rospy.myargv()

    sim_rate = int(rospy.get_param('~sim_rate', RATE_SIM))
    sim_rate = np.clip(sim_rate, 1, 100).astype(int)
    pub_rate = int(rospy.get_param('~pub_rate', RATE_PUB))
    pub_rate = np.clip(pub_rate, 1, 100).astype(int)

    # load config file
    nav_config = rospy.get_param('sim/nav', dict())

    # final config
    config = dict()
    config.update(nav_config)
    config.update({
        'topic_odom': rospy.get_param('~topic_odom', TOPIC_ODM),
        'output_nav': rospy.get_param('~topic_nav', TOPIC_NAV),
        'input_forces': rospy.get_param('~topic_forces', TOPIC_FRC),
        'frame_parent': rospy.get_param('~frame_parent', FRAME_PARENT),
        'frame_odom': rospy.get_param('~frame_odom', FRAME_ODOM),
        'frame_child': rospy.get_param('~frame_child', FRAME_CHILD),
        'verbose': bool(rospy.get_param('~verbose', False))
    })

    # console info
    rospy.loginfo('%s: odom topic: %s', name, config['topic_odom'])
    rospy.loginfo('%s: odom frame: %s', name, config['frame_odom'])
    rospy.loginfo('%s: child frame: %s', name, config['frame_child'])
    rospy.loginfo('%s: simulation rate: %s Hz', name, sim_rate)
    rospy.loginfo('%s: publisher rate: %s Hz', name, pub_rate)
    rospy.loginfo('%s: nav config:\n%s', name, nav_config)

    # init sim at fixed rate
    ns = NodeSimulator(name, sim_rate, pub_rate, **config)

    try:
        ns.run()
    except Exception:
        tb = traceback.format_exc()
        rospy.logfatal('%s uncaught exception, dying!\n%s', name, tb)

if __name__ == '__main__':
    main()
