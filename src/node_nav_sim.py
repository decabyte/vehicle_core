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

"""Navigation Simulator node is the core part of the navigation_simulator module.

This node calculates the position of the vehicle in a simulated environment and publish the navigation messages at a given rate.

It consumes a force input in body-frame coordinates and updates the position of the vehicle in the simulated environment.
The input forces can be generated using the thrusters_simulator provided in the navigation_simulator package. This two aspects
have been separated during the implementation of the nav_sim node to reduce coupling and allow the implementation of a real
thruster simulator using the thruster model developed during experiments conducted in the OSL tank.

This will enable software-in-the-loop (SIL) and hardware-in-the-loop (HIL)
simulations for a generic underwater vehicle given the thruster allocation
matrix (TAM) and the dynamic equations (DE).
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

from vehicle_core.model import vehicle_model as vm
from vehicle_core.model import dynamic_model as dm

from nav_msgs.msg import Odometry
from auv_msgs.msg import NavSts
from std_srvs.srv import Empty, EmptyResponse

from vehicle_interface.msg import Vector6Stamped
from vehicle_interface.srv import Vector6Service, Vector6ServiceResponse


# params
RATE_SIM = 20       # simulation rate (Hz) (this is limited by the current implementation)
RATE_PUB = 10       # publisher rate (Hz) (one should publish messages at a slower rate)

TOPIC_NAV = 'nav/nav_sts'
TOPIC_THR = 'thrusters/commands'
TOPIC_FRC = 'forces/sim/body'
TOPIC_NET = 'forces/sim/net'

SRV_RESET = 'nav/reset'
SVR_OFFSET = 'nav/offset'

TOPIC_ODM = 'nav/odometry'
FRAME_PARENT = 'odom'
FRAME_CHILD = 'base_link'


# simulator constants
MAX_ABOVE_SEA_LEVEL = -0.15


# utils
def wrap_pi(angle):
    return ((angle + np.pi) % (2 * np.pi)) - np.pi

def wrap_2pi(angle):
    return ((angle + np.pi) % (2 * np.pi))



class NavigationSimulator(object):
    """NavigationSimulator is part of the navigation_simulator module. it provides interfacing with the ROS system.

    This class handles force inputs and publish navigation messages simulating the behaviour of the navigation modules
    running inside the vehicle during real operations. It also offers extra topics useful for monitoring the vehicle during
    simulation, like the total net force acting on the vehicle or the status of the internal navigation simulator.
    """

    def __init__(self, name, initial_position, sim_rate, pub_rate, **kwargs):
        self.name = name
        self.dt = 1.0 / sim_rate
        self.verbose = kwargs.get('verbose', False)

        # state
        self.t = 0.0
        self.tau = np.zeros(6)
        self.depth_bottom = kwargs.get('depth_bottom', 50)

        # dynamic model
        self.model_config = rospy.get_param('sim/model', dict())
        self.model = vm.VehicleModel(self.model_config)

        # state of the vehicle (body frame referenced if not specified)
        #   x axis is along the vehicle, y axis is to the right of the vehicle, z axis is downward oriented
        #
        #       vel = [u v w p q r]
        #       a = d vel/dt
        #       pos = [x y z phi theta psi]
        #
        self.acc = np.zeros(6)      # output: linear and angular acceleration
        self.vel = np.zeros(6)      # velocity:	linear and angular velocity (body-frame)
        self.pos = np.zeros(6)      # position:	linear and angular position
        self.pos_prev = np.zeros(6) # position: linear and angular position

        # nav offset
        self.offset_pos = np.zeros(6)

        # initial conditions
        self.pos = initial_position
        self.rk4_state = np.concatenate((self.pos, self.vel))   # NOTE: review this with body frame global conversion

        # jacobians
        self.J = np.zeros((6, 6))       # jacobian matrix (translate velocity from body referenced to Earth referenced)
        self.J_inv = np.zeros((6, 6))   # inverse jacobian matrix


        # topics
        self.input_forces = kwargs.get('input_forces', TOPIC_FRC)
        self.output_nav = kwargs.get('output_nav', TOPIC_NAV)
        self.output_forces = kwargs.get('output_forces', TOPIC_NET)

        # odometry
        self.br = tf.TransformBroadcaster()
        self.topic_odom = kwargs.get('topic_odom', TOPIC_ODM)
        self.frame_parent = kwargs.get('frame_parent', FRAME_PARENT)
        self.frame_child = kwargs.get('frame_child', FRAME_CHILD)

        rot_mat_z = np.zeros((6,6))
        rot_mat_z[0:3, 0:3] = tft.rotation_matrix(np.deg2rad(180), np.array([0, 0, 1]))[:3, :3]
        rot_mat_z[3:6, 3:6] = tft.rotation_matrix(np.deg2rad(180), np.array([0, 0, 1]))[:3, :3]

        rot_mat_y = np.zeros((6,6))
        rot_mat_y[0:3, 0:3] = tft.rotation_matrix(np.deg2rad(180), np.array([0, 1, 0]))[:3, :3]
        rot_mat_y[3:6, 3:6] = tft.rotation_matrix(np.deg2rad(180), np.array([0, 1, 0]))[:3, :3]

        # odometry rotation matrix
        self.rot_mat = np.dot( rot_mat_z, rot_mat_y )

        # ros interface
        self.sub_forces = rospy.Subscriber(self.input_forces, Vector6Stamped, self.handle_forces, tcp_nodelay=True, queue_size=1)
        self.pub_navs = rospy.Publisher(self.output_nav, NavSts, tcp_nodelay=True, queue_size=1)
        self.pub_odom = rospy.Publisher(self.topic_odom, Odometry, tcp_nodelay=True, queue_size=1)
        #self.pub_force = rospy.Publisher(self.output_forces, WrenchStamped, tcp_nodelay=True, queue_size=1)

        self.srv_zero = rospy.Service(SRV_RESET, Empty, self.handle_reset)
        self.srv_offset = rospy.Service(SVR_OFFSET, Vector6Service, self.handle_offset)

        # timers
        self.sim_loop = rospy.Rate(sim_rate)
        self.t_pub = rospy.Timer(rospy.Duration(1 / pub_rate), self.publish_navigation)

        # optional interfaces
        if self.verbose:
            self.t_pri = rospy.Timer(rospy.Duration(1), self.print_status)
            #self.t_net = rospy.Timer(rospy.Duration(1 / pub_rate), self.send_forces)


    def handle_reset(self, req):
        """This is setting resetting the status of the navigation simulator."""

        self.pos = np.zeros(6)
        self.vel = np.zeros(6)
        self.pos_prev = np.zeros(6)

        rospy.logwarn('%s: resetting the nav simulator ...', self.name)
        return EmptyResponse()

    def handle_offset(self, req):
        """This is setting the nav offset using the user request."""
        self.offset_pos = np.array(req.request)

        rospy.logwarn('%s: offsetting the nav origin with %s ...', self.name, self.offset_pos)
        return Vector6ServiceResponse(True, self.offset_pos)

    def handle_forces(self, data):
        # load forces from message
        #   forces are in body frame coordinates
        self.tau = np.array(data.values[0:6])


    def publish_navigation(self, event=None):
        """This function is used for publishing sending data outside of this node.

        It applies the navigation offset, used for indoor operations without the use of the global positioning system.
        """
        # current state
        pos = np.copy(self.pos)
        vel = np.copy(self.vel)

        # apply offsets (simple strategy)
        pos[0:3] += self.offset_pos[0:3]

        # send ROS messages
        self.send_nav_sts(pos, vel)
        self.send_odom(pos, vel)


    def send_odom(self, pos, vel):
        # convert to xyz
        odom_pos = np.dot( self.rot_mat, pos.reshape((6,1)) ).flatten()
        odom_vel = np.dot( self.rot_mat, vel.reshape((6,1)) ).flatten()

        # tf broadcast
        translation = (odom_pos[0], odom_pos[1], odom_pos[2])
        rotation = tft.quaternion_from_euler(odom_pos[3], odom_pos[4], odom_pos[5], axes='sxyz')

        self.br.sendTransform(translation, rotation, rospy.Time.now(), self.frame_child, self.frame_parent)

        # odometry message
        od = Odometry()
        od.header.stamp = rospy.Time.now()
        od.header.frame_id = self.frame_parent
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


    def send_nav_sts(self, pos, vel):
        ns = NavSts()
        ns.header.stamp = rospy.Time.now()
        ns.header.frame_id = self.frame_child
        ns.position.north = pos[0]
        ns.position.east = pos[1]
        ns.position.depth = pos[2]
        ns.orientation.roll = pos[3]
        ns.orientation.pitch = pos[4]
        ns.orientation.yaw = pos[5]

        # NOTE: altitude implementation may be improved using external topics
        # for instance uwsim ones, thus its implementation is left outside the core part
        # of the navigation simulator in dymanics_simulator.py
        ns.altitude = self.depth_bottom - pos[2]

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


    def int_naive(self):
        """naive integration"""
        # calculate acceleration from forces using the dynamic model
        self.acc = self.model.update_acceleration(self.tau, self.pos, self.vel)

        # integration of velocity and convert to earth-fixed reference
        self.vel = self.vel + (self.acc * self.dt)

        self.J = dm.update_jacobian(self.J, self.pos[3], self.pos[4], self.pos[5])
        self.J_inv = np.linalg.inv(self.J)

        vel_efec = np.dot(self.J, self.vel.reshape((6, 1))).flatten()

        # integration of position (double term integrator)
        self.pos = self.pos + (vel_efec * self.dt) + (self.acc * self.dt * self.dt)


    def int_velocity_verlet(self):
        """velocity verlet integrator
            [1]: http://en.wikipedia.org/wiki/Verlet_integration
            [2]: http://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicsnumericalintegration/Physics%20Tutorial%202%20-%20Numerical%20Integration.pdf
        """
        # calculate acceleration from forces using the dynamic model
        acc_prev = self.model.update_acceleration(self.tau, self.pos, self.vel)

        # convert velocity to earth-fixed reference
        self.J = dm.update_jacobian(self.J, self.pos[3], self.pos[4], self.pos[5])
        self.J_inv = np.linalg.inv(self.J)

        vel_efec = np.dot(self.J, self.vel.reshape((6, 1))).flatten()
        acc_efec = np.dot(self.J, acc_prev.reshape((6, 1))).flatten()

        # update position
        self.pos = self.pos + (vel_efec * self.dt) + (acc_efec * self.dt * self.dt)

        # compute the new velocity from forces using the dynamic model
        self.acc = self.model.update_acceleration(self.tau, self.pos, self.vel)
        self.vel = self.vel + 0.5 * (acc_prev + self.acc) * self.dt


    # Runge-Kutta integration method:
    #   - rk4_derivative: this function update the state using derivatives
    #   - rk4: this function implements a RK4 integration method
    def rk4_derivative(self, t, state):
        pos = state[0:6]
        vel = state[6:12]

        # invoke main computation
        acc = self.model.update_acceleration(self.tau, pos, vel)
        #vel, acc = self.compute_acceleration(pos, vel)

        # convert velocity to global coordinates as we want position in global coordinates
        self.J = dm.update_jacobian(self.J, self.pos[3], self.pos[4], self.pos[5])
        self.J_inv = np.linalg.inv(self.J)

        vel_efec = np.dot(self.J, vel.reshape((6, 1))).flatten()

        return np.concatenate((vel_efec, acc))

    def rk4(self, x, h, y, f):
        k1 = f(x, y)
        k2 = f(x + 0.5 * h, y + 0.5 * h * k1)
        k3 = f(x + 0.5 * h, y + 0.5 * h * k2)
        k4 = f(x + h, y + h * k3)

        return x + h, y + ((h / 6.0) * (k1 + 2 * (k2 + k3) + k4))


    def update_simulation(self):
        """This method updates the status of the simulated vehicle.

        :param tau: forces acting on the vehicle in body-frame coordinates (ie. thrusters)
        """
        # simple integration
        #self.int_naive()

        # improved integration accuracy
        self.int_velocity_verlet()

        # # RK4 integration
        # self.t += self.dt
        # self.t, self.rk4_state = self.rk4(self.t, self.dt, self.rk4_state, self.rk4_derivative)
        # self.vel = self.rk4_state[6:12]     # velocity and position are already in body frame
        # self.pos = self.rk4_state[0:6]      # position is the integration of the velocity in body frame (for RK4)

        # wrap angles and limit pitch (-90 / 90)
        self.pos[3:6] = wrap_pi(self.pos[3:6])
        self.pos[4] = np.clip(self.pos[4], -1.570, 1.570)

        # prevent the vehicle to fly to high! :)
        if self.pos[2] <= MAX_ABOVE_SEA_LEVEL:
            self.acc[2] = 0
            self.vel[2] = 0
            self.pos[2] = MAX_ABOVE_SEA_LEVEL


    def run(self):
        # init simulation
        self.tau = np.zeros(6)

        # run simulation
        while not rospy.is_shutdown():
            self.update_simulation()

            try:
                self.sim_loop.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo('%s shutdown requested ...', self.name)


    def print_status(self, event=None):
        print(self)


    def __str__(self):
        return """nav_sim:
          tau:   %s
          F_net: %s
          acc:   %s
          vel:   %s
          pos:   %s %s
          off_pos: %s %s
          altitude: %s
        """ % (
            self.tau, self.model.F_net,
            self.acc, self.vel,
            self.pos[0:3], np.rad2deg(self.pos[3:6]),
            self.offset_pos[0:3], np.rad2deg(self.offset_pos[3:6]),
            self.depth_bottom,
        )


def main():
    rospy.init_node('nav_sim')
    name = rospy.get_name()
    rospy.loginfo('%s initializing ...', name)

    # parse args
    args = rospy.myargv()

    if '-v' in args:
        verbose = True
    else:
        verbose = False

    # TODO: remove parameters from launch files and move them to navigation configuration file (/nav/sim namespace)
    # load parameters
    pose_x = int(rospy.get_param('~pose_x', 0))
    pose_y = int(rospy.get_param('~pose_y', 0))
    pose_z = int(rospy.get_param('~pose_z', 0))
    pose_k = int(rospy.get_param('~pose_k', 0))
    pose_m = int(rospy.get_param('~pose_m', 0))
    pose_n = int(rospy.get_param('~pose_n', 0))

    pose = np.array([pose_x, pose_y, pose_z, pose_k, pose_m, pose_n])

    sim_rate = int(rospy.get_param('~sim_rate', RATE_SIM))
    sim_rate = np.clip(sim_rate, 1, 100).astype(int)
    pub_rate = int(rospy.get_param('~pub_rate', RATE_PUB))
    pub_rate = np.clip(pub_rate, 1, 100).astype(int)

    # load config file
    nav_config = rospy.get_param('navigation_simulator', dict())

    # final config
    config = {
        'topic_odom': rospy.get_param('~topic_odom', TOPIC_ODM),
        'frame_parent': rospy.get_param('~frame_parent', FRAME_PARENT),
        'frame_child': rospy.get_param('~frame_child', FRAME_CHILD),
        'verbose': verbose
    }
    config.update(nav_config)

    # console info
    rospy.loginfo('%s: odom topic: %s', name, config['topic_odom'])
    rospy.loginfo('%s: parent frame: %s', name, config['frame_parent'])
    rospy.loginfo('%s: child frame: %s', name, config['frame_child'])
    rospy.loginfo('%s: simulation rate: %s Hz', name, sim_rate)
    rospy.loginfo('%s: publisher rate: %s Hz', name, pub_rate)
    rospy.loginfo('%s: nav config:\n%s', name, nav_config)

    # init sim at fixed rate
    nn = NavigationSimulator(name, pose, sim_rate, pub_rate, **config)

    try:
        nn.run()
    except Exception:
        tb = traceback.format_exc()
        rospy.logfatal('%s uncaught exception, dying!\n%s', name, tb)


if __name__ == '__main__':
    main()
