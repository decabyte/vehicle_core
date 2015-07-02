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

"""
The buoyancy term should be carefully adjusted cause is is dependent on the actual trim of the vehicle,
for this reason a good working value can be derived from the vehicle's weight and leaving the extra corrections
to the controller subsystem in terms of gains and offsets.

For simulation purposes the buoyancy it can be computed using formula:
  B = rho * volume * 9.81    # buoyancy (N)

An early experiment measured this value to be 546.1106 for Nessie VII AUV but that value was depending on the
specific setup and trimming related to the experimental setup used for its measurement.

The COG and COB need a similar approach, in this case, they can be assumed to be in the same geometrical position
in order to avoid the generation of strong pitch forces when using this model with the actual vehicle. These terms
instead can be safely adjusted for simulation purposes.
"""

from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import dynamic_model as dm

# constants
DEFAULT_RHO = 1025.0    # density of water (salt water is default)
MAX_VEL = 20.0          # stable numerical limit for the vehicle velocity vector (m/s or rad/s)
MAX_ATT = np.pi         # stable numerical limit for the vehicle attitude vector (rad)


class VehicleModel(object):
    """VehicleModel class provides an easy tool to deal with dynamical forces acting on the vehicle.

    This module is separated from the ROS interface for improving the testing and debug procedures as well as allowing its
    embedding in other modules (for instance to calculate navigation estimates at run-time without launching the heavy machinery
    involved for the complete nav_sim node).

    It can be used to calculate the total net forces acting on the vehicle if provided with an input force in body-frame
    coordinates and the position and velocities of the vehicle itself. This approach is used by the nav_sim modules.

    It can be used to calculate the total hydrodynamic forces if provided with the navigation information (position and
    velocities) measured with on-board sensors. This approach is used by the vehicle_controller modules.
    """

    def __init__(self, config, **kwargs):
        self.shape = config.get('shape', 'cylindrical')

        if self.shape != 'cylindrical':
            raise NotImplemented('Could not create VehicleModel for non-cylindrical shapes.')

        self.mass = config.get('mass')                              # total mass of the vehicle [kg]
        self.W = config.get('weight')                               # weight [N]    (it should be mass * 9.81)
        self.B = config.get('buoyancy')                             # buoyancy [N]  (it should be close to W)
        self.radius = config.get('radius')                          # radius [m]
        self.length = config.get('length')                          # length [m]
        self.volume = config.get('volume')                          # volume of the vehicle [m^3]
        self.water_rho = config.get('water_rho', DEFAULT_RHO)       # density (salt water) [kg/m^3]

        self.cog = np.array(config.get('cog'), dtype=np.float64)    # center of gravity [m] (xg, yg, zg)
        self.cob = np.array(config.get('cob'), dtype=np.float64)    # center of buoyancy [m]  (xb, yb, zb)

        # principal quadratic drag coefficients (approximation for underwater vehicles)
        #   [x_uu, y_vv, z_ww, k_pp, m_qq, n_rr]
        self.quadratic_drag = np.array(config.get('quadratic_drag'), dtype=np.float64)

        # inertia tensor wrt origin of vehicle [mass * length^2]
        self.inertia = np.array([
            self.mass * (self.radius ** 2) * (1.0 / 2.0),
            self.mass * (3 * (self.radius ** 2) + (self.length ** 2)) * (1.0 / 12.0),
            self.mass * (3 * (self.radius ** 2) + (self.length ** 2)) * (1.0 / 12.0),
        ], dtype=np.float64)

        # forces
        self.F_net = np.zeros(6, dtype=np.float64)         # total net force acting on the vehicle
        # self.F_model = np.zeros(6, dtype=np.float64)     # total hydrodynamic forces acting on the vehicle
        # self.F_C = np.zeros(6, dtype=np.float64)         # coriolis forces
        # self.F_D = np.zeros(6, dtype=np.float64)         # damping forces
        # self.F_G = np.zeros(6, dtype=np.float64)         # restoring forces

        # calculate added terms based on cylindrical shape
        self.added_terms = np.array([
            -0.1 * self.mass,                                                                           # xu_dot
            -np.pi * self.water_rho * (self.radius ** 2) * self.length,                                 # yv_dot
            -np.pi * self.water_rho * (self.radius ** 2) * self.length,                                 # zw_dot
            -np.pi * self.water_rho * (self.radius ** 4) * (1.0 / 4.0),                                 # kp_dot
            -np.pi * self.water_rho * (self.radius ** 2) * (self.length ** 3) * (1.0 / 12.0),           # mq_dot
            -np.pi * self.water_rho * (self.radius ** 2) * (self.length ** 3) * (1.0 / 12.0)            # nr_dot
        ], dtype=np.float64)

        # calculate rigid body inertia
        xg, yg, zg = self.cog
        ix, iy, iz = self.inertia

        self.MRB = np.array([
            [self.mass, 0.0, 0.0, 0.0, self.mass * zg, -self.mass * yg],
            [0.0, self.mass, 0.0, -self.mass * zg, 0.0, self.mass * xg],
            [0.0, 0.0, self.mass, self.mass * yg, -self.mass * xg, 0.0],
            [0.0, -self.mass * zg, self.mass * yg, ix, 0.0, 0.0],
            [self.mass * zg, 0.0, -self.mass * xg, 0.0, iy, 0.0],
            [-self.mass * yg, self.mass * xg, 0.0, 0.0, 0.0, iz]
        ], dtype=np.float64)

        # added mass (approximation for underwater vehicles)
        #
        #     [xu_dot, 0, 0, 0, 0, 0],
        #     [0, yv_dot, 0, 0, 0, 0],
        #     [0, 0, zw_dot, 0, 0, 0],
        #     [0, 0, 0, kp_dot, 0, 0],
        #     [0, 0, 0, 0, mq_dot, 0],
        #     [0, 0, 0, 0, 0, nr_dot]
        #
        self.MA = np.diag(self.added_terms)

        # dynamic equation mass matrix and inverse matrix
        self.M = self.MRB - self.MA
        self.inv_M = np.linalg.pinv(self.M)

        # vehicle limits
        self.lim_att = MAX_ATT * np.ones(3, dtype=np.float64)   # k, m, n
        self.lim_vel = MAX_VEL * np.ones(6, dtype=np.float64)   # u, v, w, p, q, r


    def update_forward_model(self, pos, vel):
        """Calculates the forward model forces based on the provided position and velocity vector using the acceleration
        approximation used in literature in order to allow the use of feed-forward controllers.

        If available this functions uses the optimized version of the dynamic model, loading the .so file, providing
        a fast computation despite its long list of parameters.

        :param pos: current position of the vehicle [x,y,z,k,m,n]   (in meters and radians)
        :param vel: current velocity of the vehicle [u,v,w,p,q,r]   (in m/s and rad/s)
        :return: numpy.ndarray of shape (6,) with forces acting in body-frame [X,Y,Z,K,M,N]
        """

        # numerical limits
        #pos[3:6] = np.clip(pos[3:6], -self.lim_att, self.lim_att)
        vel = np.clip(vel, -self.lim_vel, self.lim_vel)

        return dm.calc_model_forward(
            pos, vel, self.cog, self.cob, self.mass, self.inertia, self.W, self.B,
            self.M, self.added_terms, self.quadratic_drag
        )


    def update_coupled_model(self,pos,vel, acc, des_vel):

        """Calculates forces needed in the coupled controller based on the position, velocity and desired velocity.

        :param pos: current position of the vehicle [x,y,z,k,m,n]   (in meters and radians)
        :param vel: current velocity of the vehicle [u,v,w,p,q,r]   (in m/s and rad/s)
        :param des_vel : desired velocity of the vehicle [ud, vd, wd, pd, qd, rd] (in m/s and rad/s)
        :return: numpy.ndarray of shape (6,) with forces required for coupled-control implementation in body-frame [X,Y,Z,K,M,N]
        """

        vel = np.clip(vel, -self.lim_vel, self.lim_vel)
        tau = dm.calc_coupled_forces(
            pos, vel, des_vel, self.cog, self.cob, self.mass, self.inertia, self.W, self.B,
            self.added_terms, self.quadratic_drag
        )

        return tau

    def update_tau(self, pos, vel, acc):
        """Calculates the tau given the requested acceleration, the actual position and velocities, including the dynamics
        of the vehicle using the model parameters. It can be used for linearizing the feedback loop in the vehicle controller.

        If available this functions uses the optimized version of the dynamic model, loading the .so file, providing
        a fast computation despite its long list of parameters.

        :param pos: current position of the vehicle [x,y,z,k,m,n]   (in meters and radians)
        :param vel: current velocity of the vehicle [u,v,w,p,q,r]   (in m/s and rad/s)
        :param acc: requested acceleration of the vehicle [udot,vdot,wdot,pdot,qdot,rdot]   (in m/s^2 and rad/s^2)
        :return: numpy.ndarray of shape (6,) with forces acting in body-frame [X,Y,Z,K,M,N]
        """

        # numerical limits
        #pos[3:6] = np.clip(pos[3:6], -self.lim_att, self.lim_att)
        vel = np.clip(vel, -self.lim_vel, self.lim_vel)

        return np.dot(self.M, acc) + dm.calc_other_forces(
            pos, vel, self.cog, self.cob, self.mass, self.inertia, self.W, self.B,
            self.added_terms, self.quadratic_drag
        )


    def update_acceleration(self, tau, pos, vel):
        """Calculates the acceleration using inverse model, given the input force and the current position and velocities.

        If available this functions uses the optimized version of the dynamic model, loading the .so file, providing
        a fast computation despite its long list of parameters. It also stores the last computed value as an object
        attribute for later inspection or verbose printing.

        :param tau: input vector with forces acting in body-frame [X,Y,Z,K,M,N]
        :param pos: current position of the vehicle [x,y,z,k,m,n]   (in meters and radians)
        :param vel: current velocity of the vehicle [u,v,w,p,q,r]   (in m/s and rad/s)
        :return: computed acceleration of the vehicle [udot,vdot,wdot,pdot,qdot,rdot]   (in m/s^2 and rad/s^2)
        """

        # calculate model
        tau_model = self.update_forward_model(pos, vel)

        # dynamic equation (compute total force)
        self.F_net = tau - tau_model

        # calculate acceleration from forces
        return np.dot(self.inv_M, self.F_net)


    def __str__(self):
        pass
