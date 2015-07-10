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

"""Navigation Simulator node calculates the position of the vehicle in a simulated environment.

It consumes a force input in body-frame coordinates and updates the position of the vehicle in the simulated environment.
The input forces can be generated using the thrusters_simulator provided in the vehicle_core package. This two aspects
have been separated during the implementation of the nav_sim node to reduce coupling and allow the implementation of a real
thruster simulator using the thruster model developed during experiments conducted in the OSL tank.

This will enable software-in-the-loop (SIL) and hardware-in-the-loop (HIL)
simulations for a generic underwater vehicle given the thruster allocation
matrix (TAM) and the dynamic equations (DE).
"""
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

from vehicle_core.model import vehicle_model as vm
from vehicle_core.model import dynamic_model as dm
from vehicle_core.util import conversions as cnv

# simulator constants
DEFAULT_SEED = 47
DEFAULT_SEA_DEPTH = 1000.0          # meters
MIN_DEPTH = -0.15                   # meters
MAX_PITCH = 1.570                   # radians
MAX_SENSED_DEPTH = 100.0            # meters (this set the maximum value of the altitude in nav_stat)
MAX_CURRENT = 3.0                   # m/s

# console log
CONSOLE_STATUS = """nav_sim:
  tau:   %s
  F_net: %s
  acc:   %s
  vel:   %s
  pos:   %s %s
  altitude: %s
  vel_water: %s
"""


class NavigationSimulator(object):
    """NavigationSimulator is ..."""

    def __init__(self, dt, model_config, **kwargs):
        # state
        self.t = 0.0                                                            # simulation time (sec)
        self.dt = dt                                                            # simulation step (sec)

        # state of the vehicle (body frame referenced if not specified)
        #   x axis is along the vehicle, y axis is to the right of the vehicle, z axis is downward oriented
        #
        #       vel = [u v w p q r]
        #       a = d vel/dt
        #       pos = [x y z phi theta psi]
        #
        self.acc = np.zeros(6, dtype=np.float64)            # output: linear and angular acceleration
        self.vel = np.zeros(6, dtype=np.float64)            # velocity:	linear and angular velocity (body-frame)
        self.pos = np.zeros(6, dtype=np.float64)            # position:	linear and angular position
        self.pos_prev = np.zeros(6, dtype=np.float64)       # position: linear and angular position
        self.tau = np.zeros(6, dtype=np.float64)            # input forces [x y z k m n ] (N)

        # initial config
        self.pos = np.array(kwargs.get('pos', self.pos.tolist()))               # initial position
        self.depth_bottom = kwargs.get('depth_bottom', DEFAULT_SEA_DEPTH)       # sea bottom (meters)

        # dynamic model
        self.model_config = model_config
        self.model = vm.VehicleModel(self.model_config)

        # jacobians
        self.J = np.zeros((6, 6), dtype=np.float64)         # jacobian matrix (translate from body to Earth referenced)
        self.J_inv = np.zeros((6, 6), dtype=np.float64)     # inverse jacobian matrix

        # velocity vectors used in the navigation simulator
        #   vel_model is used for the combined velocity of vehicle and water current in the vehicle model equations
        #   vel_water is used for the water current velocity (at vehicle depth)
        self.vel_model = np.zeros(6, dtype=np.float64)
        self.vel_water = np.zeros(6, dtype=np.float64)

        # water currents
        self.water_surf = kwargs.get('water_surf', 0.0)             # surface speed of water current
        self.water_sigma = kwargs.get('water_sigma', 0.001)         #   normal distribution (mu, sigma)

        self.water_a = kwargs.get('water_a', 0.0)                   # angle of attack elevation (radians)
        self.water_a_sigma = kwargs.get('water_a_sigma', 0.001)     # angle of attack elevation variance (radians)
        self.water_b = kwargs.get('water_b', 0.0)                   # angle of attack azimuth (radians)
        self.water_b_sigma = kwargs.get('water_b_sigma', 0.001)     # angle of attack azimuth variance (radians)

        # rk related
        self.rk4_state = np.concatenate((self.pos, self.vel))   # NOTE: review this with body frame global conversion

        # init the rng (aiming for repeatable experiments)
        np.random.seed(DEFAULT_SEED)

    def reset(self):
        """Resets the state of the navigation simulator by setting to zero the internal state"""
        self.pos = np.zeros(6, dtype=np.float64)
        self.vel = np.zeros(6, dtype=np.float64)
        self.acc = np.zeros(6, dtype=np.float64)
        self.tau = np.zeros(6, dtype=np.float64)
        self.pos_prev = np.zeros(6, dtype=np.float64)

    def update_water_current(self, v, sigma_v, b, sigma_b, a, sigma_a):
        """Updates the water current model used inside the navigation simulator"""
        self.water_surf = v
        self.water_sigma = sigma_v
        self.water_b = b
        self.water_b_sigma = sigma_b
        self.water_a = a
        self.water_a_sigma = sigma_a

    def calc_currents(self):
        """Updates the vel_model variable used for the dynamic model equations based on the current state.

        First the velocity of the water current is calculated based on vehicle position, direction of the currents and
        the sea state. Later the water current velocity is added to the actual vehicle velocity.
        """
        Cza = np.eye(3)
        Cyb = np.eye(3)

        # water flow orientation model with added noise
        a = np.random.normal(self.water_a, self.water_a_sigma)
        b = np.random.normal(self.water_b, self.water_b_sigma)

        Cza[0, 0] = np.cos(a)
        Cza[0, 2] = -np.sin(a)
        Cza[2, 0] = -np.sin(a)
        Cza[2, 2] = np.cos(a)

        Cyb[0, 0] = np.cos(-b)
        Cyb[0, 1] = np.sin(-b)
        Cyb[1, 0] = -np.sin(-b)
        Cyb[1, 1] = np.cos(-b)

        # calculate the water velocity
        #   this assumes a single layer below the surface (with constant behaviour for the first 10 meters)
        #   and logarithmic decay with the increase of depth
        vs = np.random.normal(self.water_surf, self.water_sigma)

        if self.pos[2] < 10.0:
            vz = vs
        else:
            vz = vs * np.log10(1 + ((9.0 * self.pos[2]) / (self.depth_bottom - self.pos[2])))

        # calculate the velocity vector
        vc = np.array([vz, 0.0, 0.0], dtype=np.float64)
        water_ef = np.dot(Cza, np.dot(Cyb, vc.reshape((-1, 1))))

        self.vel_water = np.dot(self.J_inv[0:3, 0:3], water_ef).flatten()

        self.vel_model[0:5] = self.vel[0:5]     # copy the values in the vel_model
        self.vel_model[0:3] += self.vel_water   # add water currents

    def int_naive(self):
        """naive integration"""
        # calculate acceleration from forces using the dynamic model
        self.acc = self.model.update_acceleration(self.tau, self.pos, self.vel_model)

        # integration of velocity and convert to earth-fixed reference
        self.vel = self.vel + (self.acc * self.dt)

        self.J = dm.update_jacobian(self.J, self.pos[3], self.pos[4], self.pos[5])
        self.J_inv = np.linalg.pinv(self.J)

        vel_efec = np.dot(self.J, self.vel.reshape((6, 1))).flatten()

        # integration of position (double term integrator)
        self.pos = self.pos + (vel_efec * self.dt) + 0.5 * (self.acc * self.dt * self.dt)

    def int_velocity_verlet(self):
        """velocity verlet integrator
            [1]: http://en.wikipedia.org/wiki/Verlet_integration
            [2]: http://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicsnumericalintegration/Physics%20Tutorial%202%20-%20Numerical%20Integration.pdf
        """
        # calculate acceleration from forces using the dynamic model
        acc_prev = self.model.update_acceleration(self.tau, self.pos, self.vel_model)

        # convert velocity to earth-fixed reference
        self.J = dm.update_jacobian(self.J, self.pos[3], self.pos[4], self.pos[5])
        self.J_inv = np.linalg.pinv(self.J)

        vel_efec = np.dot(self.J, self.vel.reshape((6, 1))).flatten()
        acc_efec = np.dot(self.J, acc_prev.reshape((6, 1))).flatten()

        # update position
        self.pos = self.pos + (vel_efec * self.dt) + 0.5 * (acc_efec * self.dt * self.dt)

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

        # convert velocity to global coordinates as we want position in global coordinates
        self.J = dm.update_jacobian(self.J, self.pos[3], self.pos[4], self.pos[5])
        self.J_inv = np.linalg.pinv(self.J)

        vel_efec = np.dot(self.J, vel.reshape((6, 1))).flatten()

        return np.concatenate((vel_efec, acc))

    def rk4(self, x, h, y, f):
        k1 = f(x, y)
        k2 = f(x + 0.5 * h, y + 0.5 * h * k1)
        k3 = f(x + 0.5 * h, y + 0.5 * h * k2)
        k4 = f(x + h, y + h * k3)

        return x + h, y + ((h / 6.0) * (k1 + 2 * (k2 + k3) + k4))

    def update(self, tau):
        """Updates the state of the simulated vehicle by applying the forces of vector tau.

        :param tau: forces acting on the vehicle in body-frame coordinates (ie. thrusters)
        """
        # simulation step
        self.tau = tau
        self.t += self.dt

        # take into account the water currents
        self.calc_currents()

        # simple integration
        #self.int_naive()

        # improved integration accuracy
        self.int_velocity_verlet()

        # RK4 integration
        # self.t += self.dt
        # self.t, self.rk4_state = self.rk4(self.t, self.dt, self.rk4_state, self.rk4_derivative)
        # self.vel = self.rk4_state[6:12]     # velocity and position are already in body frame
        # self.pos = self.rk4_state[0:6]      # position is the integration of the velocity in body frame (for RK4)

        # wrap angles and limit pitch (-90 / 90)
        self.pos[3:6] = cnv.wrap_pi(self.pos[3:6])
        self.pos[4] = np.clip(self.pos[4], -MAX_PITCH, MAX_PITCH)

        # prevents the vehicle to fly to high! :)
        #   remember that a negative depth in NED frame means you are above the surface
        if self.pos[2] <= MIN_DEPTH:
            self.acc[2] = 0
            self.vel[2] = 0
            self.pos[2] = MIN_DEPTH

    def __str__(self):
        return CONSOLE_STATUS % (
            self.tau, self.model.F_net,
            self.acc, self.vel,
            self.pos[0:3], np.rad2deg(self.pos[3:6]),
            self.depth_bottom,
            self.vel_water
        )
