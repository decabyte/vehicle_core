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

import cvxpy as cp

# default config
MODEL_THR_FWD = np.array([0.003, 0.059, 0.059])
MODEL_THR_REV = np.array([0.003, 0.075, 0.050])


class OptimalThrustAllocator(object):

    def __init__(self, tam):
        # config
        self.n = tam.shape[0]       # number of thrusters
        self.tam = tam              # thruster allocation matrix

        # parameters
        self.beta = 10.0

        # quadratic current to thrust
        W = np.repeat(MODEL_THR_FWD.reshape(1, -1), self.n, 0)

        # quadratic term for slack variables
        Q = np.eye(self.n) * MODEL_THR_FWD[0] * 1000

        # construct the problem
        self.u = cp.Variable(self.n)
        self.s = cp.Variable(self.n)
        self.td = cp.Parameter(self.n)
        self.u_avail = cp.Parameter(self.n, sign='positive')

        # build the cost function
        currents = MODEL_THR_FWD[0] * cp.abs(self.u)**2 + MODEL_THR_FWD[1] * cp.abs(self.u) + MODEL_THR_FWD[2]
        slacks = cp.quad_form(self.s, Q)
        max_effort = self.beta * cp.max_entries(self.u)

        # define cost and objective
        self.cost = cp.sum_entries(currents) + slacks # + max_effort
        self.objective = cp.Minimize(self.cost)

        # define problem constraints
        self.constraints = [
            self.u <= self.u_avail, self.u >= -self.u_avail,
            self.tam * self.u == self.td - self.s,
        ]

        # define the parametric problem
        self.prob = cp.Problem(self.objective, self.constraints)


    def allocate_thrust(self, tau, force_avail):
        # set the parameters
        self.td.value = tau
        self.u_avail.value = force_avail

        # solve the problem
        self.prob.solve(solver=cp.ECOS, abstol=1e-3, reltol=1e-3, feastol=1e-3)

        # TODO: throw an exception if the solver is not working and switch back to standard allocation in pilot
        if self.prob.status != cp.OPTIMAL:
            return np.zeros_like(force_avail)

        return np.array(self.u.value).flatten()
