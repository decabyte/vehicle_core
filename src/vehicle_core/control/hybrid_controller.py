#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Ocean Systems Laboratory, Heriot-Watt University, UK.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of the Heriot-Watt University nor the names of
# its contributors may be used to endorse or promote products
# derived from this software without specific prior written
# permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  Original authors:
#   Valerio De Carolis, Marian Andrecki, Corina Barbalata, Gordon Frost

from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)
np.set_printoptions(formatter={'float': '{: 0.3f}'.format})
# see: http://stackoverflow.com/questions/2891790/pretty-printing-of-numpy-array

import vehicle_controller as vc

from vehicle_core.model import vehicle_model as vm
from vehicle_core.model import dynamic_model as dm
from vehicle_core.util import conversions as cnv

# default config
HYBRID_LIM_POS = 0.8        # meters

CONSOLE_STATUS = """%s
  req_v: %s
  lim_v: %s
  ep: %s
  epp: %s
  dp: %s
  evp: %s
  evd: %s
  evi: %s
  tau_c: %s
"""


class HydridController(vc.VehicleController):

    def __init__(self, dt, ctrl_config, model_config, **kwargs):
        super(HydridController, self).__init__(dt, ctrl_config, **kwargs)

        # init params
        self.kpos = np.zeros_like(self.pos)
        self.kposprev = np.zeros_like(self.pos)
        self.lim_pos = np.ones_like(self.pos) * HYBRID_LIM_POS

        # state
        self.des_pos_prev = np.zeros_like(self.pos)
        self.delta_pos = np.zeros_like(self.pos)
        self.sigma_pos = np.zeros_like(self.pos)

        # errors
        self.err_pos = np.zeros_like(self.pos)
        self.err_pos_prev = np.zeros_like(self.pos)

        self.err_vel = np.zeros_like(self.vel)
        self.err_vel_prev = np.zeros_like(self.vel)
        self.err_vel_der = np.zeros_like(self.vel)
        self.err_vel_int = np.zeros_like(self.vel)

        # intermediate requests
        self.req_vel = np.zeros_like(self.pos)
        self.tau_ctrl = np.zeros_like(self.pos)

        # init jacobian matrices
        self.J = np.zeros((6, 6))           # jacobian matrix (translate velocity from body frame to Earth frame)
        self.J_inv = np.zeros((6, 6))       # inverse jacobian matrix


    def update_config(self, ctrl_config, model_config):
        # sliding mode
        self.kpos = np.array(ctrl_config['kpos'])
        self.kposprev = np.array(ctrl_config['kposprev'])
        self.kgamma = np.array(ctrl_config['kgamma'])
        self.lim_pos = np.array(ctrl_config['lim_pos'])

        # pid parameters (velocity)
        self.vel_Kp = np.array([
            ctrl_config['vel_u']['kp'],
            ctrl_config['vel_v']['kp'],
            ctrl_config['vel_w']['kp'],
            ctrl_config['vel_p']['kp'],
            ctrl_config['vel_q']['kp'],
            ctrl_config['vel_r']['kp'],
        ])

        self.vel_Kd = np.array([
            ctrl_config['vel_u']['kd'],
            ctrl_config['vel_v']['kd'],
            ctrl_config['vel_w']['kd'],
            ctrl_config['vel_p']['kd'],
            ctrl_config['vel_q']['kd'],
            ctrl_config['vel_r']['kd'],
        ])

        self.vel_Ki = np.array([
            ctrl_config['vel_u']['ki'],
            ctrl_config['vel_v']['ki'],
            ctrl_config['vel_w']['ki'],
            ctrl_config['vel_p']['ki'],
            ctrl_config['vel_q']['ki'],
            ctrl_config['vel_r']['ki'],
        ])

        self.vel_lim = np.array([
            ctrl_config['vel_u']['lim'],
            ctrl_config['vel_v']['lim'],
            ctrl_config['vel_w']['lim'],
            ctrl_config['vel_p']['lim'],
            ctrl_config['vel_q']['lim'],
            ctrl_config['vel_r']['lim'],
        ])

        self.vel_input_lim = np.array([
            ctrl_config['vel_u']['input_lim'],
            ctrl_config['vel_v']['input_lim'],
            ctrl_config['vel_w']['input_lim'],
            ctrl_config['vel_p']['input_lim'],
            ctrl_config['vel_q']['input_lim'],
            ctrl_config['vel_r']['input_lim'],
        ])


    def update(self, position, velocity):
        # store nav updates
        self.pos = position
        self.vel = velocity

        # update jacobian
        self.J = dm.update_jacobian(self.J, self.pos[3], self.pos[4], self.pos[5])
        self.J_inv = np.linalg.pinv(self.J)

        # update errors
        self.err_pos = self.pos - self.des_pos_prev
        self.delta_pos = self.des_pos - self.des_pos_prev

        # update request position
        self.des_pos_prev = self.des_pos

        # neuronal approach
        # ...

        # error limits
        self.err_pos = np.clip(self.err_pos, -self.lim_pos, self.lim_pos)

        # error dynamics
        self.vel_efec = np.dot(self.J, self.vel.reshape((6, 1))).flatten()
        self.err_pos_dot = (self.err_pos - self.delta_pos) + (self.dt * self.vel_efec)

        # error dynamics with neuron
        # ...

        # sliding surface
        self.sigma_pos = self.err_pos_dot - self.err_pos + (self.kposprev * self.err_pos_prev)

        # sliding surface with neuron
        # ...

        # anti-chattering
        self.gamma = self.kgamma * np.tanh(0.5 * self.sigma_pos)

        # virtual position
        self.virtual_pos = self.delta_pos - (self.kpos * self.err_pos) - (self.kposprev * self.err_pos_prev) - self.gamma

        # update sliding surface for yaw
        a = np.arctan2(self.virtual_pos[1], self.virtual_pos[0]) - self.pos[5]
        self.gamma[5] = self.kgamma[5] * np.tanh(0.5 * cnv.wrap_pi(a))

        # calculate required velocity
        self.req_vel = np.zeros_like(self.vel)
        d = (np.arctan2(self.virtual_pos[1], self.virtual_pos[0]) - self.pos[5] - self.gamma[5]) / self.dt

        self.req_vel[0] = np.linalg.norm(self.virtual_pos[0:2]) / self.dt
        self.req_vel[5] = np.tanh(0.5 * cnv.wrap_pi(d))

        # TESTED also with:
        #self.req_vel[5] = np.tanh(0.5 * cnv.wrap_pi(a))

        #print('a: %s' % np.rad2deg(cnv.wrap_pi(a)))
        #print('v: %s' % self.req_vel[5])
        #print('vpos: %s\ngamma: %s\nreqv: %s' % (self.virtual_pos, self.gamma, self.req_vel))

        # save old error
        self.err_pos_prev = self.err_pos

        # if running in velocity mode ignore the first pid
        if self.ctrl_mode == vc.MODE_VELOCITY:
            self.req_vel = self.des_vel

        # apply user velocity limits (if any)
        self.req_vel = np.clip(self.req_vel, -self.lim_vel, self.lim_vel)


        ### REUSE PREVIOUS VELOCITY PID ###

        # model-free pid cascaded controller
        #   second pid (inner loop on velocity)
        self.err_vel = np.clip(self.vel - self.req_vel, -self.vel_input_lim, self.vel_input_lim)
        self.err_vel_der = (self.err_vel - self.err_vel_prev) / self.dt
        self.err_vel_int = np.clip(self.err_vel_int + self.err_vel, -self.vel_lim, self.vel_lim)

        # velocity integral terms set to zero to avoid oscillations (ignore depth)
        vel_changed = np.sign(self.err_vel) != np.sign(self.err_vel_prev)
        vel_changed[2] = False
        self.err_vel_int[vel_changed] = 0.0

        # update previous error
        self.err_vel_prev = self.err_vel

        # second pid output
        self.tau_ctrl = (-self.vel_Kp * self.err_vel) + (-self.vel_Kd * self.err_vel_der) + (-self.vel_Ki * self.err_vel_int)


        # hard limits on forces
        #   default: no roll allowed
        self.tau_ctrl[3] = 0.0

        # # trimming forces: add offsets from config (if any)
        # self.tau_ctrl[2] += self.offset_z  # depth
        # self.tau_ctrl[4] += self.offset_m  # pitch

        return self.tau_ctrl

    def __str__(self):
        return CONSOLE_STATUS % (
            super(HydridController, self).__str__(),
            self.req_vel, self.lim_vel,
            self.err_pos, self.err_pos_prev, self.delta_pos,
            self.err_vel, self.err_vel_der, self.err_vel_int,
            self.tau_ctrl
        )
