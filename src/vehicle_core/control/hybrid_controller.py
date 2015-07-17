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
#   Valerio De Carolis, Nicola Di Lecce

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
HYBRID_LIM_POS = 1.0        # meters
HYBRID_CLOSE = 3.0          # meters

CONSOLE_STATUS = """%s
  req_v: %s
  lim_v: %s
  ep: %s
  epp: %s
  dp: %s
  vp: %s
  evp: %s
  evd: %s
  evi: %s
  tau_c: %s
  ls: %s
"""


class HydridController(vc.VehicleController):

    def __init__(self, dt, ctrl_config, model_config, **kwargs):
        super(HydridController, self).__init__(dt, ctrl_config, **kwargs)

        # init params
        self.kpos = np.zeros_like(self.pos)
        self.kposprev = np.zeros_like(self.pos)
        self.lim_pos = np.ones_like(self.pos) * HYBRID_LIM_POS

        # scaling coefficients
        self.ku = self.dt / np.sqrt(2.0 * (self.lim_pos**2))
        self.kw = self.dt / self.lim_pos
        self.kr = 1 / np.tanh(0.5 * np.pi)
        self.kr_in = self.dt

        self.feedforward_model = False
        self.neuron_model = False
        self.model = None
        self.local_state = False

        # state
        self.des_pos_prev = np.zeros_like(self.pos)
        self.delta_pos = np.zeros_like(self.pos)
        self.sigma_pos = np.zeros_like(self.pos)
        self.virtual_pos = np.zeros_like(self.pos)

        # neuron membrane
        self.V = np.zeros_like(self.pos)
        self.A = np.ones_like(self.pos)
        self.B = np.ones_like(self.pos)
        self.D = np.ones_like(self.pos)
        self.act = np.zeros_like(self.pos)
        self.deact = np.zeros_like(self.pos)

        # errors
        self.err_pos = np.zeros_like(self.pos)
        self.err_pos_prev = np.zeros_like(self.pos)
        self.err_vel = np.zeros_like(self.vel)
        self.err_vel_prev = np.zeros_like(self.vel)
        self.err_vel_der = np.zeros_like(self.vel)
        self.err_vel_int = np.zeros_like(self.vel)

        # optional position pid
        self.epj = np.zeros_like(self.pos)
        self.epj_prev = np.zeros_like(self.pos)
        self.epj_der = np.zeros_like(self.pos)
        self.epj_int = np.zeros_like(self.pos)

        self.pos_Kp = np.zeros_like(self.pos)
        self.pos_Ki = np.zeros_like(self.pos)
        self.pos_Kd = np.zeros_like(self.pos)
        self.pos_lim = np.zeros_like(self.pos)

        # intermediate requests
        self.req_vel = np.zeros_like(self.pos)
        self.req_vel_prev = np.zeros_like(self.vel)
        self.tau_ctrl = np.zeros_like(self.pos)

        # init jacobian matrices
        self.J = np.zeros((6, 6))           # jacobian matrix (translate velocity from body frame to Earth frame)
        self.J_inv = np.zeros((6, 6))       # inverse jacobian matrix


    def update_config(self, ctrl_config, model_config):
        # sliding mode
        self.kpos = ctrl_config['kpos']
        self.kposprev = ctrl_config['kposprev']
        self.kgamma = ctrl_config['kgamma']
        self.lim_pos = ctrl_config['lim_pos']

        self.ku = ctrl_config['ku']
        self.kw = ctrl_config['kw']
        self.kr = ctrl_config['kr']

        # trimming offsets
        self.offset_z = float(ctrl_config.get('offset_z', 0.0))
        self.offset_m = float(ctrl_config.get('offset_m', 0.0))

        # vehicle model
        self.feedforward_model = bool(ctrl_config.get('feedforward_model', False))
        self.neuron_model = bool(ctrl_config.get('neuron_model', False))

        if self.feedforward_model:
            self.model = vm.VehicleModel(model_config)

        if self.neuron_model:
            self.A = np.array(ctrl_config['A'])
            self.B = np.array(ctrl_config['B'])
            self.D = np.array(ctrl_config['D'])

        # pid parameters (position)
        self.pos_Kp = np.array([
            ctrl_config['pos_x']['kp'],
            ctrl_config['pos_y']['kp'],
            ctrl_config['pos_z']['kp'],
            ctrl_config['pos_k']['kp'],
            ctrl_config['pos_m']['kp'],
            ctrl_config['pos_n']['kp'],
        ])

        self.pos_Kd = np.array([
            ctrl_config['pos_x']['kd'],
            ctrl_config['pos_y']['kd'],
            ctrl_config['pos_z']['kd'],
            ctrl_config['pos_k']['kd'],
            ctrl_config['pos_m']['kd'],
            ctrl_config['pos_n']['kd'],
        ])

        self.pos_Ki = np.array([
            ctrl_config['pos_x']['ki'],
            ctrl_config['pos_y']['ki'],
            ctrl_config['pos_z']['ki'],
            ctrl_config['pos_k']['ki'],
            ctrl_config['pos_m']['ki'],
            ctrl_config['pos_n']['ki'],
        ])

        self.pos_lim = np.array([
            ctrl_config['pos_x']['lim'],
            ctrl_config['pos_y']['lim'],
            ctrl_config['pos_z']['lim'],
            ctrl_config['pos_k']['lim'],
            ctrl_config['pos_m']['lim'],
            ctrl_config['pos_n']['lim'],
        ])

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
        self.err_pos = (self.pos - self.des_pos_prev)

        # check for proximity to goal
        if not self.local_state:
            if np.linalg.norm(self.err_pos[0:2]) < HYBRID_CLOSE:
                self.local_state = True

        if not np.allclose(self.des_pos, self.des_pos_prev):
            self.local_state = False


        # wrap angles and limit pitch
        self.err_pos[3:6] = cnv.wrap_pi(self.err_pos[3:6])
        self.err_pos[4] = np.clip(self.err_pos[4], -vc.MAX_PITCH, vc.MAX_PITCH)

        # update request position
        self.delta_pos = (self.des_pos - self.des_pos_prev)

        # wrap angles and limit pitch
        self.delta_pos[3:6] = cnv.wrap_pi(self.delta_pos[3:6])
        self.delta_pos[4] = np.clip(self.delta_pos[4], -vc.MAX_PITCH, vc.MAX_PITCH)

        # keep track of previous request
        self.des_pos_prev = self.des_pos


        # change the error if neuron is enabled
        if self.neuron_model:
            self.err_pos = np.clip(self.err_pos, -self.D, self.B)
            self.err_pos_prev = np.clip(self.err_pos_prev, -self.D, self.B)
            self.act = np.maximum(self.err_pos, 0.0)
            self.deact = np.maximum(-self.err_pos, 0.0)

            self.V = -self.A * self.err_pos_prev + (self.B - self.err_pos_prev) * self.act - (self.D + self.err_pos_prev) * self.deact
            self.err_pos += self.V * self.dt

            # wrap angles and limit pitch
            self.err_pos[3:6] = cnv.wrap_pi(self.err_pos[3:6])
            self.err_pos[4] = np.clip(self.err_pos[4], -vc.MAX_PITCH, vc.MAX_PITCH)
        else:
            # otherwise apply error limits
            self.err_pos = np.clip(self.err_pos, -self.lim_pos, self.lim_pos)


        # error dynamics
        self.vel_efec = np.dot(self.J, self.vel.reshape((6, 1))).flatten()
        self.err_pos_dot = (self.err_pos - self.delta_pos) + (self.dt * self.vel_efec)

        # sliding surface
        self.sigma_pos = self.err_pos_dot - self.err_pos + (self.kposprev * self.err_pos_prev)

        # anti-chattering
        self.gamma = self.kgamma * np.tanh(0.5 * self.sigma_pos)

        # virtual position
        self.virtual_pos = self.delta_pos - (self.kpos * self.err_pos) - (self.kposprev * self.err_pos_prev) - self.gamma

        self.gamma[5] = np.arctan2(self.virtual_pos[1], self.virtual_pos[0])
        self.virtual_pos[5] = self.delta_pos[5] - (self.kpos * self.err_pos[5]) # 2.2

        # calculate required velocity
        self.req_vel = np.zeros_like(self.vel)
        self.req_vel[0] = self.ku * np.linalg.norm(self.virtual_pos[0:2]) / self.dt
        self.req_vel[2] = self.kw * self.virtual_pos[2] / self.dt

        # # update sliding surface for yaw
        a = np.arctan2(self.virtual_pos[1], self.virtual_pos[0]) - self.pos[5]
        self.gamma[5] = self.kgamma * np.tanh(0.5 * cnv.wrap_pi(a))

        # calculate required velocity for yaw
        #d = self.kr * self.virtual_pos[5] / self.dt
        #self.req_vel[5] = self.krr * np.tanh(0.25 * cnv.wrap_pi(d) )

        d = self.kr_in * (np.arctan2(self.virtual_pos[1], self.virtual_pos[0]) - self.pos[5] - self.gamma[5]) / self.dt
        self.req_vel[5] = self.kr * np.tanh(0.5 * cnv.wrap_pi(d))

        # # limit u speed because of r speed
        # #   u = clip(u, umax(r))
        # #   umax(r) = umax * (1-r) / rmax
        # u_max = 1.0 * (1.0 - (np.abs(self.req_vel[5]) / self.kr))
        # self.req_vel[0] = np.clip(self.req_vel[0], -u_max, u_max)

        # save old position error
        self.err_pos_prev = self.err_pos


        # limit surge velocity when close to goal
        if self.local_state:
            self.ep = self.pos - self.des_pos
            self.epj = np.dot(self.J_inv, self.ep.reshape((6, 1))).flatten()

            # wrap angles and limit pitch
            self.epj[3:6] = cnv.wrap_pi(self.epj[3:6])
            self.epj[4] = np.clip(self.epj[4], -vc.MAX_PITCH, vc.MAX_PITCH)

            # update errors
            self.epj_der = (self.epj - self.epj_prev) / self.dt
            self.epj_int = np.clip(self.epj_int + self.epj, -self.pos_lim, self.pos_lim)

            # Position integral terms set to zero to avoid oscillations
            pos_changed = np.sign(self.epj) != np.sign(self.epj_prev)
            pos_changed[2] = False  # ignore the depth
            self.epj_int[pos_changed] = 0.0

            # update previous error
            self.epj_prev = self.epj

            # first pid output (plus speed limits if requested by the user)
            self.req_vel = (-self.pos_Kp * self.epj) + (-self.pos_Kd * self.epj_der) + (-self.pos_Ki * self.epj_int)


        # if running in velocity mode ignore the first pid
        if self.ctrl_mode == vc.MODE_VELOCITY:
            self.req_vel = self.des_vel

        # apply user velocity limits (if any)
        self.req_vel = np.clip(self.req_vel, -self.lim_vel, self.lim_vel)


        # backstepping controller
        #   pid (inner loop on velocity)
        self.err_vel = np.clip(self.req_vel - self.vel, -self.vel_input_lim, self.vel_input_lim)
        self.err_vel_der = (self.req_vel - self.vel) / self.dt
        self.err_vel_int = np.clip(self.err_vel_int + self.err_vel, -self.vel_lim, self.vel_lim)

        # velocity integral terms set to zero to avoid oscillations (ignore depth)
        vel_changed = np.sign(self.err_vel) != np.sign(self.err_vel_prev)
        vel_changed[2] = False
        self.err_vel_int[vel_changed] = 0.0

        # update previous request velocity
        self.req_vel_prev = self.req_vel

        # update previous error
        self.err_vel_prev = self.err_vel

        # pid output
        self.tau_ctrl = self.err_vel_der + (self.vel_Kp * self.err_vel) + (self.vel_Kd * self.err_vel_der) + (self.vel_Ki * self.err_vel_int)

        # use model only if the linearized model is disabled
        if self.feedforward_model:
            self.tau_model = self.model.update_forward_model(self.pos, self.vel)
            self.tau_ctrl = self.tau_ctrl + self.tau_model

        # hard limits on forces
        #   default: no roll allowed
        self.tau_ctrl[3] = 0.0

        # trimming forces: add offsets from config (if any)
        self.tau_ctrl[2] += self.offset_z  # depth
        self.tau_ctrl[4] += self.offset_m  # pitch

        return self.tau_ctrl

    def __str__(self):
        return CONSOLE_STATUS % (
            super(HydridController, self).__str__(),
            self.req_vel, self.lim_vel,
            self.err_pos, self.err_pos_prev, self.delta_pos, self.virtual_pos,
            self.err_vel, self.err_vel_der, self.err_vel_int,
            self.tau_ctrl, self.local_state
        )
