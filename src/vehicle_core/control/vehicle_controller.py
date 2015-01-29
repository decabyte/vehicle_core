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

from vehicle_core.model import vehicle_model as vm
from vehicle_core.model import dynamic_model as dm

# controller modes
MODE_POSITION = 0
MODE_VELOCITY = 1
MODE_STATION = 2

MAX_PITCH = np.deg2rad(60)  # max pitch (rad)


# utils
def wrap_pi(angle):
    return ((angle + np.pi) % (2 * np.pi)) - np.pi

#def wrap_2pi(angle):
#    return ((angle + np.pi) % (2*np.pi))


class VehicleController(object):
    """VehicleController class wraps the low-level control logic.

    This class represent the parent class used to describe the behaviour of a generic vehicle controller.
    Some standard features are implemented in this parent-class while specific approaches are implemented in children
    classes overriding interface methods.
    """

    def __init__(self, dt, config, **kwargs):
        # config
        self.dt = dt
        self.config = config

        # mode
        self.ctrl_mode = MODE_POSITION

        # states
        self.pos = np.zeros(6)
        self.vel = np.zeros(6)

        # requests
        self.des_pos = np.zeros(6)
        self.des_vel = np.zeros(6)

        # limits
        self.lim_vel = kwargs.get('lim_vel', 10 * np.ones(6))


    def update_config(self, ctrl_config, model_config):
        pass

    def update(self, position, velocity):
        return np.zeros(6)

    def __str__(self):
        return """controller:
          pos: %s
          des_p: %s
          vel: %s
          des_v: %s
          mode: %s
        """ % (
            self.pos, self.des_pos, self.vel, self.des_vel, self.ctrl_mode
        )


class CascadedController(VehicleController):
    """CascadedController implements a controller strategy based on the use of cascaded PID controllers.

    This class encapsulate a cascaded PID controller and exposes several methods to provide input for the
    controller architecture and to obtain the output, namely forces and thrusters commands, to control the vehicle.
    """

    def __init__(self, dt, ctrl_config, model_config, **kwargs):
        super(CascadedController, self).__init__(dt, ctrl_config, **kwargs)

        # init params
        self.pos_Kp = np.zeros(6)
        self.pos_Kd = np.zeros(6)
        self.pos_Ki = np.zeros(6)
        self.vel_Kp = np.zeros(6)
        self.vel_Kd = np.zeros(6)
        self.vel_Ki = np.zeros(6)

        self.pos_lim = np.zeros(6)
        self.vel_lim = np.zeros(6)

        self.offset_z = 0.0
        self.offset_m = 0.0
        self.feedforward_model = False
        self.linearized_model = False

        # intermediate requests
        self.req_vel = np.zeros(6)
        self.tau_ctrl = np.zeros(6)
        self.tau_prev = np.zeros(6)

        # errors
        self.err_pos = np.zeros(6)
        self.err_pos_prev = np.zeros(6)
        self.err_pos_der = np.zeros(6)
        self.err_pos_int = np.zeros(6)
        self.err_vel = np.zeros(6)
        self.err_vel_prev = np.zeros(6)
        self.err_vel_der = np.zeros(6)
        self.err_vel_int = np.zeros(6)

        self.err_intermediate = 0.0
        self.err_intermediate_prev = 0.0
        self.err_intermediate_der = 0.0
        self.err_intermediate_int = 0.0

        # init jacobians matrices
        self.J = np.zeros((6, 6))  # jacobian matrix (translate velocity from body referenced to Earth referenced)
        self.J_inv = np.zeros((6, 6))  # inverse jacobian matrix


    def update_config(self, ctrl_config, model_config):
        # trimming offsets
        self.offset_z = float(ctrl_config.get('offset_z', 0.0))
        self.offset_m = float(ctrl_config.get('offset_m', 0.0))

        # vehicle model
        self.feedforward_model = bool(ctrl_config.get('feedforward_model', False))
        self.linearized_model = bool(ctrl_config.get('linearized_model', False))

        # depth pitch control
        #self.depth_pitch_control = bool(ctrl_config.get('depth_pitch_control', False))

        if self.feedforward_model or self.linearized_model:
            self.model = vm.VehicleModel(model_config)

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

        # update jacobians
        self.J = dm.update_jacobian(self.J, self.pos[3], self.pos[4], self.pos[5])
        self.J_inv = np.linalg.inv(self.J)

        # model-free pid cascaded controller
        #   first pid (outer loop on position)
        self.err_pos = self.pos - self.des_pos
        self.err_pos = np.dot(self.J_inv, self.err_pos.reshape((6, 1))).flatten()

        # wrap angles and limit pitch
        self.err_pos[3:6] = wrap_pi(self.err_pos[3:6])
        self.err_pos[4] = np.clip(self.err_pos[4], -MAX_PITCH, MAX_PITCH)

        # update errors
        self.err_pos_der = (self.err_pos - self.err_pos_prev) / self.dt
        self.err_pos_int = np.clip(self.err_pos_int + self.err_pos, -self.pos_lim, self.pos_lim)
        self.err_pos_prev = self.err_pos

        # Position integral terms set to zero to avoid oscillations
        pos_changed = np.sign(self.err_pos) != np.sign(self.err_pos_prev)
        pos_changed[2] = False  # ignore the depth
        self.err_pos_int[pos_changed] = 0.0

        # first pid output (plus speed limits if requested by the user)
        self.req_vel = (-self.pos_Kp * self.err_pos) + (-self.pos_Kd * self.err_pos_der) + (
            -self.pos_Ki * self.err_pos_int)


        # if running in velocity mode ignore the first pid
        if self.ctrl_mode == MODE_VELOCITY:
            self.req_vel = self.des_vel

        # apply user velocity limits (if any)
        self.req_vel = np.clip(self.req_vel, -self.lim_vel, self.lim_vel)


        # model-free pid cascaded controller
        #   second pid (inner loop on velocity)
        self.err_vel = np.clip(self.vel - self.req_vel, -self.vel_input_lim, self.vel_input_lim)
        self.err_vel_der = (self.err_vel - self.err_vel_prev) / self.dt
        self.err_vel_int = np.clip(self.err_vel_int + self.err_vel, -self.vel_lim, self.vel_lim)
        self.err_vel_prev = self.err_vel

        # velocity integral terms set to zero to avoid oscillations
        vel_changed = np.sign(self.err_vel) != np.sign(self.err_vel_prev)
        vel_changed[2] = False  # ignore the depth
        self.err_vel_int[vel_changed] = 0.0

        # second pid output
        self.tau_ctrl = (-self.vel_Kp * self.err_vel) + (-self.vel_Kd * self.err_vel_der) + (-self.vel_Ki * self.err_vel_int)


        # # velocity depth control based on pitch control
        # if self.depth_pitch_control:
        #     self.err_intermediate = np.clip(self.pos[4] - self.tau_ctrl[2], -self.vel_input_lim[4], self.vel_input_lim[4])
        #     self.err_intermediate_der = (self.err_intermediate - self.err_intermediate_prev) / self.dt
        #     self.err_intermediate_int = np.clip(self.err_intermediate_int + self.err_intermediate, -self.vel_lim[4], self.vel_lim[4])
        #     self.err_intermediate_prev = self.err_intermediate
        #
        #     self.tau_ctrl[2] = (-self.vel_Kp[4] * self.err_intermediate) + (-self.vel_Kd[4] * self.err_intermediate_der) + (-self.vel_Ki[4] * self.err_intermediate_int)


        # linearized the plant is its model and the sensor measurements
        #   enabled only if the feed-forward controller is not used!
        if self.linearized_model and not self.feedforward_model:
            # calculate the acceleration due to the dynamic coupling forces acting on the vehicle using the sensors'
            # measurements without including the tau term (set to zero in this case)

            # use the output of the pid as an acceleration
            self.acc = self.tau_ctrl

            # rewrite the requested force using the dynamical model for linearizing the plant
            self.tau_ctrl = self.model.update_tau(self.pos, self.vel, self.acc)

        # use feed-forward controller only if the linearized model is disabled
        if self.feedforward_model and not self.linearized_model:
            self.tau_model = self.model.update_forward_model(self.des_pos, self.des_vel)
            self.tau_model[3] = 0  # ignore roll

            # feed-forward controller
            self.tau_ctrl = self.tau_ctrl + self.tau_model


        # pitch controller (TODO: add the pitch controller from the autotuning version)
        # self.tau_ctrl[4] = self.tau_ctrl[4] + 0.105 * np.abs(self.vel[0])*self.vel[0] + 35.6*np.sin(self.pos[4])

        # hard limits on forces
        #   default: no roll allowed
        self.tau_ctrl[3] = 0.0

        # trimming forces: add offsets from config (if any)
        self.tau_ctrl[2] += self.offset_z  # depth
        self.tau_ctrl[4] += self.offset_m  # pitch

        return self.tau_ctrl


    def __str__(self):
        model = 'disabled'

        if self.feedforward_model:
            model = 'feedfoward'

        if self.linearized_model:
            model = 'linearized'

        return """%s
          model: %s
          req_v: %s
          lim_v: %s
          ep: %s
          ed: %s
          ei: %s
          evp: %s
          evd: %s
          evi: %s
          tau_c: %s
        """ % (
            super(CascadedController, self).__str__(),
            model,
            self.req_vel, self.lim_vel,
            self.err_pos, self.err_pos_der, self.err_pos_int,
            self.err_vel, self.err_vel_der, self.err_vel_int,
            self.tau_ctrl
        )


class AutoTuningController(CascadedController):
    """AutoTuningController is ...

    """

    def __init__(self, dt, ctrl_config, model_config, **kwargs):
        super(AutoTuningController, self).__init__(dt, ctrl_config, model_config, **kwargs)

        # adaption coefficients for each DOF of vehicle
        self.adapt_coeff_pos = np.zeros(6)  # position
        self.adapt_coeff_vel = np.zeros(6)  # velocity
        self.adapt_limit_pos = np.zeros(3)
        self.adapt_limit_vel = np.zeros(3)
        self.pitch_surge_coeff = 0.0
        self.pitch_rest_coeff = 0.0
        self.tau_ctrl_prev = np.zeros(6)


    def update_config(self, ctrl_config, model_config):
        # load parameters from default controller
        super(AutoTuningController, self).update_config(ctrl_config, model_config)

        # adaptation coefficients
        self.adapt_coeff_pos = np.array([
            ctrl_config['adapt_coeff_pos']['x'],
            ctrl_config['adapt_coeff_pos']['y'],
            ctrl_config['adapt_coeff_pos']['z'],
            ctrl_config['adapt_coeff_pos']['k'],
            ctrl_config['adapt_coeff_pos']['m'],
            ctrl_config['adapt_coeff_pos']['n'],
        ])

        self.adapt_coeff_vel = np.array([
            ctrl_config['adapt_coeff_vel']['u'],
            ctrl_config['adapt_coeff_vel']['v'],
            ctrl_config['adapt_coeff_vel']['w'],
            ctrl_config['adapt_coeff_vel']['p'],
            ctrl_config['adapt_coeff_vel']['q'],
            ctrl_config['adapt_coeff_vel']['r'],
        ])

        self.adapt_limit_pos = np.array([
            ctrl_config['adapt_limit_pos']['p'],
            ctrl_config['adapt_limit_pos']['i'],
            ctrl_config['adapt_limit_pos']['d']
        ])

        self.adapt_limit_vel = np.array([
            ctrl_config['adapt_limit_vel']['p'],
            ctrl_config['adapt_limit_vel']['i'],
            ctrl_config['adapt_limit_vel']['d']
        ])

        # pitch controller parameters
        self.pitch_surge_coeff = float(ctrl_config.get('pitch_surge_coeff', 0.0))
        self.pitch_rest_coeff = float(ctrl_config.get('pitch_rest_coeff', 0.0))


    def update(self, position, velocity):
        # store nav updates
        self.pos = position
        self.vel = velocity

        # update jacobians
        self.J = dm.update_jacobian(self.J, self.pos[3], self.pos[4], self.pos[5])
        self.J_inv = np.linalg.inv(self.J)

        # PI position controller
        self.err_pos = self.pos - self.des_pos
        self.err_pos = np.dot(self.J_inv, self.err_pos.reshape(-1, 1)).flatten()

        # wrap angles and limit pitch
        self.err_pos[3:6] = wrap_pi(self.err_pos[3:6])
        self.err_pos[4] = np.clip(self.err_pos[4], -MAX_PITCH, MAX_PITCH)


        # update the errors
        self.err_pos_int = np.clip(self.err_pos_int + self.err_pos, -self.pos_lim, self.pos_lim)
        self.err_pos_der = (self.err_pos - self.err_pos_prev) / self.dt
        self.err_pos_prev = self.err_pos

        # adaptive tuning of position gains
        self.pos_Kp += self.adapt_coeff_pos * self.err_pos * np.abs(self.err_pos)
        self.pos_Ki += self.adapt_coeff_pos * self.err_pos * self.err_pos_int
        self.pos_Kd += self.adapt_coeff_pos * self.err_pos * self.err_pos_der

        self.pos_Kp = np.clip(self.pos_Kp, -self.adapt_limit_pos[0], self.adapt_limit_pos[0])
        self.pos_Ki = np.clip(self.pos_Ki, -self.adapt_limit_pos[1], self.adapt_limit_pos[1])
        self.pos_Kd = np.clip(self.pos_Kd, -self.adapt_limit_pos[2], self.adapt_limit_pos[2])

        # Position integral terms set to zero to avoid oscillations
        pos_changed = np.sign(self.err_pos) != np.sign(self.err_pos_prev)
        pos_changed[2] = False  # ignore the depth
        self.err_pos_int[pos_changed] = 0.0

        # PI controller limited (outer loop on position) - velocity
        self.req_vel = (-np.abs(self.pos_Kp) * self.err_pos) + (-np.abs(self.pos_Ki) * self.err_pos_int) + (
            -np.abs(self.pos_Kd) * self.err_pos_der)


        # if running in velocity mode ignore the first pid
        if self.ctrl_mode == MODE_VELOCITY:
            self.req_vel = self.des_vel

        # apply user velocity limits (if any)
        self.req_vel = np.clip(self.req_vel, -self.lim_vel, self.lim_vel)


        # velocity errors
        self.err_vel = np.clip(self.vel - self.req_vel, -self.vel_input_lim, self.vel_input_lim)
        self.err_vel_int = np.clip(self.err_vel_int + self.err_vel, -self.vel_lim, self.vel_lim)
        self.err_vel_der = (self.err_vel - self.err_vel_prev) / self.dt
        self.err_vel_prev = self.err_vel

        # adaptive tuning of velocity gains
        self.vel_Kp += self.adapt_coeff_vel * self.err_vel * np.abs(self.err_vel)
        self.vel_Ki += self.adapt_coeff_vel * self.err_vel * self.err_vel_int
        self.vel_Kd += self.adapt_coeff_vel * self.err_vel * self.err_vel_der

        self.vel_Kp = np.clip(self.vel_Kp, -self.adapt_limit_vel[0], self.adapt_limit_vel[0])
        self.vel_Ki = np.clip(self.vel_Ki, -self.adapt_limit_vel[1], self.adapt_limit_vel[1])
        self.vel_Kd = np.clip(self.vel_Kd, -self.adapt_limit_vel[2], self.adapt_limit_vel[2])

        # Velocity integral terms set to zero to avoid oscillations
        vel_changed = np.sign(self.err_vel) != np.sign(self.err_vel_prev)
        vel_changed[2] = False  # ignore the depth
        self.err_vel_int[vel_changed] = 0.0

        # PI controller velocity
        self.tau_ctrl = (-np.abs(self.vel_Kp) * self.err_vel) + (-np.abs(self.vel_Ki) * self.err_vel_int) + (
            -np.abs(self.vel_Kd) * self.err_vel_der)

        # use feed-forward controller only if the linearized model is disabled
        if self.feedforward_model and not self.linearized_model:
            self.tau_model = self.model.update_forward_model(self.des_pos, self.des_vel)
            self.tau_model[3] = 0  # ignore roll

            # feed-forward controller
            self.tau_ctrl = self.tau_ctrl + self.tau_model

        if self.linearized_model and not self.feedforward_model:
            # calculate the acceleration due to the dynamic coupling forces acting on the vehicle using the sensors'
            # measurements without including the tau term (set to zero in this case)

            # use the output of the pid as an acceleration
            self.acc = self.tau_ctrl

            # rewrite the requested force using the dynamical model for linearizing the plant
            self.tau_ctrl = self.model.update_tau(self.pos, self.vel, self.acc)

            # TODO: remove system oscillations and add pitch control for linearized version
            # ...

        # pitch controller
        self.tau_ctrl[4] = self.tau_ctrl[4] + self.pitch_surge_coeff * np.abs(self.vel[0]) * self.vel[
            0] + self.pitch_rest_coeff * np.sin(self.pos[4])

        # hard limits on forces
        #   default: no roll allowed
        self.tau_ctrl[3] = 0.0

        # trimming forces: add offsets from config (if any)
        self.tau_ctrl[2] += self.offset_z  # depth
        self.tau_ctrl[4] += self.offset_m  # pitch

        return self.tau_ctrl


    def __str__(self):
        return """%s
          pos_kp: %s
          pos_kd: %s
          pos_ki: %s
          vel_kp: %s
          vel_kd: %s
          vel_ki: %s
        """ % (
            super(AutoTuningController, self).__str__(),
            self.pos_Kp, self.pos_Kd, self.pos_Ki,
            self.vel_Kp, self.vel_Kd, self.vel_Ki,
        )
