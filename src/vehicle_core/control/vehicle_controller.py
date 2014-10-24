#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

from vehicle_core.model import vehicle_model as vm
from vehicle_core.model import dynamic_model as dm


# controller modes
MODE_POSITION = 0
MODE_VELOCITY = 1
MODE_STATION = 2

MAX_PITCH = np.deg2rad(60)                      # max pitch (rad)


# utils
def wrap_pi(angle):
    return ((angle + np.pi) % (2*np.pi)) - np.pi

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
        self.model_based = False

        # intermediate requests
        self.req_vel = np.zeros(6)
        self.tau_ctrl = np.zeros(6)

        # errors
        self.err_pos = np.zeros(6)
        self.err_pos_prev = np.zeros(6)
        self.err_pos_der = np.zeros(6)
        self.err_pos_int = np.zeros(6)
        self.err_vel = np.zeros(6)
        self.err_vel_prev = np.zeros(6)
        self.err_vel_der = np.zeros(6)
        self.err_vel_int = np.zeros(6)

        # init jacobians matrices
        self.J = np.zeros((6,6))     # jacobian matrix (translate velocity from body referenced to Earth referenced)
        self.J_inv = np.zeros((6,6)) # inverse jacobian matrix

        # load controller configuration
        #self.update_config(ctrl_config, model_config)


    def update_config(self, ctrl_config, model_config):
        # trimming offsets
        self.offset_z = float(ctrl_config.get('offset_z', 0.0))
        self.offset_m = float(ctrl_config.get('offset_m', 0.0))

        # vehicle model
        self.model_based = bool(ctrl_config.get('model_based', False))

        if self.model_based:
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
        self.err_pos = np.dot( self.J_inv, self.err_pos.reshape((6,1)) ).flatten()

        # wrap angles and limit pitch
        self.err_pos[3:6] = wrap_pi(self.err_pos[3:6])
        self.err_pos[4] = np.clip(self.err_pos[4], -MAX_PITCH, MAX_PITCH)

        # update errors
        self.err_pos_der = (self.err_pos - self.err_pos_prev) / self.dt
        self.err_pos_int = np.clip(self.err_pos_int + self.err_pos, -self.pos_lim, self.pos_lim)
        self.err_pos_prev = self.err_pos


        # first pid output (plus speed limits if requested by the user)
        self.req_vel = (-self.pos_Kp * self.err_pos) + (-self.pos_Kd * self.err_pos_der) + (-self.pos_Ki * self.err_pos_int)


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

        # second pid output
        self.tau_ctrl = (-self.vel_Kp * self.err_vel) + (-self.vel_Kd * self.err_vel_der) + (-self.vel_Ki * self.err_vel_int)


        # trimming forces: add offsets from config (if any)
        self.tau_ctrl[2] += self.offset_z       # depth
        self.tau_ctrl[4] += self.offset_m       # pitch

        # use vehicle model if enabled
        if self.model_based:
            self.tau_model = self.model.update_model(self.pos, self.vel)
            self.tau_model[3] = 0   # ignore roll

            self.tau_ctrl = self.tau_ctrl - self.tau_model

        # hard limits on forces
        #   default: no roll allowed
        self.tau_ctrl[3] = 0.0

        return self.tau_ctrl


    def __str__(self):
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
            self.model_based, self.req_vel, self.lim_vel,
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


    def update_config(self, ctrl_config, model_config):
        # trimming offsets
        self.offset_z = float(ctrl_config.get('offset_z', 0.0))
        self.offset_m = float(ctrl_config.get('offset_m', 0.0))

        # vehicle model
        self.model_based = bool(ctrl_config.get('model_based', False))

        if self.model_based:
            self.model = vm.VehicleModel(model_config)


        # adaptation coefficients
        self.alpha = float(ctrl_config.get('adapt_alpha', 0.0))

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
            ctrl_config['adapt_coeff_vel']['r'],
            ctrl_config['adapt_coeff_vel']['p'],
            ctrl_config['adapt_coeff_vel']['q'],
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

        self.pos_lim = np.array([
            ctrl_config['pos_x']['lim'],
            ctrl_config['pos_y']['lim'],
            ctrl_config['pos_z']['lim'],
            ctrl_config['pos_k']['lim'],
            ctrl_config['pos_m']['lim'],
            ctrl_config['pos_n']['lim'],
        ])

        self.vel_lim = np.array([
            ctrl_config['vel_u']['lim'],
            ctrl_config['vel_v']['lim'],
            ctrl_config['vel_w']['lim'],
            ctrl_config['vel_p']['lim'],
            ctrl_config['vel_q']['lim'],
            ctrl_config['vel_r']['lim'],
        ])


    def update(self, position, velocity):
        # store nav updates
        self.pos = position
        self.vel = velocity

        # update jacobians
        self.J = dm.update_jacobian(self.J, self.pos[3], self.pos[4], self.pos[5])
        self.J_inv = np.linalg.inv(self.J)

        # PI position controller
        self.err_pos = self.pos - self.des_pos
        self.err_pos = np.dot(self.J_inv, self.err_pos.reshape(-1,1)).flatten()

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


        # PI controller limited (outer loop on position) - velocity
        self.req_vel = (-np.abs(self.pos_Kp) * self.err_pos) + (-np.abs(self.pos_Ki) * self.err_pos_int) + (-np.abs(self.pos_Kd) * self.err_pos_der)


        # if running in velocity mode ignore the first pid
        if self.ctrl_mode == MODE_VELOCITY:
            self.req_vel = self.des_vel

        # apply user velocity limits (if any)
        self.req_vel = np.clip(self.req_vel, -self.lim_vel, self.lim_vel)


        # velocity errors
        self.err_vel = self.vel - self.req_vel
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


        # PI controller velocity
        self.tau_ctrl = (-np.abs(self.vel_Kp) * self.err_vel) + (-np.abs(self.vel_Ki) * self.err_vel_int) + (-np.abs(self.vel_Kd) * self.err_vel_der)


        # trimming forces: add offsets from config (if any)
        self.tau_ctrl[2] += self.offset_z       # depth
        self.tau_ctrl[4] += self.offset_m       # pitch

        # use vehicle model if enabled
        if self.model_based:
            self.tau_model = self.model.update_model(self.pos, self.vel)
            self.tau_model[3] = 0   # ignore roll

            self.tau_ctrl = self.tau_ctrl - self.tau_model

        # hard limits on forces
        #   default: no roll allowed
        self.tau_ctrl[3] = 0.0

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


if __name__ == '__main__':
    pass
