#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import argparse
import traceback
import numpy as np
np.set_printoptions(precision=3, suppress=True)

from vehicle_core.config import thrusters_config as tc
from vehicle_core.model import thruster_model as tm
from vehicle_core.model import dynamic_model as dm
from vehicle_core.control import thrust_allocation as ta
from vehicle_core.control import vehicle_controller as vc

import rospy
import roslib
roslib.load_manifest('vehicle_core')

from auv_msgs.msg import NavSts
from diagnostic_msgs.msg import KeyValue

from vehicle_interface.msg import ThrusterCommand, PilotStatus, PilotRequest, Vector6Stamped, FloatArrayStamped, Vector6, Vector6ArrayStamped
from vehicle_interface.srv import BooleanService, BooleanServiceResponse, FloatService, FloatServiceResponse


# config
DEFAULT_RATE = 10                           # Hz
STATUS_RATE = 2                             # Hz

MAX_PITCH = np.deg2rad(60)                      # max pitch (rad)
MAX_SPEED = np.array([1.5, 1, 1, 0, 2, 2])      # max speed (m/s and rad/s)


# controller status
CTRL_DISABLED = 0
CTRL_ENABLED = 1

STATUS_CTRL = {
    CTRL_DISABLED: PilotStatus.PILOT_DISABLED,
    CTRL_ENABLED: PilotStatus.PILOT_ENABLED
}

STATUS_MODE = {
    vc.MODE_POSITION: PilotStatus.MODE_POSITION,
    vc.MODE_VELOCITY: PilotStatus.MODE_VELOCITY,
    vc.MODE_STATION: PilotStatus.MODE_STATION
}


# ros topics
TOPIC_NAV = 'nav/nav_sts'
TOPIC_CMD = 'thrusters/commands'
TOPIC_STATUS = 'pilot/status'
TOPIC_FORCES = 'pilot/forces'

TOPIC_POS_REQ = 'pilot/position_req'
TOPIC_BODY_REQ = 'pilot/body_req'
TOPIC_VEL_REQ = 'pilot/velocity_req'
TOPIC_STAY_REQ = 'pilot/stay_req'

TOPIC_USER = 'user/forces'
SRV_SWITCH = 'pilot/switch'
SRV_SPEED_LIM = 'pilot/speed_limits'
SRV_THRUSTERS = 'thrusters/switch'


# adaptive fault reaction
TOPIC_METRIC = 'thrusters/diagnostics'      # read the diagnostic metric (if any)
SRV_FAULT_CTRL = 'pilot/fault_control'      # enable/disable the adaptive fault thruster reallocation
SRV_FAULT_SPEEDS = 'pilot/fault_speeds'     # enable/disable the adaptive fault speeds correction

# autotuning
TOPIC_GAINS = 'controller/gains'


THRESH_METRIC = np.array([0.9, 10, 10, 0.9, 10, 10])        # threshold for thrusters diagnostic metric
W_ADPT_RATE = 0.01                                          # thruster weight adaptation rate
W_THRS = np.array([0.3, 0.3, 0.3, 0.3, 0.3, 0.3])           # thruster exclusion threshold (% of reference design)


# utils
def wrap_pi(angle):
    return ((angle + np.pi) % (2*np.pi)) - np.pi

#def wrap_2pi(angle):
#    return ((angle + np.pi) % (2*np.pi))


# TODO: add a counter to user requests in order to avoid remembering very old requests for a long time
# TODO: if no requests are sent within a given timeout just disable the low-level controller and float!

class VehiclePilot(object):
    """VehiclePilot class represent the ROS interface for the pilot subsystem.

    This class implements all the required business logic to keep the vehicle within operative limits and requested
    behaviour. The pilot doesn't implement a low-level controller itself but uses one of the VehicleController implementations
    available in the vehicle_pilot module, thus making the vehicle control strategy easy to swap and separated from the main
    interfacing logic. This class parses the user requests, the joystick requests and any other request coming from external
    modules maintaining an internal status representing the navigation behaviour of the vehicle.

    In this class several safety features can be implemented, mainly preventing a faulty or non publishing nav, to mess with
    the low-level controller.
    """

    def __init__(self, name, pilot_rate, **kwargs):
        self.name = name
        self.pilot_rate = pilot_rate

        self.topic_output = kwargs.get('topic_output', TOPIC_CMD)
        self.throttle_limit = kwargs.get('throttle_limit', tc.MAX_THROTTLE)
        self.verbose = kwargs.get('verbose', False)

        # timing
        self.dt = 1.0 / self.pilot_rate
        self.pilot_loop = rospy.Rate(self.pilot_rate)
        self.pilot_status = rospy.Timer(rospy.Duration(1.0 / STATUS_RATE), self.send_status)

        # initial status
        self.ctrl_status = CTRL_DISABLED
        self.ctrl_mode = vc.MODE_POSITION
        self.lim_vel_user = MAX_SPEED
        self.lim_vel_ctrl = MAX_SPEED
        self.disable_axis = np.zeros(6)


        self.local_TAM = np.copy(tc.TAM)                        # thrust allocation matrix
        self.local_inv_TAM = np.copy(tc.inv_TAM)                # inverse of thrust allocation matrix
        self.thruster_efficiency = np.ones(tc.TAM.shape[1])     # thrusters efficiency (0 faulty to 1 healthy)

        self.pos = np.zeros(6)
        self.vel = np.zeros(6)
        self.des_pos = np.zeros(6)
        self.des_vel = np.zeros(6)
        self.err_pos = np.zeros(6)
        self.err_vel = np.zeros(6)

        # controller placeholder
        self.controller = None
        self.ctrl_type = None

        # load configuration
        self.reload_config()

        # outputs
        self.tau_ctrl = np.zeros(6, dtype=np.float64)         # u = [X, Y, Z, K, M, N]
        self.tau_user = np.zeros(6, dtype=np.float64)
        self.tau_total = np.zeros(6, dtype=np.float64)
        self.forces = np.zeros(6, dtype=np.float64)
        self.tau_tam = np.zeros(6, dtype=np.float64)
        self.throttle = np.zeros(6, dtype=np.float64)

        # alpha mapping for mixing thruster allocation matrices
        self.thres_fast_speed = 1.0
        self.thres_slow_speed = 0.0
        self.speed_m = 1.0 / np.abs(self.thres_fast_speed - self.thres_slow_speed)
        self.speed_q = -self.speed_m * self.thres_slow_speed

        # ros interface
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_nav, tcp_nodelay=True, queue_size=1)
        self.sub_user = rospy.Subscriber(TOPIC_USER, Vector6Stamped, self.handle_user, tcp_nodelay=True, queue_size=1)
        self.pub_status = rospy.Publisher(TOPIC_STATUS, PilotStatus, tcp_nodelay=True, queue_size=1)
        self.pub_forces = rospy.Publisher(TOPIC_FORCES, Vector6Stamped, tcp_nodelay=True, queue_size=1)
        self.pub_thr = rospy.Publisher(self.topic_output, ThrusterCommand, tcp_nodelay=True, queue_size=3)

        # pilot requests
        self.sub_pos_req = rospy.Subscriber(TOPIC_POS_REQ, PilotRequest, self.handle_pos_req, tcp_nodelay=True, queue_size=1)
        self.sub_body_req = rospy.Subscriber(TOPIC_BODY_REQ, PilotRequest, self.handle_body_req, tcp_nodelay=True, queue_size=1)
        self.sub_vel_req = rospy.Subscriber(TOPIC_VEL_REQ, PilotRequest, self.handle_vel_req, tcp_nodelay=True, queue_size=1)

        # services
        self.s_switch = rospy.Service(SRV_SWITCH, BooleanService, self.srv_switch)
        self.s_lim = rospy.Service(SRV_SPEED_LIM, FloatService, self.srv_speed_limits)

        # adaptive thruster allocation matrix and thruster usage weights
        self.diagnostic_metric = np.zeros(6)
        self.available_forces = np.copy(tc.MAX_U)

        # diagnostics
        self.sub_diag = rospy.Subscriber(TOPIC_METRIC, FloatArrayStamped, self.handle_diagnostics, tcp_nodelay=True, queue_size=3)
        self.s_fault_ctrl = rospy.Service(SRV_FAULT_CTRL, BooleanService, self.srv_fault_ctrl)
        self.s_fault_speeds = rospy.Service(SRV_FAULT_SPEEDS, BooleanService, self.srv_fault_speeds)

        # gains
        self.pub_gains = rospy.Publisher(TOPIC_GAINS, Vector6ArrayStamped, tcp_nodelay=True, queue_size=1)

        # optional info
        if self.verbose:
            t_pri = rospy.Timer(rospy.Duration(0.5), self.print_info)



    def reload_config(self, user_config=None):
        """This functions parses the configuration for the pilot and the low-level controller.

        It uses a configuration dictionary if provided, otherwise it queries the ROS param server for loading the latest
        version of the configuration parameters.

        :param user_config: user configuration dictionary (optional)
        """
        try:
            pilot_config = rospy.get_param('pilot', None)
        except Exception:
            tb = traceback.format_exc()
            rospy.logerr('%s: error in loading the configuration from ROS param server:\n%s', self.name, tb)
            return

        # pilot params
        self.pitch_enable = bool(pilot_config.get('pitch_enable', False))
        self.prioritize_axis = bool(pilot_config.get('prioritize_axis', False))

        self.adaptive_yaw = bool(pilot_config.get('adaptive_yaw', False))
        self.fault_control = bool(pilot_config.get('fault_control', False))
        self.fault_speeds = bool(pilot_config.get('fault_speeds', False))

        self.thres_fast_speed = float(pilot_config.get('threshold_fast', 1.0))      # high speed manoeuvring above this fwd speed
        self.thres_slow_speed = float(pilot_config.get('threshold_slow', 0.25))     # low speed manoeuvring below this fwd speed

        self.thres_fast_speed = np.clip(self.thres_fast_speed, 0, MAX_SPEED[0])
        self.thres_slow_speed = np.clip(self.thres_slow_speed, 0, MAX_SPEED[0])

        # alpha mapping for mixing thruster allocation matrices
        self.speed_m = 1.0 / np.abs(self.thres_fast_speed - self.thres_slow_speed)
        self.speed_q = -self.speed_m * self.thres_slow_speed

        if self.thres_fast_speed <= self.thres_slow_speed:
            self.adaptive_yaw = False

            rospy.logwarn('%s: adaptive yaw disabled due to wrong threshold params:\nfast: %s\nslow: %s',
                self.name, self.thres_fast_speed, self.thres_slow_speed)

        # update controller params
        self.ctrl_config = rospy.get_param('pilot/controller', dict())
        self.model_config = rospy.get_param('vehicle/model', dict())


        # controller selection (if a new controller has been requested by user)
        if self.ctrl_type != self.ctrl_config.get('type', 'cascaded'):

            # store new type
            self.ctrl_type = self.ctrl_config.get('type', 'cascaded')

            # create new controller
            if self.ctrl_type == 'cascaded':
                self.controller = vc.CascadedController(self.dt, self.ctrl_config, self.model_config, lim_vel=MAX_SPEED)
            elif self.ctrl_type == 'autotuning':
                self.controller = vc.AutoTuningController(self.dt, self.ctrl_config, self.model_config, lim_vel=MAX_SPEED)
            else:
                rospy.logfatal('controller type [%s] not supported', self.ctrl_type)
                raise ValueError('controller type [%s] not supported', self.ctrl_type)

            # notify the selection
            rospy.loginfo('%s: enabling %s controller ...', self.name, self.ctrl_type)

        # load or reload controller configuration
        self.controller.update_config(self.ctrl_config, self.model_config)



    # safety switch service
    def srv_switch(self, req):
        """This function handles the switch service.

        This will enable/disable the low-level controller, leaving the pilot parsing only the user input if disabled.
        Upon enabling this function reloads the configuration from the ROS param server, providing a quick way to change
        parameters in order to adjust the behaviour of the pilot or the low-level controller.

        Upon re-enabling the pilot will use the last known values for mode, desired position and velocities.
        """
        if req.request is True:
            # reload configuration
            self.reload_config()

            # enable the low-level controller
            self.ctrl_status = CTRL_ENABLED

            # reset axis
            self.disable_axis = np.zeros(6)

            return BooleanServiceResponse(True)
        else:
            self.ctrl_status = CTRL_DISABLED
            return BooleanServiceResponse(False)


    # speed limit service
    def srv_speed_limits(self, data):
        if len(data.request) == 6:
            self.lim_vel_ctrl = np.array(data.request[0:6])
            self.check_inputs()
        else:
            rospy.logwarn('%s: resetting speed limits', self.name)
            self.lim_vel_ctrl = MAX_SPEED

        rospy.logdebug('%s: set controller speed limits: %s', self.name, self.lim_vel_ctrl)
        return FloatServiceResponse(result=True, response=self.lim_vel_ctrl.tolist())


    # fault services
    def srv_fault_ctrl(self, data):
        self.fault_control = bool(data.request)

        # reset adaptive fault reaction on disable request
        if not self.fault_control:
            self.thruster_efficiency = np.ones(6)
            rospy.logwarn('%s: resetting adaptive fault control', self.name)

        rospy.logwarn('%s: adaptive fault control status: %s', self.name, self.fault_control)
        return BooleanServiceResponse(self.fault_control)

    def srv_fault_speeds(self, data):
        self.fault_speeds = bool(data.request)

        if not self.fault_speeds:
            self.lim_vel_ctrl = MAX_SPEED

        rospy.logwarn('%s: adaptive fault speeds status: %s', self.name, self.fault_speeds)
        return BooleanServiceResponse(self.fault_control)



    def handle_diagnostics(self, data):
        if len(data.values) == 6:
            self.diagnostic_metric = np.array(data.values[0:6])
        else:
            self.diagnostic_metric = np.zeros(6)

        # trigger only if adaptive correction is enabled
        # and a fault condition is detected (threshold crossing)
        if self.fault_control:
            indexes = np.where(self.diagnostic_metric > THRESH_METRIC)[0]

            # update the efficiency and costs
            self.thruster_efficiency[indexes] -= W_ADPT_RATE                        # reduce thruster efficiency
            self.thruster_efficiency = np.maximum(self.thruster_efficiency, 0)      # prevent negative values

            # check if is better to exclude inefficient thrusters
            if np.any(self.thruster_efficiency <= W_THRS):
                idx_disable = np.where(self.thruster_efficiency <= W_THRS)[0]
                self.thruster_efficiency[idx_disable] = 0



    def handle_user(self, data):
        # read user input
        if len(data.values) == 6:
            user_input = np.array(data.values)
        else:
            user_input = np.zeros(6)

        # limit user input
        self.tau_user = np.clip(user_input, -tc.MAX_U, tc.MAX_U)


    # TODO: prevent zero or missing nav data to mess with the pilot node!
    def handle_nav(self, data):
        # parse navigation data
        self.pos = np.array([
            data.position.north,
            data.position.east,
            data.position.depth,
            data.orientation.roll,
            data.orientation.pitch,
            data.orientation.yaw
        ])

        self.vel = np.array([
            data.body_velocity.x,
            data.body_velocity.y,
            data.body_velocity.z,
            data.orientation_rate.roll,
            data.orientation_rate.pitch,
            data.orientation_rate.yaw
        ])

        # populate errors (used for info only)
        self.err_pos = self.pos - self.des_pos
        self.err_vel = self.vel - self.des_vel


    def check_inputs(self):
        # position input checks:
        #   prevent negative depths (out of the water)
        #   remove roll
        #   wrap angles if necessary
        self.des_pos[2] = np.maximum(self.des_pos[2], 0)
        self.des_pos[3] = 0
        self.des_pos[3:6] = wrap_pi(self.des_pos[3:6])

        # velocity input checks:
        #   enforce speed limits (hurra!)
        #   remove roll
        self.des_vel = np.clip(self.des_vel, -MAX_SPEED, MAX_SPEED)     # m/s and rad/s
        self.des_vel[3] = 0

        # limits input checks:
        #   prevent this go above the maximum rated speed
        self.lim_vel_user = np.clip(self.lim_vel_user, -MAX_SPEED, MAX_SPEED)
        self.lim_vel_ctrl = np.clip(self.lim_vel_ctrl, -MAX_SPEED, MAX_SPEED)



    def handle_pos_req(self, data):
        try:
            # global referenced request
            self.des_pos = np.array(data.position[0:6])
            self.des_vel = np.zeros(6)
            self.lim_vel_user = MAX_SPEED

            # ignore disabled axis
            self.disable_axis = np.array(data.disable_axis)

            # optionally apply speeds limits if requested by the user
            if len(data.limit_velocity) == 6:
                idx_vel = np.array(data.limit_velocity) > 0
                self.lim_vel_user[idx_vel] = np.array(data.velocity)[idx_vel]

            self.check_inputs()
            self.ctrl_mode = vc.MODE_POSITION
        except TypeError:
            pass    # BUG: remove this clause after updating numpy in Nessie (bug in 1.6.1)
        except Exception:
            tb = traceback.format_exc()
            rospy.logerr('%s: bad position request\n%s', self.name, tb)



    def handle_body_req(self, data):
        try:
            # body referenced request
            J = dm.compute_jacobian(0, 0, self.pos[5])

            body_request = np.dot(J, np.array(data.position[0:6]))

            self.des_pos = self.pos + body_request
            self.des_vel = np.zeros(6)
            self.lim_vel_user = MAX_SPEED

            # ignore disabled axis
            self.disable_axis = np.array(data.disable_axis)

            # optionally apply speeds limits if requested by the user
            if len(data.limit_velocity) == 6:
                idx_vel = np.array(data.limit_velocity) > 0
                self.lim_vel_user[idx_vel] = np.array(data.velocity)[idx_vel]

            self.check_inputs()
            self.ctrl_mode = vc.MODE_POSITION
        except TypeError:
            pass    # BUG: remove this clause after updating numpy in Nessie (bug in 1.6.1)
        except Exception:
            tb = traceback.format_exc()
            rospy.logerr('%s: bad body request\n%s', self.name, tb)


    def handle_vel_req(self, data):
        try:
            self.des_pos = np.zeros(6)
            self.des_vel = np.array(data.velocity[0:6])
            self.lim_vel_user = MAX_SPEED

            # ignore disabled axis
            self.disable_axis = np.array(data.disable_axis)

            self.check_inputs()
            self.ctrl_mode = vc.MODE_VELOCITY
        except Exception:
            tb = traceback.format_exc()
            rospy.logerr('%s: bad velocity request\n%s', self.name, tb)



    def send_status(self, event=None):
        ps = PilotStatus()
        ps.header.stamp = rospy.Time.now()

        ps.status = STATUS_CTRL[self.ctrl_status]
        ps.mode = STATUS_MODE[self.ctrl_mode]
        ps.des_pos = self.des_pos.tolist()
        ps.des_vel = self.des_vel.tolist()
        ps.err_pos = self.err_pos.tolist()
        ps.err_vel = self.err_vel.tolist()

        ps.lim_vel_ctrl = self.lim_vel_ctrl.tolist()
        ps.lim_vel_user = self.lim_vel_user.tolist()

        ps.available_forces = self.available_forces
        ps.thruster_efficiency = self.thruster_efficiency

        self.pub_status.publish(ps)


        # added for the autotuning controller test
        if self.ctrl_type == 'autotuning':
            self.send_gains()


    # added for the autotuning controller test
    def send_gains(self, event=None):

        if self.ctrl_type != 'autotuning':
            return

        va = Vector6ArrayStamped()
        gains = [
            self.controller.pos_Kp,
            self.controller.pos_Kd,
            self.controller.pos_Ki,
            self.controller.vel_Kp,
            self.controller.vel_Kd,
            self.controller.vel_Ki,
        ]

        for g in gains:
            va.values.append(Vector6(g))

        self.pub_gains.publish(va)



    def loop(self):
        # init commands
        self.throttle = np.zeros(6)
        self.tau_total = np.zeros(6)
        self.tau_ctrl = np.zeros(6)

        # run the low-level control only if enabled explicitly
        if self.ctrl_status == CTRL_ENABLED:
            # set controller
            self.controller.des_pos = self.des_pos
            self.controller.des_vel = self.des_vel
            self.controller.ctrl_mode = self.ctrl_mode

            # select the strict constraint on vehicle speeds
            #   can be set on each request (ie. by the path planner module)
            #   can be set asynchronously (using service requests)
            self.controller.lim_vel = np.minimum(self.lim_vel_user, self.lim_vel_ctrl)

            # get computed forces
            self.tau_ctrl = self.controller.update(self.pos, self.vel)

            # avoid pitch is not explicitly request by config
            if not self.pitch_enable:
                self.tau_ctrl[4] = 0.0

            # disable control of specific axis (if requested by user)
            idx_dis = np.where(self.disable_axis == 1)
            self.tau_ctrl[idx_dis] = 0


        # compute the total force
        self.tau_total = self.tau_ctrl + self.tau_user

        # clip the total force using the local copy of maximum allowed force
        #self.u_total = np.clip(self.u_total, -tc.MAX_U, tc.MAX_U)

        # thruster efficiency for current speed
        local_efficiency = np.copy(self.thruster_efficiency)

        # assign a different cost to lateral thrusters depending on forward speed
        #   thruster allocation algorithm will reduce the use of lateral thrusters for yawing at high speeds
        #   not active if fault mitigation is active
        if self.adaptive_yaw and not self.fault_control:
            alpha = self.speed_m * self.vel[0] + self.speed_q
            alpha = np.clip(alpha, 0, 1)

            local_efficiency[2:4] = 1 - alpha   # efficiency is 0 at high speeds and 1 at low speeds


        # update the inverse thruster allocation matrix based on current thruster efficiency
        self.local_inv_TAM = ta.tam_weighted_inverse(self.local_TAM, local_efficiency)


        # update speeds limits using available thrust and efficiencies
        if self.fault_speeds:
            self.available_forces = ta.evaluate_max_force(self.local_inv_TAM)

            # avoid divisions by zero
            for dof in np.argwhere(tc.MAX_U != 0):
                self.lim_vel_ctrl[dof] = MAX_SPEED * np.sqrt(self.available_forces[dof] / tc.MAX_U[dof])

            self.lim_vel_ctrl = np.clip(self.lim_vel_ctrl, 0, MAX_SPEED)


        # convert force to thrust using the local copy of the TAM
        self.forces = np.dot( self.local_inv_TAM, self.tau_total )

        # allocate thrusters preventing saturation and prioritizing axes (yaw, sway, surge)
        #   otherwise use the naive approach (see thrust_allocation.py)
        if self.prioritize_axis:
            self.forces_sat = ta.priority_allocation(self.tau_total, self.local_inv_TAM)
        else:
            self.forces_sat = ta.saturation_allocation(self.tau_total, self.local_inv_TAM)

        # use inverse thruster model to compensate non linear effects
        self.throttle = tm.estimate_throttle(self.forces_sat, tc.THRUST_TO_THROTTLE, tc.LINEAR_THROTTLE, tc.THRUST_THRESHOLD, tc.MAX_FORCE)

        # direct mapping (doesn't apply linear forces)
        #self.throttle = (self.tau / tc.MAX_THRUST) * tc.MAX_THROTTLE

        # limit throttles as specified in the configuration (vehicle tuning)
        self.throttle = np.clip(self.throttle, -self.throttle_limit, self.throttle_limit).astype(int)

        # send throttle
        ttc = ThrusterCommand()
        ttc.header.stamp = rospy.Time.now()
        ttc.throttle = self.throttle.flatten().tolist()
        self.pub_thr.publish(ttc)

        # send force feedback
        pf = Vector6Stamped()
        pf.header.stamp = rospy.Time.now()
        pf.values = self.tau_total.tolist()
        self.pub_forces.publish(pf)


    def print_info(self, event=None):
        print(self)

        if self.ctrl_status == CTRL_ENABLED:
            print(self.controller)


    def run(self):
        # init pilot
        # ...

        # pilot loop
        while not rospy.is_shutdown():
            self.loop()

            try:
                self.pilot_loop.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo('%s shutdown requested ...', self.name)


        # graceful shutdown
        self.ctrl_status = CTRL_DISABLED

        # send zero throttle
        # ttc = ThrusterCommand()
        # ttc.header.stamp = rospy.Time.now()
        # ttc.throttle = np.zeros(6).tolist()
        # self.pub_thr.publish(ttc)


    def __str__(self):
        return """pilot:
          pos: %s
          vel: %s
          des_p: %s
          des_v: %s
          lim_vu: %s
          lim_vc: %s
          tau_c: %s
          tau_u: %s
          tau: %s
          for: %s
          for_s: %s
          thl: %s
          eff: %s
          dis: %s
        """ % (
            self.pos, self.vel, self.des_pos, self.des_vel,
            self.lim_vel_user, self.lim_vel_ctrl,
            self.tau_ctrl, self.tau_user, self.tau_total,
            self.forces, self.forces_sat, self.throttle,
            self.thruster_efficiency, self.disable_axis
        )



def main():
    rospy.init_node('vehicle_pilot')
    name = rospy.get_name()
    rospy.loginfo('%s initializing ...', name)

    # parse args
    #   ros_args[0] is always the program name
    ros_args = rospy.myargv()

    parser = argparse.ArgumentParser(
        description='Vehicle Pilot ROS Node. This node is the sending commands to the thrusters driver.',
        epilog='This is part of vehicle_pilot module.'
    )
    parser.add_argument('-v', '--verbose', action='store_true', help='Print detailed information.')
    args = parser.parse_args(ros_args[1:])

    if args.verbose:
        verbose = True
    else:
        verbose = False

    # load global parameters
    rate = int(rospy.get_param('~pilot_rate', DEFAULT_RATE))
    lim = int(rospy.get_param('thrusters/throttle_limit', tc.MAX_THROTTLE))
    topic_output = rospy.get_param('~topic_output', TOPIC_CMD)

    rate = int(np.clip(rate, 1, 100).astype(int))
    lim = int(np.clip(lim, 0, 100).astype(int))

    # show current settings
    rospy.loginfo('%s pilot rate: %s Hz', name, rate)
    rospy.loginfo('%s topic output: %s', name, topic_output)
    rospy.loginfo('%s throttle limit: %s%%', name, lim)

    # (reduce verbosity) show current config
    # rospy.loginfo('%s pitch enabled: %s', name, config['pitch_enable'])
    # rospy.loginfo('%s adaptive yaw enabled: %s', name, config.get('adaptive_yaw', False))
    # rospy.loginfo('%s fault control enabled: %s', name, config.get('fault_control', False))
    # rospy.loginfo('%s fault speeds enabled: %s', name, config.get('fault_speeds', False))


    # init the vehicle
    srv_thrusters = None

    try:
        rospy.wait_for_service(SRV_THRUSTERS, timeout=1)
        srv_thrusters = rospy.ServiceProxy(SRV_THRUSTERS, BooleanService)
        srv_thrusters.call(True)
    except rospy.ServiceException as se:
        rospy.logwarn('%s error contacting thrusters service', name)
    except rospy.ROSException as re:
        rospy.logwarn('%s could not contact thrusters service', name)

    # start vehicle control node
    pilot = VehiclePilot(name, rate, throttle_limit=lim, topic_output=topic_output, verbose=verbose)

    try:
        pilot.run()
    except Exception:
        tb = traceback.format_exc()
        rospy.logfatal('%s uncaught exception, dying!\n%s', name, tb)

    # disable thrusters using thruster service
    if srv_thrusters:
        srv_thrusters.call(False)


if __name__ == '__main__':
    main()
