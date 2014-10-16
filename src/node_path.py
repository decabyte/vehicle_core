#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This script picks up a list of waypoints from a service, processes the points
differently depending on the mode. Passes a pilot request to the controller.

Different modes that can be used:
- simple - list of points is passed straight to the low level controller
- line - list is converted so that the trajectory is composed of only
   straight lines and yaw adjustments

In the future?
- fast - optimises for time, smoothest path between the points
- hold orientation - for looking at a wall while moving along it, etc.
- avoid - use sonar data to generate path around an obstacle
"""

from __future__ import division

import traceback
import numpy as np
np.set_printoptions(precision=3, suppress=True)

from vehicle_core.path import path_strategy as ps

# imports
import rospy
import roslib
roslib.load_manifest('vehicle_core')

# msgs and services
from diagnostic_msgs.msg import KeyValue
from visualization_msgs.msg import Marker, MarkerArray

from auv_msgs.msg import NavSts
from vehicle_interface.msg import PathStatus, PilotRequest
from vehicle_interface.srv import PathService, PathServiceResponse, BooleanService


# constants
TOPIC_NAV = 'nav/nav_sts'
TOPIC_POS_REQ = 'pilot/position_req'
TOPIC_PILOT = 'pilot/status'
SRV_CONTROLLER = 'pilot/switch'

TOPIC_STATUS = 'path/status'
TOPIC_VIS = 'path/markers'

SRV_PATH_CONTROL = 'path/control'


# states
S_RESET = 0
S_HOVERING = 1
S_RUNNING = 2

# status publishing
EV_DEFAULT = 0
EV_COMPLETE = 1
EV_TIMEOUT = 2
EV_RESET = 3

# path modes
PATH_SIMPLE = 'simple'
PATH_FAST = 'fast'
PATH_LINES = 'lines'

# timing
DEFAULT_RATE = 5        # Hz
T_PILOT_STS = 2         # secs


# utils
def wrap_angle(angle):
    return (angle + np.pi) % (2*np.pi)-np.pi


class PathController(object):
    def __init__(self, name, visualization=False):
        self.name = name

        # state
        self.state = S_RESET
        self.t0 = rospy.Time.now().to_sec()

        # path data
        self.path_mode = None
        self.path_timeout = 0
        self.path_time_0 = 0
        self.path_id = -1

        self.position = np.zeros(6)          # north, east, depth, roll, pitch, yaw
        self.velocity = np.zeros(6)          # u, v, w, p, q, r
        self.des_pos = np.zeros(6)
        self.des_vel = np.zeros(6)
        self.err_pos = np.zeros(6)
        self.err_vel = np.zeros(6)

        # functions
        self.state_machine = {
            S_RESET:    self.idling,
            S_HOVERING: self.hovering,
            S_RUNNING:  self.running
        }

        # ros interface
        self.pub_path_status = rospy.Publisher(TOPIC_STATUS, PathStatus, queue_size=1)
        self.pub_pos_req = rospy.Publisher(TOPIC_POS_REQ, PilotRequest, queue_size=1)
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.update_pos, queue_size=1)
        # self.sub_pilot = rospy.Subscriber(TOPIC_PILOT, PilotStatus, self.update_error, queue_size=1)

        self.visualization = True
        self.pub_vis = rospy.Publisher(TOPIC_VIS, MarkerArray, queue_size=1, latch=True)

        self.srv_controller = rospy.ServiceProxy(SRV_CONTROLLER, BooleanService)
        self.srv_path = rospy.Service(SRV_PATH_CONTROL, PathService, self.handle_path_srv)

        # timers
        self.t_path_status = rospy.Timer(rospy.Duration(T_PILOT_STS), self.publish_path_status)


    def loop(self):
        try:
            self.state_machine[self.state]()
        except Exception:
            tb = traceback.format_exc()
            rospy.logfatal('%s error during navigation, resetting path nav:\n%s', self.name, tb)
            self.cmd_reset()


    def idling(self):
        pass

    def hovering(self):
        self.send_position_request()        # maybe send a stay request?

    def running(self):
        if self.path_mode is None or self.path_mode.path_completed is True:
            self.cmd_hover()
            return

        # update navigation time
        self.path_time_elapsed = rospy.Time().now().to_sec() - self.path_time_0

        if self.path_timeout > 0 and self.path_timeout < self.path_time_elapsed:
            rospy.loginfo('%s path timed out', self.name)
            self.publish_path_status(event=EV_COMPLETE)
            self.cmd_hover()
        else:
            self.path_mode.update(self.position, self.velocity)


            self.des_pos = self.path_mode.des_pos
            self.send_position_request()

            if self.path_mode.path_completed is True:
                self.publish_path_status(event=EV_COMPLETE)
                self.cmd_hover(self.des_pos)

            rospy.logdebug('%s position request: %s', self.name, self.des_pos)


    def handle_path_srv(self, request):
        packed_request = {
            'command':  request.command,
            'points':   request.points
        }

        for option in request.options:
            packed_request[option.key] = option.value

        act_on_command = {
            'start': self.cmd_start,
            'pause': self.cmd_hover,
            'reset': self.cmd_reset,
            'path': self.cmd_path
        }

        info = {}
        res = PathServiceResponse()
        res.result = True

        try:
            cmd = packed_request['command']
            response = act_on_command[cmd](**packed_request)
            info.update(response)
        except Exception:
            info['error'] = 'unspecified command'

            tb = traceback.format_exc()
            rospy.logerr('%s error in processing service request:\n%s', self.name, tb)

        if 'error' in info.keys():
            res.result = False

        res.info = [KeyValue(key, value) for key, value in info.items()]
        res.state = str(self.state)
        return res


    def cmd_start(self, **kwargs):
        try:
            response = self.srv_controller.call(True)
            rospy.loginfo('%s switching on controller: %s', self.name, response)
            self.state = S_RUNNING
        except rospy.ServiceException:
            tb = traceback.format_exc()
            rospy.logerr('%s: controller service error:\n%s', self.name, tb)
            return {'error': 'controller switch failed'}
        return {}


    def cmd_hover(self, point=None, **kwargs):
        # default hover at the last known pos
        self.des_pos = self.position

        # user gave us a specific point
        if point is not None:
            self.des_pos = point

        self.state = S_HOVERING
        return {}


    def cmd_reset(self, **kwargs):
        try:
            response = self.srv_controller.call(False)
            rospy.loginfo('%s switching off controller: %s', self.name, response)
            self.state = S_RESET
            self.path_mode = None
            self.input_points = None
            self.trajectory = None
            self.path_timeout = 0
            self.path_time_0 = 0
            self.publish_path_status(event=EV_RESET)
        except rospy.ServiceException:
            tb = traceback.format_exc()
            rospy.logwarn('%s: controller service error:\n%s', self.name, tb)
            return {'error': 'controller switch failed'}
        return {}


    def cmd_path(self, mode=None, points=None, **kwargs):
        if mode is None:
            rospy.logerr('%s mode not specified', self.name)
            return {'error': 'mode not specified'}

        if points is None:
            rospy.logerr('%s no points specified', self.name)
            return {'error': 'no points specified'}

        self.path_mode = mode
        points_array = np.zeros((len(points), 6))

        for i, point in enumerate(points):
            points_array[i] = np.array(point.values)

        # if missing set timeout to a negative number (infinite time)
        self.path_timeout = float(kwargs.get('timeout', -1))

        if self.path_timeout < 0:
            rospy.loginfo('%s path without a timeout', self.name)

        generate_path = {
            PATH_SIMPLE: ps.SimpleStrategy,
            PATH_LINES: ps.LineStrategy,
            PATH_FAST: ps.FastTimeStrategy
        }

        # constructor arguments
        path_options = {}
        path_options['position'] = self.position
        path_options['points'] = points_array
        path_options.update(kwargs)                 # pass any optional parameters from the request to the path mode

        try:
            self.path_mode = generate_path[self.path_mode](**path_options)
            self.path_id += 1
            self.path_time_0 = rospy.Time().now().to_sec()

            rospy.loginfo('%s new path accepted: %d', self.name, self.path_id)
        except KeyValue:
            rospy.logerr('%s unknown path mode: %s', self.name, self.path_mode)
            return {'error': 'unknown path mode'}

        return {'path_id': str(self.path_id)}


    def update_pos(self, data):
        self.position = np.array([
            data.position.north,
            data.position.east,
            data.position.depth,
            data.orientation.roll,
            data.orientation.pitch,
            data.orientation.yaw
        ])

        self.velocity = np.array([
            data.body_velocity.x,
            data.body_velocity.y,
            data.body_velocity.z,
            data.orientation_rate.roll,
            data.orientation_rate.pitch,
            data.orientation_rate.yaw
        ])


    def send_position_request(self):
        pr = PilotRequest()
        pr.position = self.des_pos
        #pr.velocity = self.des_vel
        self.pub_pos_req.publish(pr)


    def publish_path_status(self, event=None):
        ps = PathStatus()
        ps.header.stamp = rospy.Time().now()

        if self.state is S_RESET:
            ps.status = 'reset'
        elif self.state is S_HOVERING:
            ps.status = 'hovering'
        elif self.state is S_RUNNING:
            ps.status = 'running'
            ps.path_id = self.path_id
            ps.time_elapsed = self.path_time_elapsed
            # distance_travelled = self.trajectory_distances[self.trajectory_cnt-1]
            # average_speed = distance_travelled / ps.time_elapsed
            # ps.time_arrival = (self.trajectory_distances[-1] - distance_travelled) / average_speed
            # ps.path_completion = distance_travelled / self.trajectory_distances[-1]
        else:
            ps.status = 'wrong_state'

        # TODO: completion messages can be lost and clients stop advancing the mission
        #
        #   possible solutions:
        #       add extra status S_COMPLETED, S_TIMEOUT
        #       both mapping to S_HOVERING but broadcasting the completion information
        #       this can fit the existing logic and can suggest to add all the required fields to the status message
        #       and populate the fields only interested by the advertised status
        #
        #       create a new topic where completion messages are broadcasted
        #       this is suboptimal

        if event == EV_COMPLETE:
            ps.info.append(KeyValue('path_completed', str(self.path_id)))
            ps.info.append(KeyValue('time_started', str(self.path_time_0)))
            ps.info.append(KeyValue('time_completed', str(rospy.Time().now().to_sec())))

        elif event == EV_TIMEOUT:
            ps.info.append(KeyValue('path_timeout', str(self.path_id)))

        elif event == EV_RESET:
            ps.info.append(KeyValue('path_reset', str(self.path_id)))

        self.pub_path_status.publish(ps)

        if self.visualization:
            self.publish_markers()


    def publish_markers(self):
        if self.path_mode is None:
            return

        ma = MarkerArray()
        ma.markers = []

        for n, point in enumerate(self.path_mode.points):
            mm = Marker()
            mm.header.stamp = rospy.Time.now()
            mm.header.frame_id = 'odom'
            mm.ns = 'current_path'
            mm.id = n
            mm.type = Marker.SPHERE
            mm.action = Marker.ADD

            mm.pose.position.x = point[0]
            mm.pose.position.y = -point[1]
            mm.pose.position.z = -point[2]
            mm.pose.orientation.x = 0.0
            mm.pose.orientation.y = 0.0
            mm.pose.orientation.z = 0.0
            mm.pose.orientation.w = 1.0
            mm.scale.x = 0.2
            mm.scale.y = 0.2
            mm.scale.z = 0.2

            if n == self.path_mode.cnt:
                mm.color.r = 0.75
                mm.color.g = 0.75
                mm.color.b = 0.75
            elif n == self.path_mode.cnt + 1:
                mm.color.r = 0.75
                mm.color.g = 0.75
                mm.color.b = 0.0
            elif n > self.path_mode.cnt:
                mm.color.r = 1.0
                mm.color.g = 0.0
                mm.color.b = 0.0
            else:
                mm.color.r = 0.0
                mm.color.g = 1.0
                mm.color.b = 0.0

            mm.color.a = 1.0
            ma.markers.append(mm)

        self.pub_vis.publish(ma)


# MAIN
if __name__ == '__main__':
    rospy.init_node('path_navigator')
    name = rospy.get_name()

    # init
    path = PathController(name)
    r_loop = rospy.Rate(DEFAULT_RATE)

    rospy.loginfo('%s: initializing ...', name)
    rospy.loginfo('%s: waiting for nav ...', name)

    rospy.wait_for_message(TOPIC_NAV, NavSts)
    rospy.loginfo('%s: nav received ...', name)

    rospy.wait_for_service(SRV_CONTROLLER, timeout=5)
    rospy.loginfo('%s: pilot detected ...', name)

    # start from a safe setting (clean state and pilot disabled)
    path.cmd_reset()

    while not rospy.is_shutdown():
        path.loop()

        try:
            r_loop.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr('%s: unclean shutdown ...', name)

    # shutdown procedure: stop navigation and disable the pilot (using cmd_reset)
    path.cmd_reset()
    rospy.loginfo('%s: graceful shutdown ...', name)
