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
from vehicle_interface.msg import PilotRequest, PathRequest, PathStatus
from vehicle_interface.srv import PathService, PathServiceResponse, BooleanService


# constants
TOPIC_NAV = 'nav/nav_sts'
TOPIC_PILOT = 'pilot/status'
TOPIC_POS_REQ = 'pilot/position_req'
TOPIC_VEL_REQ = 'pilot/velocity_req'

TOPIC_REQ = 'path/request'
TOPIC_STATUS = 'path/status'
TOPIC_VIS = 'path/markers'

SRV_CONTROLLER = 'pilot/switch'
SRV_PATH_CONTROL = 'path/control'


# node states
S_ERROR = -1
S_RESET = 0
S_HOVERING = 1
S_RUNNING = 2

# path status
P_IDLE = 0
P_RUNNING = 1
P_COMPLETED = 2
P_TIMEOUT = -1
P_ABORT = -2

# mapping status
STATUS_PATH = {
    P_IDLE: PathStatus.PATH_IDLE,
    P_RUNNING: PathStatus.PATH_RUNNING,
    P_COMPLETED: PathStatus.PATH_COMPLETED,
    P_TIMEOUT: PathStatus.PATH_TIMEOUT,
    P_ABORT: PathStatus.PATH_ABORT
}

STATUS_NAV = {
    S_RESET: PathStatus.NAV_IDLE,
    S_HOVERING: PathStatus.NAV_HOVERING,
    S_RUNNING: PathStatus.NAV_RUNNING,
    S_ERROR: PathStatus.NAV_ERROR
}

# path modes
PATH_SIMPLE = 'simple'
PATH_FAST = 'fast'
PATH_LINES = 'lines'

# timing
RATE_NODE = 5           # Hz
RATE_STATUS = 0.5       # Hz
HOVER_TIMEOUT = 60      # sec


class PathController(object):

    def __init__(self, name, **kwargs):
        self.name = name

        # state
        self.rate_node = 1.0 / RATE_NODE
        self.rate_status = 1.0 / RATE_STATUS

        self.state = S_RESET
        self.t0 = rospy.Time.now().to_sec()

        # path data
        self.path_id = -1
        self.path_status = P_IDLE
        self.path_mode = PATH_SIMPLE
        self.path_obj = None

        self.path_timeout = 0.0
        self.path_time_start = -1.0
        self.path_time_end = -1.0
        self.path_time_elapsed = 0.001

        self.hover_timeout = HOVER_TIMEOUT

        self.position = np.zeros(6)          # north, east, depth, roll, pitch, yaw
        self.velocity = np.zeros(6)          # u, v, w, p, q, r
        self.des_pos = np.zeros(6)
        self.des_vel = np.zeros(6)
        self.dis_axis = np.zeros(6)

        self.err_pos = np.zeros(6)
        self.err_vel = np.zeros(6)

        # functions
        self.state_machine = {
            S_RESET:    self.idling,
            S_HOVERING: self.hovering,
            S_RUNNING:  self.running
        }

        self.act_on_command = {
            'start': self.cmd_start,
            'pause': self.cmd_hover,
            'reset': self.cmd_reset,
            'path': self.cmd_path
        }

        # ros interface
        self.sub_req = rospy.Subscriber(TOPIC_REQ, PathRequest, self.handle_request, queue_size=1)
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.update_nav, queue_size=1)
        self.pub_path_status = rospy.Publisher(TOPIC_STATUS, PathStatus, queue_size=1)
        self.pub_pos_req = rospy.Publisher(TOPIC_POS_REQ, PilotRequest, queue_size=1)

        # status related
        # self.sub_pilot = rospy.Subscriber(TOPIC_PILOT, PilotStatus, self.update_error, queue_size=1)

        # user related
        self.visualization = True
        self.pub_vis = rospy.Publisher(TOPIC_VIS, MarkerArray, queue_size=1, latch=True)

        # services
        self.srv_controller = rospy.ServiceProxy(SRV_CONTROLLER, BooleanService)
        self.srv_path = rospy.Service(SRV_PATH_CONTROL, PathService, self.handle_path_srv)

        # timers
        self.t_path_status = rospy.Timer(rospy.Duration(self.rate_status), self.publish_path_status)


    def loop(self):
        try:
            self.state_machine[self.state]()
        except Exception:
            tb = traceback.format_exc()
            rospy.logfatal('%s: error during navigation, resetting path nav:\n%s', self.name, tb)
            self.cmd_reset()


    def idling(self):
        pass

    def hovering(self):
        if self.hover_timeout is None:
            self.send_position_request()        # maybe send a stay request?
        else:
            if self.hover_timeout > 0:
                self.hover_timeout -= self.rate_node
                self.send_position_request()    # maybe send a stay request?
            else:
                rospy.logwarn('%s: hover timeout expired, resetting pilot ...', self.name)
                self.cmd_reset()

    def running(self):
        if self.path_obj is None or self.path_obj.path_completed is True:
            self.cmd_hover()
            return

        # update navigation time
        self.path_time_elapsed = rospy.Time().now().to_sec() - self.path_time_start

        if self.path_timeout > 0 and self.path_timeout < self.path_time_elapsed:
            # path timeout expiration
            rospy.loginfo('%s: path timed out', self.name)
            self.path_status = P_TIMEOUT
            self.publish_path_status()

            # switch to hover mode
            self.cmd_hover(timeout=HOVER_TIMEOUT)
        else:
            # default case navigation is allowed and running
            self.path_obj.update(self.position, self.velocity)

            # pass the set values to the pilot
            self.des_pos = self.path_obj.des_pos
            self.dis_axis = self.path_obj.dis_axis

            # send commands to pilot
            self.send_position_request()

            # check for completion
            if self.path_obj.path_completed is True:
                self.path_status = P_COMPLETED
                self.path_time_end = rospy.Time().now().to_sec()
                self.publish_path_status()

                # calculate the remaining path time
                hover_timeout = self.path_timeout - self.path_time_elapsed
                hover_timeout = max(HOVER_TIMEOUT, hover_timeout)

                # switch to hover mode
                self.cmd_hover(self.des_pos, timeout=hover_timeout)

            #rospy.logdebug('%s: position request: %s', self.name, self.des_pos)


    def handle_request(self, request):
        packed_request = {
            'command':  request.command,
            'points':   request.points
        }

        for option in request.options:
            packed_request[option.key] = option.value

        try:
            cmd = packed_request['command']

            if cmd == '' or cmd == None:
                rospy.logwarn('%s: path request without command, assuming new path request', self.name)
                cmd = 'path'

            res = self.act_on_command[cmd](**packed_request)

            if res.get('path_id', None) != None:
                rospy.loginfo('%s: starting path from request with id: %s', self.name, res['path_id'])

                # autostart the new path
                self.cmd_start()

        except Exception:
            tb = traceback.format_exc()
            rospy.logerr('%s: error in processing topic request:\n%s', self.name, tb)


    def handle_path_srv(self, request):
        packed_request = {
            'command':  request.command,
            'points':   request.points
        }

        for option in request.options:
            packed_request[option.key] = option.value

        info = {}
        res = PathServiceResponse()
        res.result = True

        try:
            cmd = packed_request['command']
            response = self.act_on_command[cmd](**packed_request)
            info.update(response)
        except Exception:
            info['error'] = 'unspecified command'

            tb = traceback.format_exc()
            rospy.logerr('%s: error in processing service request:\n%s', self.name, tb)

        if 'error' in info.keys():
            res.result = False

        res.info = [KeyValue(key, value) for key, value in info.items()]
        res.state = STATUS_PATH[self.state]

        return res


    def cmd_start(self, **kwargs):
        try:
            response = self.srv_controller.call(True)
            rospy.loginfo('%s: switching on controller: %s', self.name, response)

            self.state = S_RUNNING
            self.path_status = P_RUNNING
            self.path_time_end = -1
        except rospy.ServiceException:
            tb = traceback.format_exc()
            rospy.logerr('%s: controller service error:\n%s', self.name, tb)
            return {'error': 'controller switch failed'}

        return {}


    def cmd_hover(self, point=None, timeout=None, **kwargs):
        # default hover at the last known pos
        self.des_pos = self.position

        # user gave us a specific point
        if point is not None:
            self.des_pos = point

        # user gave us a timeout for hovering
        self.hover_timeout = timeout

        # set internal state
        self.state = S_HOVERING

        # update path status
        if self.path_status not in (P_COMPLETED, P_TIMEOUT):
            self.path_status = P_IDLE

        return {}


    def cmd_reset(self, reason='abort', **kwargs):
        try:
            self.state = S_RESET

            self.path_obj = None
            self.input_points = None
            self.trajectory = None

            self.path_timeout = 0
            self.path_time_end = -1

            # TODO: change this by parsing the "reason" parameter (abort --> P_ABORT, anything else --> P_IDLE)
            if self.path_status != P_IDLE:
                self.path_status = P_ABORT

            self.publish_path_status()

            response = self.srv_controller.call(False)
            rospy.loginfo('%s: switching off controller: %s', self.name, response)
        except rospy.ServiceException:
            tb = traceback.format_exc()
            rospy.logwarn('%s: controller service error:\n%s', self.name, tb)
            return {'error': 'controller switch failed'}
        return {}


    def cmd_path(self, mode=None, points=None, **kwargs):
        if mode is None:
            mode = PATH_LINES
            rospy.logwarn('%s: mode not specified, defaulting to %s', self.name, mode)
            #return {'error': 'mode not specified'}

        if points is None:
            rospy.logerr('%s: no points specified', self.name)
            return {'error': 'no points specified'}

        self.path_mode = mode
        points_array = np.zeros((len(points), 6))

        for i, point in enumerate(points):
            points_array[i] = np.array(point.values)

        # if missing set timeout to a negative number (infinite time)
        self.path_timeout = float(kwargs.get('timeout', -1))

        if self.path_timeout < 0:
            rospy.logwarn('%s: path requested without timeout', self.name)

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
            #self.path_mode = generate_path[self.path_mode](**path_options)
            self.path_obj = generate_path[self.path_mode](**path_options)
            self.path_id += 1
            self.path_time_start = rospy.Time().now().to_sec()

            rospy.loginfo('%s: new path accepted [%d] with timeout %s', self.name, self.path_id, self.path_timeout)
            #rospy.loginfo('%s: path to follow:\n%s', self.name, self.path_obj.points)
        except KeyValue:
            rospy.logerr('%s: unknown path mode: %s', self.name, self.path_mode)
            return {'error': 'unknown path mode'}

        return {'path_id': str(self.path_id)}


    def update_nav(self, data):
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
        pr.header.stamp = rospy.Time.now()
        pr.position = self.des_pos
        #pr.velocity = self.des_vel
        pr.disable_axis = self.dis_axis

        self.pub_pos_req.publish(pr)


    def publish_path_status(self, event=None):
        ps = PathStatus()
        ps.header.stamp = rospy.Time().now()
        ps.navigation_status = STATUS_NAV[self.state]
        ps.path_status = STATUS_PATH[self.path_status]

        if self.path_obj is not None:
            ps.path_id = self.path_id
            pos = None

            # this prevents the counters to overflow when the vehicle terminates the path navigation
            # successfully and the user decides to use a different type of control (joystick, pilot requests, etc.)
            # if self.path_status == P_RUNNING:
            #     pos = self.position

            ps.distance_completed = self.path_obj.distance_completed(pos)
            ps.distance_left = self.path_obj.distance_left(pos)

            ps.time_elapsed = self.path_time_elapsed
            ps.speed_average = ps.distance_completed / ps.time_elapsed

            if ps.speed_average != 0:
                ps.time_arrival = ps.distance_left / ps.speed_average
            else:
                ps.time_arrival = np.Inf

            ps.time_start = self.path_time_start
            ps.time_end = self.path_time_end

            ps.path_completion = (ps.distance_completed / self.path_obj.total_distance) * 100.0

        self.pub_path_status.publish(ps)

        if self.visualization:
            self.publish_markers()


    def publish_markers(self):
        if self.path_obj is None:
            return

        ma = MarkerArray()
        ma.markers = []

        for n, point in enumerate(self.path_obj.points):
            mm = Marker()
            mm.header.stamp = rospy.Time.now()
            mm.header.frame_id = 'map'
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

            if n == self.path_obj.cnt:
                mm.color.r = 0.75
                mm.color.g = 0.75
                mm.color.b = 0.75
            elif n == self.path_obj.cnt + 1:
                mm.color.r = 0.75
                mm.color.g = 0.75
                mm.color.b = 0.0
            elif n > self.path_obj.cnt:
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
    r_loop = rospy.Rate(RATE_NODE)

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
