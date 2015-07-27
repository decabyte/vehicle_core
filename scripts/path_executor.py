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

import sys
import os
import traceback
import time
import json
import csv
import argparse

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('vehicle_core')

from diagnostic_msgs.msg import KeyValue
from vehicle_interface.msg import PathRequest, PathStatus, Vector6, PilotRequest
from vehicle_interface.srv import PathService

# local config
USE_ENERGY = True

try:
    roslib.load_manifest('saetta_energy')
    from saetta_energy.msg import EnergyReport
except ImportError:
    # disable energy measurements if the energy subsystem is not available
    USE_ENERGY = False

# topics
TOPIC_STATUS = 'path/status'
TOPIC_PATH = 'path/request'
TOPIC_POS = 'pilot/position_req'
SRV_PATH = 'path/control'
SRV_SWITCH = 'pilot/switch'
TOPIC_ENERGY = 'saetta/report'

# default conf
CSV_PREFIX = 'results'
DEPTH = 1.0
YAW_OFFSET = 0
X_OFFSET = 0
Y_OFFSET = 0


class PathExecutor(object):

    def __init__(self, name, offset_yaw, suffix='last', **kwargs):
        self.name = name

        # path
        self.path_file = kwargs['path']
        self.path_mode = kwargs.get('mode', 'lines')
        self.path_points = list()

        # status
        self.cnt = 0
        self.duration = 0
        self.energy_used = 0
        self.energy_initial = 0
        self.energy_last = 0
        self.time_started = 0

        # flags
        self.initial_point = False
        self.experiment_running = False
        self.experiment_completed = False

        # export data
        self.label = kwargs.get('label', 'current')
        self.csv_file = '{0}_{1}.csv'.format(CSV_PREFIX, suffix)
        self.csv_file = os.path.join(os.getcwd(), self.csv_file)

        self.header = [
            'time', 'theta', 'path', 'mode', 'label', 'duration'
        ]

        # change sign to switch from marine to maths angle direction convention
        # angle from vehicle's north to wavetank's north
        self.theta = -offset_yaw

        rospy.loginfo('%s: theta: %s', self.name, self.theta)
        rospy.loginfo('%s: x offset: %s', self.name, X_OFFSET)
        rospy.loginfo('%s: y offset: %s', self.name, Y_OFFSET)

        # ros interface
        self.srv_path = rospy.ServiceProxy(SRV_PATH, PathService)
        self.pub_path = rospy.Publisher(TOPIC_PATH, PathRequest, queue_size=1, tcp_nodelay=True)
        self.sub_pil = rospy.Subscriber(TOPIC_STATUS, PathStatus, self.handle_status, queue_size=10, tcp_nodelay=True)

        if USE_ENERGY:
            self.sub_energy = rospy.Subscriber(TOPIC_ENERGY, EnergyReport, self.handle_energy, queue_size=10, tcp_nodelay=True)
            self.header.append('energy_used')
            rospy.loginfo('%s: energy framework found, logging energy ...', self.name)
        else:
            rospy.loginfo('%s: energy framework not found, skipping energy ...', self.name)


    def check_export(self):
        if os.path.exists(self.csv_file):
            with open(self.csv_file, 'r') as exp_file:
                reader = csv.reader(exp_file, delimiter=',')

                # check if an header is already there
                line = reader.next()

                if not line[0] == self.header[0]:
                    add_header = True
                else:
                    add_header = False
        else:
            add_header = True

        if add_header:
            with open(self.csv_file, 'a') as exp_file:
                writer = csv.writer(exp_file, delimiter=',')
                writer.writerow(self.header)

    def append_data(self):
        if self.energy_initial == 0:
            rospy.logwarn('%s: not logging energy ...', self.name)

        with open(self.csv_file, 'a') as exp_file:
            writer = csv.writer(exp_file, delimiter=',')
            filename = self.path_file.split('/')[-1]

            row = list()
            row.append(self.time_started)
            row.append(self.theta)
            row.append(filename)
            row.append(self.path_mode)
            row.append(self.label)
            row.append(self.duration)

            if USE_ENERGY:
                row.append(self.energy_used)

            rospy.loginfo('%s: saving data to %s', self.name, self.csv_file)
            writer.writerow(row)

    def load_path(self):
        """This loads the path points from the file provided by the user"""
        # calculate offsets and points
        rot_matrix = np.eye(6)
        rot_matrix[0,0] = np.cos(self.theta)
        rot_matrix[0,1] = -np.sin(self.theta)
        rot_matrix[1,0] = np.sin(self.theta)
        rot_matrix[1,1] = np.cos(self.theta)

        try:
            with open(self.path_file, 'rt') as f:
                path_dict = json.loads(f.read())

                for point in path_dict['points']:
                    # apply rotation
                    rot_point = np.dot(rot_matrix, point)

                    # add point to the list
                    self.path_points.append(
                        Vector6(rot_point)
                    )

            rospy.loginfo('%s: loaded %s points ...', self.name, len(self.path_points))

        except Exception:
            tb = traceback.format_exc()
            rospy.logfatal('%s: caught exception:\n%s', self.name, tb)
            sys.exit(-1)    

    def path_reset(self):
        """This functions uses a service request to reset the state of the path controller"""
        try:
            self.srv_path.call(command='reset')
        except Exception as e:
            rospy.logerr('%s: unable to communicate with path service ...', self.name)

    def send_path(self, points, mode='simple', timeout=1000, target_speed=1.0, look_ahead=5.0, **kwargs):
        msg = PathRequest()
        msg.header.stamp = rospy.Time.now()
        msg.command = 'path'
        msg.points = points
        msg.options = [
            KeyValue('mode', mode),
            KeyValue('timeout', str(timeout)),
            KeyValue('target_speed', str(target_speed)),
            KeyValue('look_ahead', str(look_ahead)),
        ]

        self.pub_path.publish(msg)

    def handle_energy(self, msg):
        # parse energy data
        self.energy_last = msg.energy_used

    def handle_status(self, msg):
        self.time_started = msg.time_start

        if msg.path_status == PathStatus.PATH_COMPLETED and msg.navigation_status == PathStatus.NAV_HOVERING:
            if not self.experiment_running and not self.initial_point:
                self.initial_point = True

            if self.experiment_running:
                self.experiment_completed = True
                self.experiment_running = False

                # update counters
                self.duration = msg.time_elapsed
                self.energy_used = self.energy_last - self.energy_initial

                # dump experiment data
                self.append_data()


    def run(self):
        # check data file
        self.check_export()

        # load path
        self.load_path()

        # reach initial point
        points = list()
        points.append(self.path_points[0])
        points.append(self.path_points[0])
        self.send_path(points)

        # initial reset (workaround)
        rospy.sleep(1.0)
        self.path_reset()
        rospy.sleep(1.0)

        rospy.loginfo('%s: requesting initial point ...', self.name)
        self.send_path(points)

        while not rospy.is_shutdown():
            if self.experiment_completed:
                break

            if self.initial_point and not self.experiment_running:
                # start the experiment
                rospy.loginfo('%s: starting experiment ...', self.name)
                self.energy_initial = self.energy_last
                self.t_expire = time.time() + 1000
                self.t_last = time.time()

                rospy.loginfo('%s: requesting path (mode: %s) ...', self.name, self.path_mode)
                self.send_path(self.path_points, mode=self.path_mode, target_speed=0.5)

                self.experiment_running = True

            if self.experiment_running and self.t_last >= self.t_expire:
                rospy.logerr('%s: experiment timeout ...', self.name)
                break

            rospy.sleep(0.5)

        # reset the path controller
        #self.path_reset()

        # congratulations!
        rospy.loginfo('%s: experiment completed ...', self.name)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Utility for running experiments using the path controller.',
        epilog='This is part of vehicle_core module.'
    )

    # navigation group
    # parser.add_argument('n_offset', type=float, help='North offset of initial point in wavetank coordinates.')
    # parser.add_argument('e_offset', type=float, help='East offset of initial point in wavetank coordinates.')
    parser.add_argument('path', help='Path file to execute (es. simple.json).')
    parser.add_argument('yaw_offset', type=float, nargs='?', default=0.0, help='Yaw offset between magnetic north and wavetank coordinates.')
    parser.add_argument('--mode', default='lines', help='Select the navigation mode.')


    # output group
    parser.add_argument('--output', default='last', help='Output file to save during the experiments.')
    parser.add_argument('--label', default='current', help='Optional comment to add to the result file.')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print detailed information.')

    return parser.parse_args()

def main():
    # init the ROS node
    rospy.init_node('path_executor')
    name = rospy.get_name()

    # parse args
    args = parse_arguments()

    # config
    # off_n = args.n_offset
    # off_e = args.e_offset
    off_yaw = np.deg2rad(args.yaw_offset)
    suffix = args.output

    # if off_n > 10 or off_n < -10:
    #     rospy.logfatal('%s: wrong input commands', name)
    #     sys.exit(-1)
    #
    # if off_e > 10 or off_e < -10:
    #     rospy.logfatal('%s: wrong input commands', name)
    #     sys.exit(-1)

    if off_yaw > np.pi or off_yaw < -np.pi:
        rospy.logfatal('%s: wrong input commands', name)
        sys.exit(-1)

    if not os.path.exists(args.path):
        rospy.logfatal('%s: file %s does not exists!', name, args.path)
        sys.exit(-1) 

    # run the experiment
    we = PathExecutor(name, offset_yaw=off_yaw, suffix=suffix, mode=args.mode, path=args.path, label=args.label)

    try:
        we.run()
    except Exception:
        tb = traceback.format_exc()
        rospy.logfatal('%s: uncaught exception:\n%s', name, tb)
        sys.exit(-1)

if __name__ == '__main__':
    main()
