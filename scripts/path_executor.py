#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Assumes vehicle is to start from the left bottom corner of the tank and
# move 15 meters to the left and 10 meters away.

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
from vehicle_interface.msg import PathStatus, Vector6, PilotRequest
from vehicle_interface.srv import PathService


# enable energy measurements if the energy subsystem is available
USE_ENERGY = False

try:
    from saetta_energy.msg import EnergyReport
    USE_ENERGY = True
except ImportError:
    print('No energy framework detected, skipping energy measurements ...')


# topics
TOPIC_STATUS = 'path/status'
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
        self.wait = True
        self.experiment_running = False

        # export data
        self.csv_file = '{0}_{1}.csv'.format(CSV_PREFIX, suffix)
        self.csv_file = os.path.join(os.getcwd(), self.csv_file)

        self.header = [
            'time', 'theta', 'path', 'mode', 'duration'
        ]

        # change sign to switch from marine to maths angle direction convention
        # angle from vehicle's north to wavetank's north
        self.theta = -offset_yaw

        rospy.loginfo('%s: theta: %s', self.name, self.theta)
        rospy.loginfo('%s: x offset: %s', self.name, X_OFFSET)
        rospy.loginfo('%s: y offset: %s', self.name, Y_OFFSET)

        # ros interface
        self.srv_path = rospy.ServiceProxy(SRV_PATH, PathService)
        self.sub_pil = rospy.Subscriber(TOPIC_STATUS, PathStatus, self.handle_status)

        if USE_ENERGY:
            self.sub_energy = rospy.Subscriber(TOPIC_ENERGY, EnergyReport, self.handle_energy)
            self.header.append('energy_used')



    def handle_energy(self, msg):
        # parse energy data
        self.energy_last = msg.energy_used


    def handle_status(self, msg):
        # values
        self.time_started = msg.time_start

        if msg.path_status == PathStatus.PATH_COMPLETED:
            self.wait = False

            if self.experiment_running:
                self.experiment_completed = True
                self.experiment_running = False

                # update counters
                self.duration = msg.time_elapsed
                self.energy_used = self.energy_last - self.energy_initial

                # dump experiment data
                self.append_data()


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

            row = list()
            row.append(self.time_started)
            row.append(self.theta)
            row.append(self.path_file)
            row.append(self.path_mode)
            row.append(self.duration)

            if USE_ENERGY:
                row.append(self.energy_used)

            rospy.loginfo('%s: saving data to %s', self.name, self.csv_file)
            writer.writerow(row)



    def run(self):
        # check data file
        self.check_export()

        # calculate offsets and points
        rot_matrix = np.eye(6)
        rot_matrix[0,0] = np.cos(self.theta)
        rot_matrix[0,1] = -np.sin(self.theta)
        rot_matrix[1,0] = np.sin(self.theta)
        rot_matrix[1,1] = np.cos(self.theta)

        # load path
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

        # reset the path controller
        try:
            self.srv_path.call(command='reset')
        except Exception:
            rospy.logerr('%s unable to communicate with path service ...', self.name)

        # reach initial point
        rospy.loginfo('%s: reaching initial point ...', self.name)

        path_initial = [
            self.path_points[0]
        ]
        options = [
            KeyValue('mode', 'simple'),
            KeyValue('timeout', '1000'),
            KeyValue('target_speed', '0.4')
        ]

        try:
            self.wait = True
            self.srv_path.call(command='path', points=path_initial, options=options)
            self.srv_path.call(command='start')
        except Exception:
            self.wait = False
            rospy.logerr('%s unable to communicate with path service ...', self.name)
            sys.exit(-1)


        while self.wait:
            if rospy.is_shutdown():
                sys.exit(-1)
            else:
                rospy.sleep(0.5)

        # wait a little bit
        rospy.sleep(5)

        # experiment
        rospy.loginfo('%s: starting experiment ...', self.name)

        self.options = [
            KeyValue('mode', self.path_mode),
            KeyValue('timeout', '1000'),
            KeyValue('target_speed', '0.4')
        ]

        try:
            self.experiment_running = True
            self.wait = True
            self.energy_initial = self.energy_last
            self.t_expire = time.time() + 1000
            self.t_last = time.time()

            rospy.loginfo('%s: requesting path (mode: %s) ...', self.name, self.path_mode)
            self.srv_path.call(command='path', points=self.path_points, options=self.options)
            self.srv_path.call(command='start')

        except Exception:
            self.wait = False
            rospy.logerr('%s unable to communicate with path service ...', self.name)


        while self.wait:
            if rospy.is_shutdown():
                rospy.logerr('%s: experiment timeout ...', self.name)
                break
            elif self.t_last >= self.t_expire:
                rospy.logerr('%s: ros is shutdown ...', self.name)
                break
            else:
                rospy.sleep(2.0)

        try:
            self.srv_path.call(command='reset')
        except Exception:
            rospy.logerr('%s unable to communicate with path service ...', self.name)

        # congratulations!
        rospy.loginfo('%s: experiment completed ...', self.name)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Utility for running experiments using the path controller.',
        epilog='This is part of vehicle_core module.'
    )

    # navigation group
    # parser.add_argument('n_offset', type=float, help='North offset of initial point in wavetank coordinates.')
    # parser.add_argument('e_offset', type=float, help='East offset of initial point in wavetank coordinates.')
    parser.add_argument('yaw_offset', type=float, help='Yaw offset between magnetic north and wavetank coordinates.')
    parser.add_argument('--mode', default='lines', help='Select the navigation mode.')
    parser.add_argument('--path', help='Path file to execute (es. simple.json).')

    # output group
    parser.add_argument('--output', default='last', help='Output file to save during the experiments.')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print detailed information.')

    args = parser.parse_args()

    # init the ROS node
    rospy.init_node('path_executor')
    name = rospy.get_name()

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
    we = PathExecutor(name, offset_yaw=off_yaw, suffix=suffix, mode=args.mode, path=args.path)

    try:
        we.run()
    except Exception:
        tb = traceback.format_exc()
        rospy.logfatal('%s: uncaught exception:\n%s', name, tb)
        sys.exit(-1)
