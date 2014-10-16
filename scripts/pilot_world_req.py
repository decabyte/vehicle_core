#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
import signal
import argparse
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('vehicle_core')

from vehicle_interface.msg import PilotRequest


# config
TOPIC_PUB = '/pilot/position_req'

TYPES = (
    'depth',
    'altitude'
)

# info
DESC='''
This utility sends position requests to the pilot module. The type parameter (depth or altitude) requests either depth
or altitude control on the z-axis. If 'n' is given for an axis value the control will be disabled for that axis.
'''

EPI='''
Please refer to the package's documentation for further info.
'''


def main():
    parser = argparse.ArgumentParser(description=DESC, epilog=EPI)

    parser.add_argument('type', choices=TYPES, metavar='type', help='select z-axis mode', default='depth')
    parser.add_argument('x', help='north coordinate', default='n', nargs='?')
    parser.add_argument('y', help='east coordinate', default='n', nargs='?')
    parser.add_argument('z', help='depth or altitude', default='n', nargs='?')
    parser.add_argument('m', help='pitch attitude', default='n', nargs='?')
    parser.add_argument('n', help='yaw attitude', default='n', nargs='?')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print detailed information.')

    if len(sys.argv) <= 1:
        parser.print_help()
        sys.exit(1)

    # parse input
    args = parser.parse_args()

    # default request
    des_pos = np.zeros(6)
    des_vel = np.zeros(6)
    dis_axi = np.zeros(6)
    lim_vel = np.zeros(6)
    mode = args.type

    # parse degrees of freedom
    for i, d in enumerate((args.x, args.y, args.z, 0, args.m, args.n)):
        if d == 'n':
            dis_axi[i] = 1
        else:
            des_pos[i] = float(d)

    # log info
    print('Sending [%s] request at 1 Hz:' % TOPIC_PUB)
    print('mode: %s' % mode)
    print('position: %s' % des_pos)
    print('disable: %s\n' % dis_axi)

    print('Press Ctrl+C to interrupt ...')

    # ros code
    rospy.init_node('pilot_cli', anonymous=True)
    pub = rospy.Publisher(TOPIC_PUB, PilotRequest, tcp_nodelay=True, queue_size=1)
    pr = PilotRequest()

    while not rospy.is_shutdown():
        pr.header.stamp = rospy.Time.now()
        pr.position = des_pos.tolist()
        pr.velocity = des_vel.tolist()
        pr.disable_axis = dis_axi.tolist()
        pr.limit_velocity = lim_vel.tolist()

        pub.publish(pr)
        rospy.sleep(1)

if __name__ == '__main__':
    main()
