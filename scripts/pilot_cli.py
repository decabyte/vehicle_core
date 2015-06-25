#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
import signal
import argparse

import numpy as np
np.set_printoptions(precision=3, suppress=True)
np.set_printoptions(formatter={'float': '{: 0.3f}'.format})
# see: http://stackoverflow.com/questions/2891790/pretty-printing-of-numpy-array

import rospy
import roslib
roslib.load_manifest('vehicle_core')

from vehicle_interface.msg import PilotRequest, PilotStatus
from vehicle_interface.srv import BooleanService

# config
TOPIC_STATUS = 'pilot/status'
TOPIC_POS = 'pilot/position_req'
TOPIC_VEL = 'pilot/velocity_req'
TOPIC_STY = 'pilot/stay_req'
TOPIC_BOD = 'pilot/body_req'
SRV_SWITCH = 'pilot/switch'

DEFAULT_SLEEP = 1.0

TYPE_POS = 'position'
TYPE_VEL = 'velocity'
TYPE_STY = 'stay'
TYPE_BOD = 'body'

TYPE_TOPIC = {
    TYPE_POS: TOPIC_POS,
    TYPE_VEL: TOPIC_VEL,
    TYPE_STY: TOPIC_STY,
    TYPE_BOD: TOPIC_BOD
}

MODES = (
    'depth',
    'altitude',
)

STATUS_FIELDS = ('status', 'mode', 'des_pos', 'des_vel', 'err_pos', 'err_vel','lim_vel_user', 'lim_vel_ctrl')

# info
DESC='''
This utility sends position, velocity, body or stay requests to the pilot module.
The mode parameter (depth or altitude) requests either depth or altitude control on the z-axis.
If the axis value is missing or equals to 'n' the control will be disabled for that axis.
'''

EPI='''
Stay requests, require all arguments to be provided by the user (e.g. all zeros).
Please refer to the package's documentation for further info.
'''


def main():
    parser = argparse.ArgumentParser(description=DESC, epilog=EPI)

    parser.add_argument('type', choices=TYPE_TOPIC.keys(), metavar='type', help='select request to send', default='position')

    parser.add_argument('x', help='north coordinate', default='n', nargs='?')
    parser.add_argument('y', help='east coordinate', default='n', nargs='?')
    parser.add_argument('z', help='depth or altitude', default='n', nargs='?')
    parser.add_argument('m', help='pitch attitude', default='n', nargs='?')
    parser.add_argument('n', help='yaw attitude', default='n', nargs='?')

    parser.add_argument('mode', choices=MODES, metavar='mode', help='select z-axis mode', default='depth', nargs='?')
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
    mode = args.mode
    type = args.type
    topic = TYPE_TOPIC[type]

    # parse degrees of freedom
    for i, d in enumerate((args.x, args.y, args.z, 0, args.m, args.n)):
        if d == 'n':
            dis_axi[i] = 1
        else:
            if type == TYPE_VEL:
                des_vel[i] = float(d)
            else:
                des_pos[i] = float(d)

    # log info
    print('Sending %s request at 1 Hz:' % type)
    print('-----------------------------------------------------------------------------------')
    print('mode: %s' % mode)
    print('position: %s' % des_pos)
    print('velocity: %s' % des_vel)
    print('disable: %s\n' % dis_axi)

    # ros code
    rospy.init_node('pilot_cli', anonymous=True)

    def send_request():
        pr = PilotRequest()
        pr.header.stamp = rospy.Time.now()
        pr.position = des_pos.tolist()
        pr.velocity = des_vel.tolist()
        pr.disable_axis = dis_axi.tolist()
        pr.limit_velocity = lim_vel.tolist()

        pub.publish(pr)

    def handle_status(data):
        print('Pilot Status [%s]' % rospy.Time.now().to_sec())
        print('-----------------------------------------------------------------------------------')

        for field in STATUS_FIELDS:
            value = getattr(data, field)

            if field is not ('status', 'mode'):
                value = np.array(value)

            print('%s:\t%s' % (field, value))

        print('dis_axi:\t%s' % dis_axi)

        print('-----------------------------------------------------------------------------------')
        print('Press Ctrl+C to interrupt ...\n\n')


    # ros interface
    pub = rospy.Publisher(topic, PilotRequest, tcp_nodelay=True, queue_size=1)
    sub = rospy.Subscriber(TOPIC_STATUS, PilotStatus, handle_status, tcp_nodelay=True, queue_size=1)
    srv = rospy.ServiceProxy(SRV_SWITCH, BooleanService)

    def enable_pilot():
        try:
            srv.call(request=True)
        except rospy.ServiceException as se:
            print('Pilot Service not available!')
            print(se)
            sys.exit(-1)

    # shutdown hook (intercepts Ctrl+C)
    def disable_pilot():
        try:
            print('Got interrupt! Disabling the pilot on exit!\n')
            srv.call(request=False)
        except rospy.ServiceException as se:
            print('Pilot Service not available!')
            print(se)

    rospy.on_shutdown(disable_pilot)

    # start the processing
    enable_pilot()

    # send initial burst
    for n in xrange(5):
        send_request()

    # main loop
    while not rospy.is_shutdown():
        # keep sending new requests only for specific types
        if type not in (TYPE_STY, TYPE_BOD):
            send_request()

        rospy.sleep(DEFAULT_SLEEP)


if __name__ == '__main__':
    main()
