#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
import argparse

import numpy as np

np.set_printoptions(precision=3, suppress=True)

from numpy import cos, sin

# fix imports
sys.path.append('../src')

from vehicle_core.path import trajectory_tools as tt


# constants
T_TYPES = [
    'surge',
    'sway',
    'heave',
    'yaw',
    'surge+heave',
    'sway+heave',
    'yaw+heave'
]


def main():
    parser = argparse.ArgumentParser(description="Utility for generating navigation trajectories used by navigator module.", 
        epilog="This is part of vehicle_pilot module.")

    parser.add_argument('type', choices=T_TYPES, metavar='type', help='Specify the DOFs used by the trajectory.')
    parser.add_argument('n', type=float, help='Initial NORTH coordinate.')
    parser.add_argument('e', type=float, help='Initial EAST coordinate.')
    parser.add_argument('d', type=float, help='Initial DEPTH coordinate.')
    parser.add_argument('y', type=float, help='Initial YAW coordinate.')
    parser.add_argument('delta_dof', type=float, metavar='delta_dof', help='Maximum displacement in <type> trajectory.')

    parser.add_argument('--output', default='json', help='Output trajectory format.')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print detailed information.')

    args = parser.parse_args()

    # check displacement
    if args.delta_dof < 1 or args.delta_dof > 15:
        print('Could not generate trajectory with {} maximum displacement.\n'.format(args.delta_dof))
        sys.exit(1)

    if args.d < 0 or args.d > 3:
        print('Could not generate trajectory with {} maximum displacement.\n'.format(args.delta_dof))
        sys.exit(1)

    if args.y > np.pi or args.y < -np.pi:
        print('Could not generate trajectory with {} yaw angle (-pi, pi).\n'.format(args.y))
        sys.exit(1)

    # waypoints matrix
    C = 10
    N = 2 * C + 1
    WPS = np.zeros((N, 6))

    # initial position
    INIT = np.array([args.n, args.e, args.d, 0, 0, args.y])
    WPS = np.tile(INIT, (N, 1))

    # displacements
    dw = [args.delta_dof]

    # select geometry
    if args.type == 'surge':
        dof = [0]
    elif args.type == 'sway':
        dof = [1]
    elif args.type == 'heave':
        dof = [2]
        dw = [min(args.delta_dof, 3)]
    elif args.type == 'yaw':
        dof = [5]
    elif args.type == 'surge+heave':
        dof = [0,2]
        dw = [args.delta_dof, min(args.delta_dof, 3)]
    elif args.type == 'sway+heave':
        dof = [1,2]
        dw = [args.delta_dof, min(args.delta_dof, 3)]
    elif args.type == 'yaw+heave':
        dof = [5,2]
        dw = [args.delta_dof, min(args.delta_dof, 3)]
    else:
        print('Could not generate {} trajectory geometry.\n'.format(args.type))
        sys.exit(1)

    # trajectory generation
    for i,d in enumerate(dof):
        w_max = WPS[0, d] + dw[i]
        w_min = WPS[0, d] + np.ceil(dw[i] / N)

        WPS[1::2, d] = np.linspace(w_max, w_min, num=C)
        WPS[2::2, d] = WPS[0, d] * np.ones((C,))

    # compensate for initial yaw
    ROT = np.eye(6)     # rotation matrix
    r = 0               # roll
    p = 0               # pitch
    y = WPS[0,5]        # yaw

    # set the rotation using current attitude
    ROT[0:2, 0:2] = [
        [cos(p)*cos(y),    cos(r)*sin(y)+sin(r)*sin(p)*cos(y)],
        [-cos(p)*sin(y),   cos(r)*cos(y)-sin(r)*sin(p)*sin(y)]
    ]

    # apply rotation
    WPR = np.dot(WPS, ROT)

    # trajectory export
    spec = {
        'type': args.type,
        'delta': dw,
        'dof': dof
    }

    if args.verbose:
        print(WPS)
        print(' ')
        print(WPR)
        print(' ')

        tt.plot_trajectory(WPR, arrow_length=0.2)

    # print final trajectory
    try:
        import yaml
        print(yaml.dump(tt.traj_as_dict(WPR,  **spec)))
    except ImportError:
        import json
        print(json.dumps(tt.traj_as_dict(WPR, **spec)))


if __name__ == '__main__':
    main()
