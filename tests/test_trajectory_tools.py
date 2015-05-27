#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
import numpy as np
np.set_printoptions(precision=3, suppress=True)

sys.path.append('../src')

from vehicle_core.path.trajectory_tools import *


def demo_bezier(points=None):
    """Demonstrates the performance of interpolate_bezier function. Displays input points, interpolated
    trajectory and orientation at each point.

    :param points:  Example input points:
                        points = np.array([
                            [0, 0, 0, 0, 0, 0],
                            [5, -15, 0, 0, 0, 0],
                            [-5, 20, 0, 0, 0, 0],
                            [0, 10, 0, 0, 0, 0]
                        ])
    :return: None
    """
    if points is None:
        points = format_bezier_input(
            start=np.array([0, 0, 0, 0, 0, 0]),
            p1=np.array([15, 50]),
            p2=np.array([15, 50]),
            end=np.array([0, 10, 0, 0, 0, 0]),
            degrees=True
        )

    traj = interpolate_bezier(points)

    fig, ax = plt.subplots()
    ax.plot(aspect='equal')
    ax.plot(points[:, 1], points[:, 0], 'or')
    ax.plot(traj[:, 1], traj[:, 0], 'g')

    r = np.ones(traj.shape[0])     # use a fixed value for arrow length
    th = -traj[:, 5] + (np.pi / 2)                 # convert from navigation angles
    x, y = pol2cart(r, th)

    for n in xrange(traj.shape[0]):
        ax.arrow(traj[n,1], traj[n,0], x[n], y[n], fc="k", ec="k", head_width=0.05, head_length=0.1)

    plt.axis([-15, 15, -20, 10])

    ax.grid()
    plt.show()

def main():
    # test trajectories
    demo_bezier()
    origin = np.array([0, 0, 0, 0, 0, 0])

    # inspection scenario
    ips = np.array([
        [10, 10, 0, 0, 0, np.deg2rad(45)],
        [-5, 0, 0, 0, 0, np.deg2rad(-135)],
        [10, -5, 0, 0, 0, np.deg2rad(-45)],
    ])

    # inspection trajectory
    trajectory = np.zeros((1, 6))

    # set initial point
    trajectory[0, :] = origin
    bz_step = 6                     # meters

    for n in xrange(ips.shape[0]):
        p0 = trajectory[-1]
        p3 = ips[n]
        p1 = np.zeros_like(p0)
        p2 = np.zeros_like(p0)

        alpha = (-p0[5] + np.pi/2)
        p1[0] = bz_step * np.sin(alpha) + p0[0]
        p1[1] = bz_step * np.cos(alpha) + p0[1]

        # print('P0[5]: %s' % np.rad2deg(p0[5]))
        # print('ALPHA: %s' % np.rad2deg(alpha))
        #
        # print('P0: %s' % p0)
        # print('P1: %s' % p1)

        los = calculate_orientation(p0, p3)
        alpha = (-los + np.pi/2)

        p2[0] = -bz_step * np.sin(alpha) + p3[0]
        p2[1] = -bz_step * np.cos(alpha) + p3[1]

        #print('P2: %s' % p2)
        #print('P3: %s' % p3)

        points = interpolate_bezier_cubic((p0, p1, p2, p3), steps=20)
        trajectory = np.concatenate((trajectory, points), axis=0)

        points = interpolate_sector(ips[n], radius=8, sector=90, spacing=2)
        trajectory = np.concatenate((trajectory, points), axis=0)

        reverse = np.zeros((1, 6))
        reverse[0, 0:6] = trajectory[-1]
        reverse[0, 5] = wrap_angle(reverse[0, 5] - np.pi)
        trajectory = np.concatenate((trajectory, reverse), axis=0)

    # show final
    plot_trajectory(trajectory)
    plt.show()

    # print trajectory
    import yaml
    print(yaml.dump(traj_as_dict(trajectory)))


if __name__ == '__main__':
    main()
