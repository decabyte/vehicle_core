#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
sys.path.append('../src')

import numpy as np
np.set_printoptions(precision=3, suppress=True)

from vehicle_core.path.trajectory_tools import *


def demo_bezier(points, **kwargs):
    """Demonstrates the performance of interpolate_bezier function. Displays input points, interpolated
    trajectory and orientation at each point.

    :param points:  input for the interpolate_bezier function
    :return: None
    """
    # generate trajectory
    traj = interpolate_bezier(points, **kwargs)

    # plot trajectory and add support points
    fig, ax = plot_trajectory(traj, show_orientation=False)
    ax.plot(points[:, 1], points[:, 0], 'ob')

    return fig, ax

    # fig, ax = plt.subplots()
    # ax.plot(aspect='equal')
    # ax.plot(points[:, 1], points[:, 0], 'or')
    # ax.plot(traj[:, 1], traj[:, 0], 'g')
    #
    # r = np.ones(traj.shape[0])     # use a fixed value for arrow length
    # th = -traj[:, 5] + (np.pi / 2)                 # convert from navigation angles
    # x, y = pol2cart(r, th)
    #
    # for n in xrange(traj.shape[0]):
    #     ax.arrow(traj[n,1], traj[n,0], x[n], y[n], fc="k", ec="k", head_width=0.05, head_length=0.1)
    #
    # plt.axis([-15, 15, -20, 10])
    #
    # ax.grid()
    # plt.show()

def main():
    # test one
    points = format_bezier_input(
        start=np.array([0, 0, 0, 0, 0, 0]),
        p1=np.array([5, 50]),
        p2=np.array([5, 50]),
        end=np.array([0, 10, 0, 0, 0, 0]),
        degrees=True
    )

    fig, ax = demo_bezier(points)

    # test two
    points = np.array([
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [5.0, 5.0, 0.0, 0.0, 0.0, 0.0],
        [10.0, 10.0, 0.0, 0.0, 0.0, 0.0],
        [5.0, 20.0, 0.0, 0.0, 0.0, 0.0],
        [10.0, 30.0, 0.0, 0.0, 0.0, 0.0],
        [20.0, 40.0, 0.0, 0.0, 0.0, 0.0],

    ])

    fig, ax = demo_bezier(points)

    # bspline interpolation
    import scipy as sci
    import scipy.interpolate

    n = 100.0
    trj = np.zeros((n, 6))
    trj[:, 1] = np.linspace(points[0, 1], points[-1, 1], n)

    tck = sci.interpolate.splrep(points[:, 1], points[:, 0], k=3)
    trj[:, 0] = sci.interpolate.splev(trj[:, 1], tck)

    fig, ax = plot_trajectory(trj)

    # show plots
    plt.show()

if __name__ == '__main__':
    main()
