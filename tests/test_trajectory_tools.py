#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
sys.path.append('../src')

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

import scipy as sci
import scipy.interpolate

np.set_printoptions(precision=3, suppress=True)
mpl.style.use('bmh')


from vehicle_core.path.trajectory_tools import *


def bspline_interpolation(points, steps=100, k=3, **kwargs):
    """Interpolate trajectory using the bspline interpolation of grade k.

    :param points: supporting points for bspline interpolation
    :param steps: number of waypoint to generate
    :return: wps, numpy array (steps, 6)
    """
    wps = np.zeros((steps, 6))
    wps[:, 1] = np.linspace(points[0, 1], points[-1, 1], steps)

    tck = sci.interpolate.splrep(points[:, 1], points[:, 0], k=k)
    wps[:, 0] = sci.interpolate.splev(wps[:, 1], tck)

    # calculate the xy slope at each point of the curve
    der_x = np.diff(wps[:, 0])
    der_y = np.diff(wps[:, 1])
    wps[1:, 5] = np.arctan2(der_y, der_x)

    return wps


def plot_dof_trajectory(wps, dof=0):
    # speed and accelerations
    vel = np.zeros_like(wps)
    acc = np.zeros_like(wps)
    jrk = np.zeros_like(wps)

    vel[1:, :] = np.diff(wps, axis=0)
    acc[1:, :] = np.diff(vel, axis=0)
    jrk[1:, :] = np.diff(acc, axis=0)

    fig, ax = plt.subplots()
    ax.plot(vel[:, dof], label='velocity')
    ax.plot(acc[:, dof], label='acceleration')
    ax.plot(jrk[:, dof], label='jerk')

    ax.legend(loc='best')

    return fig, ax


def main():

    # # test one
    # points = format_bezier_input(
    #     start=np.array([0, 0, 0, 0, 0, 0]),
    #     P1=np.array([5, 50]),
    #     P2=np.array([5, 50]),
    #     end=np.array([0, 10, 0, 0, 0, 0]),
    #     degrees=True,
    # )
    #
    # wps = interpolate_bezier(points)
    #
    # # plot trajectory and add support points
    # fig, ax = plot_trajectory(wps, show_orientation=False)
    # ax.plot(points[:, 1], points[:, 0], 'ob')

    # test two
    points = np.array([
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [5.0, 5.0, 0.0, 0.0, 0.0, 0.0],
        [10.0, 10.0, 0.0, 0.0, 0.0, 0.0],
        [5.0, 20.0, 0.0, 0.0, 0.0, 0.0],
        [10.0, 30.0, 0.0, 0.0, 0.0, 0.0],
        [20.0, 40.0, 0.0, 0.0, 0.0, 0.0],

    ])

    wps = interpolate_bezier(points)

    # plot trajectory and add support points
    fig, ax = plot_trajectory(wps, show_orientation=True)
    ax.plot(points[:, 1], points[:, 0], 'ob')

    #plot_dof_trajectory(wps)


    # test three
    wps = bspline_interpolation(points, k=3)

    fig, ax = plot_trajectory(wps, show_orientation=True)
    ax.plot(points[:, 1], points[:, 0], 'ob')

    #plot_dof_trajectory(wps)


    # show plots
    plt.show()

if __name__ == '__main__':
    main()
