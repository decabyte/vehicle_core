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

from __future__ import division, absolute_import

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation

from vehicle_core.util import trajectory_tools as tt
from vehicle_core.util import search_patterns as sp
from vehicle_core.path import path_strategy as ps


def plot_run(points, requested_points, vehicle_points, animate=False):
    fig, ax = tt.plot_trajectory(points)

    tt.plot_trajectory(requested_points, fig=fig, ax=ax, p_style='b', show_orientation=False)
    tt.plot_trajectory(vehicle_points, fig=fig, ax=ax, p_style='g')

    # add vehicle
    patch = plt.Rectangle((0, 0), .6, 1.3)

    if animate:
        def init():
            patch.set_x(10000)
            patch.set_y(10000)
            ax.add_patch(patch)
            return patch,

        def animate(t):
            x = vehicle_points[t, 1]
            y = vehicle_points[t, 0]
            angle = vehicle_points[t, 5]
            patch.set_x(x - 0.3*np.cos(angle) - 0.65*np.sin(angle))
            patch.set_y(y - 0.65*np.cos(angle) + 0.3*np.sin(angle))
            patch._angle = -np.rad2deg(angle)
            return patch,

        anim = mpl.animation.FuncAnimation(
            fig, animate, init_func=init, frames=len(vehicle_points),
            interval=70, blit=True
        )

    plt.show()


def demonstrate_modes(points, mode, iterations, animate=False, step=0.3, **kwargs):
    if mode == 'simple':
        path_mode = ps.SimpleStrategy(points[1:], position=points[0])
    elif mode == 'lines':
        path_mode = ps.LineStrategy(points[1:], position=points[0], spacing=1.0)
    elif mode == 'fast':
        path_mode = ps.FastTimeStrategy(points[1:], position=points[0], look_ahead=3.0)
    else:
        return

    requested_points = np.zeros((iterations-1, 6))
    vehicle_points = np.zeros((iterations, 6))
    vehicle_points[0] = points[0]

    for i in xrange(1, iterations):
        path_mode.update(vehicle_points[i-1], [])
        requested_points[i-1] = path_mode.des_pos
        vehicle_points[i] = move_vehicle(vehicle_points[i-1], path_mode.des_pos, step)

    plot_run(points, requested_points, vehicle_points, animate)


def move_vehicle(current_position, desired_position, step):
    direction = desired_position[0:3] - current_position[0:3]
    modulus = np.linalg.norm((desired_position[0:3] - current_position[0:3]))

    if modulus > 0:
        direction_norm = direction/modulus
    else:
        direction_norm = 0

    next_position = np.zeros(6)
    next_position[0:3] = current_position[0:3] + step * direction_norm
    next_position[3:6] = desired_position[3:6]

    return next_position


if __name__ == '__main__':
    # test script
    center = np.array([0, 0, 0, 0, 0, 0])

    # points = sp.circular_pattern(center, radius=50.0)
    # points = sp.square_pattern(center, distance=30.0)
    # points = sp.nested_square_pattern(center, ext_dist=30.0, int_dist=10.0, spacing=5.0)
    points = sp.spiral_pattern(center, distance=20.0, spacing=5.0)

    # points = np.array([[0, 0, 0, 0, 0, 3.14],
    #                    [150, 5, 0, 0, 0, 0],
    #                    [0, 10, 0, 0, 0, 0],
    #                    [10, 25, 0, 0, 0, 0]])

    # traj = tt.interpolate_trajectory(points, 50, face_goal=False)
    # tt.plot_trajectory(traj)
    demonstrate_modes(points, 'fast', iterations=2000, animate=True)

    A = np.array([10, -10, 0, 0, 0, 0])
    B = np.array([-20, 10, 0, 0, 0, 0])

    points = np.array([
        [0, 0, 0, 0, 0, 0],
        [5, -15, 0, 0, 0, 0],
        [-5, 20, 0, 0, 0, 0],
        [0, 10, 0, 0, 0, 0]
    ])

    # traj = tt.interpolate_arc(A, B, radius=1, spacing=1, right=True)
    # tt.plot_trajectory(traj)

    # traj = tt.interpolate_bezier_cubic(points)
    # tt.plot_trajectory(traj)

    # traj = tt.interpolate_bezier_general(points)
    # tt.plot_trajectory(traj)
    #
    # plt.show()
