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

"""Search pattern generator module for Autonomous Underwater Vehicle.

This module ...
"""

from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

from vehicle_core.path import trajectory_tools as tt


def circular_pattern(center, radius=5.0, orientation=None, delta_ip=5.0):
    """Generate circular pattern given a center (6DOF-vector) and a radius (meters).

       *orientation* specifies if the generated points in 6DOF space need to face 
       the center (using None as default value) or a given yaw angle (radians value)

    :param center:
    :param radius:
    :param orientation:
    :param delta_ip:
    :return:
    """

    TWO_PI = 2 * np.pi
    length = TWO_PI * radius                # meters
    sector = delta_ip / radius              # radians
    steps = np.round(TWO_PI / sector)       # intermediate points

    pattern = np.zeros((steps, 6))          # trajectory points
    theta = np.linspace(0, 2*np.pi, num=steps)

    pattern[:,0] = (radius * np.cos(theta)) + center[0]     # x-coordinate
    pattern[:,1] = (radius * np.sin(theta)) + center[1]     # y-coordinate
    pattern[:,2] = center[2] * np.ones(steps)               # z-coordinate

    if orientation is None:
        pattern[:,5] = theta - np.pi
    else:
        pattern[:,5] = np.max((-np.pi, np.min((orientation, np.pi))))

    return pattern

def nester_circular_pattern(center, ext_radius=30.0, int_radius=10.0, spacing=3.0, **kwargs):
    """

    :param center:
    :param ext_radius:
    :param int_radius:
    :param spacing:
    :param kwargs:
    :return:
    """
    orientation = kwargs.get('orientation', None)
    delta_ip = kwargs.get('delta_ip', 5.0)

    delta_radii = ext_radius - int_radius
    arms = np.round(delta_radii / spacing)      # spiral arms
    pattern = np.zeros((1, 6))                  # trajectory points

    for a in np.arange(arms):
        r_arm = (spacing * a) + int_radius
        a_pat = circular_pattern(center, r_arm, orientation, delta_ip)

        pattern = np.concatenate((pattern, a_pat), axis=0)

    return pattern[1:,:]


def square_pattern(center, distance=5.0, orientation=None, delta_ip=5.0):
    """Generate square pattern given a center (6DOF-vector) and a distance (meters) from it.

       *orientation* specifies if the generated points in 6DOF space need to face 
       the center (using None as default value) or a given yaw angle (radians value)

    :param center:
    :param distance:
    :param orientation:
    :param delta_ip:
    :return:
    """

    x_min = center[0] - distance
    x_max = center[0] + distance
    y_min = center[1] - distance
    y_max = center[1] + distance

    dx = x_max - x_min
    steps = np.floor(dx / delta_ip)
    pattern = np.zeros((4 * steps, 6))

    # first leg (top)
    pattern[0:steps, 0] = np.linspace(x_min, x_max, num=steps)
    pattern[0:steps, 1] = y_max
    pattern[0:steps, 5] = 0

    # second leg (right)
    pattern[steps:2*steps, 0] = x_max
    pattern[steps:2*steps, 1] = np.linspace(y_max, y_min, num=steps)
    pattern[steps:2*steps, 5] = -0.5 * np.pi

    # third leg (bottom)
    pattern[2*steps:3*steps, 0] = np.linspace(x_max, x_min, num=steps)
    pattern[2*steps:3*steps, 1] = y_min
    pattern[2*steps:3*steps, 5] = np.pi

    # forth leg (left)
    pattern[3*steps:4*steps, 0] = x_min
    pattern[3*steps:4*steps, 1] = np.linspace(y_min, y_max, num=steps)
    pattern[3*steps:4*steps, 5] = 0.5 * np.pi

    return pattern

def nested_square_pattern(center, ext_dist=30.0, int_dist=10.0, spacing=3.0, **kwargs):
    """

    :param center:
    :param ext_dist:
    :param int_dist:
    :param spacing:
    :param kwargs:
    :return:
    """
    orientation = kwargs.get('orientation', None)
    delta_ip = kwargs.get('delta_ip', 5.0)

    delta_dist = ext_dist - int_dist
    arms = np.round(delta_dist / spacing)       # spiral arms
    pattern = np.zeros((1, 6))                  # trajectory points

    for a in np.arange(arms):
        r_arm = (spacing * a) + int_dist
        a_pat = square_pattern(center, r_arm, orientation, delta_ip)

        pattern = np.concatenate((pattern, a_pat), axis=0)

    return pattern[1:,:]


def spiral_pattern(center, distance=30.0, spacing=10.0, **kwargs):
    """Generate spiral pattern given a center (6DOF-vector) and a distance (meters) from it.

       *orientation* specifies if the generated points in 6DOF space need to face 
       the spiral direction (using None as default value) or a given yaw angle (radians value)

    :param center:
    :param distance:
    :param spacing:
    :param kwargs:
    :return:
    """

    orientation = kwargs.get('orientation', None)
    delta_ip = kwargs.get('delta_ip', 5.0)

    arms = np.round(distance / spacing)
    coeff = np.array([n+3 for n in np.arange(arms)])
    length = np.sum(2**coeff) * spacing

    points = np.ceil(length / spacing)
    
    # trajectory matrix
    pattern = np.zeros((points, 6))

    # start from center
    pattern[0,:] = np.array(center)

    # loop counters
    cx = 0
    dx = 0
    mx = 1

    for n in np.arange(1, points):
        pattern[n,:] = np.copy(pattern[n-1,:])
        cx = np.mod(dx,4)

        if cx == 0:
            pattern[n,0] = pattern[n,0] + spacing   # up
            pattern[n,5] = 0
        elif cx == 1:
            pattern[n,1] = pattern[n,1] + spacing   # right
            pattern[n,5] = np.pi / 2
        elif cx == 2:
            pattern[n,0] = pattern[n,0] - spacing   # down
            pattern[n,5] = np.pi
        else:
            pattern[n,1] = pattern[n,1] - spacing   # left
            pattern[n,5] = -np.pi / 2

        # check if a leg was completed
        if np.mod(n,mx) == 0:
            dx += 1

            # check if two leg were completed (up-right)
            if np.mod(dx,2) == 0:
                mx += 1

    # correct orientation if a value is given
    if orientation is not None:
        pattern[:,5] = orientation

    return pattern



if __name__ == '__main__':
    import matplotlib.pyplot as plt

    # initial point
    center = [0, 0, 5, 0, 0, 0]
    
    # circular test
    #points = circular_pattern(center, radius=30.0)
    #print(points)

    # nested circular test
    points = nester_circular_pattern(center, ext_radius=40.0, int_radius=10.0, spacing=10.0, delta_ip=6.0)
    #points = tt.interpolate_trajectory(points, spacing=3.0)
    print(points)

    # square test
    #points = square_pattern(center, distance=30.0)
    #print(points)

    # square spiral test
    #points = nested_square_pattern(center, ext_dist=30.0, int_dist=10.0, spacing=5.0)
    #print(points)

    # spiral test
    #points = spiral_pattern(center, distance=15.0, spacing=5)
    #print(points)

    tt.plot_trajectory(points)
    plt.show()

    import yaml
    print(yaml.dump(tt.traj_as_dict(points)))

