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

"""Lawnmower pattern generator module for Autonomous Underwater Vehicle.

This module ...
"""

from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)
from numpy import cos, sin

import trajectory_tools as tg


def pattern_from_rect(width, height, delta=0.0, start=0):
    """
        area = [    
            A[n ,e, d],
            B[n ,e, d],
            C[n ,e, d],
            D[n ,e, d]
        ]

        
    area: represent the bounding box of the lawnmower pattern
        A ------------------- B
        |                     |
        |                     |
        |                     |
        D ------------------- C

    
    returns (points, n_cols): 
    where points are the lawnmower pattern points within the bounding box

        1       4 ----- 5
        |       |       |
        |       |       |
        2 ----- 3       6 ----- *

    :param width:
    :param height:
    :param delta:
    :param start:
    :return:
    """

    # trajectory matrix
    n_cols = np.ceil(width / delta) + 1      # number of columns (+ final)
    n_npc = 2                                # number of points per column
    n_points = n_npc * n_cols                # number of matrix rows

    # init empty trajectory
    trajectory = np.zeros((n_points, 6))
    depth = 0
    east = 0
    idx = 0

    # trajectory loop
    for n in np.arange(0, n_cols):  
        idx = n_npc * n

        if n % 2 == 0:
            # first leg
            trajectory[idx, :] = np.array([0, east, depth, 0, 0, np.pi])
            trajectory[idx+1, :] = np.array([-height, east, depth, 0, 0, np.pi])
        else:
            # second leg
            trajectory[idx, :] = np.array([-height, east, depth, 0, 0, 0])
            trajectory[idx+1, :] = np.array([0, east, depth, 0, 0, 0])

        # update east for next pair of legs
        east += delta

    # handle last column
    if trajectory[idx, 1] > width:
        trajectory[idx, 1] = width
        trajectory[idx+1, 1] = width

    # rotate and translate according to the starting point
    if start == 1:
        # starting point is B
        trajectory[:,1] = (-trajectory[:,1]) + width

    elif start == 2:
        # starting point is C
        trajectory[:,0] = (-trajectory[:,0]) - height
        trajectory[:,1] = (-trajectory[:,1]) + width

    elif start == 3:
        # starting point is D
        trajectory[:,0] = (-trajectory[:,0]) - height

    else:
        pass

    return (trajectory, n_cols)


def pattern_from_ned(area, start=0, spacing=2.0, overlap=0.0):
    """

    :param area:
    :param start:
    :param spacing:
    :param overlap:
    :return:
    """
    # # check A,B,C,D are in the expected order and A,B,C,D[depth] are equals
    # conditions = [
    #     np.all(area[0,0] > area[2:4,0]),    # A[north] > C,D[north]
    #     np.all(area[0,1] < area[1:3,1]),    # A[east] < B,C[east]
    #     area[0,0] == area[1,0],             # A[north] == B[north]
    #     area[0,1] == area[3,1],             # A[east] == D[east]
    #
    #     np.all(area[2,0] < area[0:2,0]),    # C[north] < A,B[north]
    #     np.all(area[2,1] > area[[0,3],1]),  # C[east] > A,D[east]
    #     area[2,0] == area[3,0],             # C[north] == D[north]
    #     area[2,1] == area[1,1],             # C[east] == B[east]
    #
    #     np.all(area[0,2] == area[:,2])      # depth are equals
    # ]
    #
    # if not np.all(conditions):
    #     raise ValueError('area is not rectangular shaped!')

    # TODO: insert another condition like ab == cd and bc == da
    a = np.linalg.norm(area[0,0:2] - area[1,0:2])   # AB
    b = np.linalg.norm(area[1,0:2] - area[2,0:2])   # BC
    c = np.linalg.norm(area[2,0:2] - area[3,0:2])   # CD
    d = np.linalg.norm(area[3,0:2] - area[0,0:2])   # DA

    diag_1 = (a**2 + b**2)
    diag_2 = (c**2 + d**2)

    if not np.abs(diag_2 - diag_1) < 1e-6:
        raise ValueError('area is not rectangular shaped!')

    # calculate bounding box dimensions
    dW = area[1,:2] - area[0,:2]    # B - A (xy)
    dH = area[3,:2] - area[0,:2]    # D - A (xy)

    width = np.sqrt(np.dot(dW, dW))
    height = np.sqrt(np.dot(dH, dH))
    depth = np.abs(area[0,2])

    # calculate delta from range and overlap
    delta = float(spacing - (spacing * overlap))

    # generate using simple geometry
    (fixes, n_cols) = pattern_from_rect(width, height, delta, start)

    # adjust depth
    fixes[:,2] = depth

    # translations (using A point to get correct position in space)
    fixes[:,0] += area[0,0]
    fixes[:,1] += area[0,1]

    # correct initial yaw
    dE = area[1,1] - area[0,1]              # delta_east between B and A
    alpha = np.arccos(dE / width)           # angle between B and A respect to NED reference
    fixes[:,5] += alpha                     # add the rotation to fixes

    # rotate waypoints
    ROT = np.eye(6)

    # set the rotation using current attitude
    ROT[0:2, 0:2] = [
        [cos(alpha),    sin(alpha)],
        [-sin(alpha),   cos(alpha)]
    ]

    for n in range(fixes.shape[0]):
        fixes[n,:] = np.dot(ROT, fixes[n,:])

    return (fixes, n_cols)


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    np.set_printoptions(precision=3, suppress=True)     # reduce numpy output length

    area = np.array([
        [4.4, -9.8, 1.0],            # A
        [4.4, -1.8, 1.0],           # B
        [1.0, -1.8, 1.0],         # C
        [1.0, -9.8, 1.0]           # D
    ])

    # # rotation case
    # area = np.array([
    #     [0,0,2],                  # A
    #     [10,10,2],                # B
    #     [0, np.sqrt(2) * 10,2],   # C
    #     [-10,10,2]                # D
    # ])

    # sonar parameters
    sonar_field = 1        # meters
    sonar_overlap = 0      # 0 to 1

    # get global points
    (fixes_a, n_cols) = pattern_from_ned(area, start=0, spacing=sonar_field, overlap=sonar_overlap)
    # print(fixes_d)

    # (fixes_b, n_cols) = pattern_from_ned(area, start=1, spacing=sonar_field, overlap=sonar_overlap)
    # print(fixes_d)

    #(fixes_c, n_cols) = pattern_from_ned(area, start=2, spacing=sonar_field, overlap=sonar_overlap)
    #print(fixes_c)

    #(fixes_d, n_cols) = pattern_from_ned(area, start=3, spacing=sonar_field, overlap=sonar_overlap)
    #print(fixes_d)

    tg.plot_trajectory(fixes_a)
    plt.show()

    import json
    data = dict()
    data['points'] = fixes_a.tolist()

    print(json.dumps(data))


    # import matplotlib.pyplot as plt
    #
    # fig = plt.figure()
    # #plt.plot(fixes_a[:,1], fixes_a[:,0], 'or-')
    # #plt.plot(fixes_b[:,1], fixes_b[:,0], '*g--')
    # #plt.plot(fixes_c[:,1], fixes_c[:,0], 'ob-')
    # plt.plot(fixes_d[:,1], fixes_d[:,0], 'oy-')
    # plt.grid()
    # plt.show()
