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

"""Thruster Configuration Module

This module is providing importable values related to the thrusters parameters.
The units are SI units and MUST be specified in comments when using these values for calculations.

Note on Thruster Allocation Matrix:
    The matrix specifies how much force each thruster (column) will generate on each axis (row) if it is producing
    1N of thrust.

Note on Inverse of Thruster Allocation Matrix:
    The matrix specifies how much thrust will have to produced by each thruster (row) if 1N is requested on a given
    axis (column).
"""

from __future__ import division

import numpy as np
np.set_printoptions(precision=5, suppress=True)     # numpy visualization options

import scipy as sci
import scipy.signal


# Thrusters Maximum Throttle
#   empirically measured to achieve better stability across all thrusters (on Nessie AUV)
MAX_THROTTLE = 85

# Thruster Maximum Force
#   calculated from thruster data sheet (kg * g = N)
MAX_FORCE = 4 * 9.81                                                 # force of as single thruster (in newtons from kg)
MAX_U = np.array([78.48, 63.2084, 61.33067, 0., 41.3982, 51.97198])  # max force for every axis in N and Nm


# Thruster Linear Throttle Zone
#   empirically measured to improve thrusters linearization (on Nessie AUV)
#   the interpolation on thruster vert-front is not following the same behaviour of other thrusters
#   therefore the linear zone is different and here we take into account this factor (5 instead of 15)
LINEAR_THROTTLE = np.array([15.0, 15.0, 15.0, 15.0, 5.0, 15.0])

# Inverse Thruster Modelling
#   model threshold:    any force or thrust requested below this threshold should translate to a zero throttle request
THRUST_THRESHOLD = 0.10

# Thruster Back Constant
#   this is taking into account the fact the thruster is less efficient when reversing
FEB = 1.108555691


# Thruster Coordinates (from nessie measurements)
#   TC = [ dx dy dz ]   (displacements from the center of the vehicle in meters)
#   for reference see: "Guidance and Control of Oceans Vehicles - Thor I. Fossen"
# TC = np.array([
#     [-0.270, -0.1425, 0.000],   # fwd-port
#     [-0.270, 0.1425, 0.000],    # fwd-stdb
#     [-0.790, 0.000, -0.050],   # lat-rear
#     [0.470, 0.000, 0.050],     # lat-front
#     [-0.675, 0.000, 0.000],    # vert-rear
#     [0.380, 0.000, 0.000],     # vert-front
# ])

# Thruster Positions as measured in the CAD model (Gordon F. 2014/09/30)
TC = np.array([
    [-0.325, -0.185, 0.000],   # fwd-port
    [-0.325, 0.185, 0.000],    # fwd-stdb
    [-0.730, 0.093, -0.052],   # lat-rear
    [0.530, -0.080, 0.053],     # lat-front
    [-0.625, 0.000, 0.104],    # vert-rear
    [0.435, 0.000, 0.075],     # vert-front
])


# Nessie Thruster Allocator Matrix (6x6)
#   rows:       degrees of freedom
#   columns:    thrusters mapping   (fwd-port, fwd-std, lat-rear, lat-front, vert-rear, vert-front)
#
#   The sign of the lever arm is taking into account the change needed in the rotation of the propeller
#   to move the vehicle in the direction requested by the controller, use extreme caution when changing this terms.
#
#   The pitch contribution need to be negated in order to produce the expected behaviour.
#   Similar approach is needed when considering the forward thrusters for yawing. For right rotation the port thruster
#   spins forward while the starboard one needs to reverse. Vertical thruster are fitted with a normal direction towards
#   the sea bottom in this way the make the depth less stressful for the vehicle itself but they require to be inverted
#   for the pitch control (taken into account in this TAM).
TAM = np.array([
    [1.0,       1.0,        0.0,        0.0,        0.0,        0.0],           # surge
    [0.0,       0.0,        1.0,        1.0,        0.0,        0.0],           # sway
    [0.0,       0.0,        0.0,        0.0,        1.0,        1.0],           # heave
    [0.0,       0.0,        0.0,        0.0,        0.0,        0.0],           # roll (no control)
    [0.0,       0.0,        0.0,        0.0,        -TC[4,0],   -TC[5,0]],      # pitch
    [-TC[0,1],   -TC[1,1],    TC[2,0],    TC[3,0],    0.0,        0.0]          # yaw (full control)
])


# Inverted Thruster Allocator Matrix (6x6)
#   ideally you want to use "np.linalg.inv(TAM)" but TAM is not always invertible
#   so we choose to use the pseudo-inverse, which in numpy is implemented using SVD decomposition (have fun!)
inv_TAM = np.linalg.pinv(TAM)


# Thruster Modelling (output current from input throttle)
#   LPF:    low-pass filter (is an IIR butterworth with fixed values derived from data)
#   I_V:    every row contains coefficients for the exp function modelling a thruster (including pos and neg speeds)
#
#   current(throttle) = a * exp(b * throttle) + c * exp(d * throttle)      (if throttle > 0)
#   current(throttle) = a * exp(b * throttle) + c * exp(d * throttle)      (if throttle < 0)
#
THROTTLE_TO_CURRENT = np.array([
    [0.958178738366333, 0.035557825709257, -0.999987180620940, 0.029896654219676, 0.919672607913601, -0.034267736621498,-0.999996766056053, -0.026332361274153],
    [7.625837714498504e+02, 0.033680300978892, -7.626304021183255e+02, 0.033673595392087, 0.916187502921983, -0.033883448804055, -0.999968304982479, -0.026510595330895],
    [0.937204912906785, 0.039942475630232, -0.971022296566244, 0.034851226676258, 0.516079153100964, -0.041702815664742, -0.561846809725246, -0.031795554139631],
    [0.971135445587198, 0.039808946465358, -0.999979968065087, 0.035094537629239, 0.965101682977174, -0.038056624637157, -0.999983066164792, -0.032675971993315],
    [-1.365305176176211e-05, 0.135838593148505, 0.099950112333818, 0.053216537773530, 0.044334000825067, -0.047744353557295, 0.089467591902870, -0.047686120767187],
    [0.962692330790658, 0.035764427358022, -0.999993314925484, 0.030164375053663, 0.962734269712297, -0.036681021091778, -0.999999313750861, -0.030990900767058]

])

# Thruster Modelling (generated thruster from output current)
#
#   thrust(current) = a * i^3 + b * i^2 + c * i      (if i > 0)
#   thrust(current) = d * i^3 + e * i^2 + f * i      (if i < 0)
#
CURRENT_TO_THRUST = np.array([0.00610, -0.112, 1.03, 0.00399, -0.0877, 0.950])


# Inverse Thruster Modelling (required throttle from requested thrust)
#
# throttle(thrust)  a * exp(b * x) + c * exp(d * x)   (if i > 0)
#                   e * exp(f * -x) + g * exp(h * -x)   (if i < 0)
#
# each row corresponds to a different thruster:
#   [a, b, c, d, e, f, g, h]
#
THRUST_TO_THROTTLE = np.array([
    [42.854169495743875, 0.190418760347136, -37.336531104280155, -2.758812449806600, -42.038362361850886, 0.190506636122806, 35.787253101217395, -2.803218349698363],
    [44.779623662569655, 0.179974767492064, -35.399521042988709, -2.460630548534455, -42.512332419819252, 0.199416365760409, 35.387238572844602, -3.172607723527273],
    [44.676472151574288, 0.150257475936502, -38.490125630619403, -2.409979160002929, -44.867194912697457, 0.151684062621202, 35.858484743491843, -2.325374374451807],
    [46.302240431343613, 0.141834031209996, -41.124503670164458, -2.208721244488205, -44.499099394045068, 0.162827359789975, 38.913980432374998, -2.606222011412692],
    [45.691981455875791, 0.155636712004254, -47.272386331380808, -1.816994467894909, -42.125932688352009, 0.189544579573439, 48.830681820629472, -2.392191827351297],
    [44.772496022010827, 0.173593142053517, -36.383596251516934, -2.133464912729834, -43.248375165595057, 0.179827504584565, 37.522929806266326, -2.767770782316266]
])


# Thruster Modelling (prediction of thruster feedback throttle given throttle request)
#   this is done using a low-pass filtering cascaded with a rate limiter
#   these are the parameters of this two components
#
LPF_FS = 10             # topic sampling frequency (Hz)
LPF_ORDER = 1           # order of the butterworth filter
LPF_CUTOFF = 0.55       # low-pass filter cutoff frequency (Hz) - adjusted with sine_test_20140625 data
LPF_WINDOW = 40         # lpf buffer length (samples)
LPF_DELAY = 1           # number of samples to delay (modelling of ROS + thruster latency)

# low pass filter (b)
LPF = sci.signal.butter(LPF_ORDER, LPF_CUTOFF / (LPF_FS / 2), 'low', analog=False)

# rate limiter
THROTTLE_DELAY = 1              # (samples)
THROTTLE_RISING_LIMIT = 5.5     # positive rate (throttle units) was 5.0
THROTTLE_FALLING_LIMIT = 5.5    # negative rate (throttle units) was 3.0
