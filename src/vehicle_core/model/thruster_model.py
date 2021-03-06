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

import numpy as np

#pythran export estimate_current(float[], float[][])
def estimate_current(throttle, coeff):
    """This function is calculating the current used by the thrusters from thruster commands

    It returns the predicted currents for each thruster using an exponential fitting.

    :param throttle: np.array of shape (N,)
    :param coeff: matrix (N,8) of model coefficients
    :return: estimated_current: np.array of shape (6,)
    """

    # init and check signs
    estimated_current = np.zeros_like(throttle)

    for i, th in enumerate(throttle):
        if th > 1:
            estimated_current[i] = coeff[i, 0] * np.exp(coeff[i, 1] * th) + coeff[i, 2] * np.exp(coeff[i, 3] * th)
        elif th < -1:
            estimated_current[i] = coeff[i, 4] * np.exp(coeff[i, 5] * th) + coeff[i, 6] * np.exp(coeff[i, 7] * th)
        else:
            estimated_current[i] = 0

    # take into account the effect of small speeds and approximation error (preventing negative values)
    return np.maximum(estimated_current, 0.0)


#pythran export estimate_forces(float[], float[], float[])
def estimate_forces(throttle, current, coeff):
    """This function calculates the thrust generated by thrusters from speeds and currents.

    The output is in kilograms as described in thruster's data sheet.

    :param throttle: thrusters input commands (throttle requested, percentage)
    :param current: thruster output currents (measured or predicted by the model, amps)
    :param coeff: equation coefficients: ax^3 + bx^2 + cx (if x >= 0) and dx^3 + ex^2 + fx (if x < 0)
    :return: estimated thrust generate by thrusters (in Newtons)
    """

    estimated_thrust = np.zeros(len(current))

    for i in xrange(len(current)):
        if throttle[i] >= 0:
            estimated_thrust[i] = coeff[0] * (current[i]**3) + coeff[1] * (current[i]**2) + coeff[2] * current[i]
        else:
            estimated_thrust[i] = -(coeff[3] * (current[i]**3) + coeff[4] * (current[i]**2) + coeff[5] * current[i])

    return estimated_thrust * 9.81


#pythran export estimate_throttle(float[], float[][], float[], float, float)
def estimate_throttle(force_request, coeff, linear, threshold, max_force):
    """This function calculates the throttle that needs to be sent to the thrusters to apply input
    forces. Inverse of T(i(v)) -> v(T) has been fitted using the thrusters data sheet, thus internally it use kilograms.
    
    :param force_request:   requested forces from the controller (N x 1) [Newtons]
    :param threshold:       symmetric limit for the linear approximation (1 x 1) [Newtons]
    :param coeff:           coefficient matrix for inverse non-linear model (N x 8)
    :param linear:          coefficient array for inverse linearized model (N x 1)
    :param max_force:       maximum model force (this is used to prevent wrong estimations)

    :return: throttle_required:     array of required throttle values to produce the requested force (N x 1)
    """

    # calculate required thrust and limit according to vehicle specs
    #   thrust_request: is in kilograms as the fitting is done using thruster's data sheet
    throttle_required = np.zeros_like(force_request)
    thrust_request = np.clip(force_request / 9.81, -max_force, max_force)

    for i, thrust in enumerate(thrust_request):
        if thrust > threshold:
            throttle_required[i] = coeff[i, 0] * np.exp(coeff[i, 1] * thrust) + coeff[i, 2] * np.exp(coeff[i, 3] * thrust)
        elif thrust < -threshold:
            throttle_required[i] = coeff[i, 4] * np.exp(coeff[i, 5] * -thrust) + coeff[i, 6] * np.exp(coeff[i, 7] * -thrust)
        else:
            throttle_required[i] = (linear[i] / threshold) * thrust

    return np.clip(throttle_required, -100, 100)
