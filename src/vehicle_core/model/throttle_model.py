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

from __future__ import division

import numpy as np
import scipy as sci
import scipy.signal

##pythran export predict_throttle(float[], float[], float[], float, float)
def predict_throttle(throttle_request, b, a, offset, limit):
    """This function returns the predicted throttle for each thruster given a throttle request using a low-pass filter
    IIR filtering. See (http://en.wikipedia.org/wiki/Infinite_impulse_response) for more details.

    The use of scipy is not possible if the pythran optimizer is employed with this module.

    :param throttle_request: matrix of throttle request (N x M) (rows are different thrusters and columns are samples)
    :param b: low-pass filter b coefficients
    :param a: low-pass filter a coefficients
    :param offset: samples offset in the throttle request
    :param limit: throttle value hard limit
    :return: throttle_model is the predicted value of the throttle
    """

    # apply latency delay (offset is positive)
    throttle_delayed = throttle_request[:, 0:-(offset + 1)]
    throttle_model = np.zeros_like(throttle_delayed)

    # apply low-pass filter (using scipy)
    throttle_model = sci.signal.lfilter(b, a, throttle_delayed)

    # # apply low-pass filter (using custom implementation)
    # P = len(b)
    # Q = len(a)
    # N = throttle_delayed.shape[0]
    # M = throttle_delayed.shape[1]
    # K = np.maximum(P, Q)
    #
    # for i in xrange(N):
    #     for j in xrange(K, M):
    #
    #         x = throttle_delayed[i, j-P:j]
    #         y = throttle_model[i, j-Q:j-1]
    #
    #         throttle_model[i,j] = (np.sum(b[::-1] * x) - np.sum(a[:0:-1] * y)) / a[0]

    # calculate the result and apply limits
    return np.clip(throttle_model[:,-1], -limit, limit)


##pythran export rate_limiter(float[], float[], float, float)
def rate_limiter(new_throttle, last_throttle, rising_limit, falling_limit):
    """Models the change in thruster's throttle.

    http://www.mathworks.co.uk/help/simulink/slref/ratelimiter.html

    :param last_throttle: result of a previous iteration
    :param new_throttle:
    :param rising_limit: rising rate limit between two samples
    :param falling_limit: falling rate limit between two samples
    :return: next_throttle: the new throttle after applying rate limits
    """
    diff_throttle = new_throttle - last_throttle
    next_throttle = np.zeros_like(new_throttle)

    for i, dth in enumerate(diff_throttle):
        if dth > rising_limit:
            next_throttle[i] = last_throttle[i] + rising_limit
        elif dth < -falling_limit:
            next_throttle[i] = last_throttle[i] - falling_limit
        else:
            next_throttle[i] = new_throttle[i]

    return next_throttle
