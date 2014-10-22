#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
import scipy as sci
import scipy.signal


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

    # apply latency delay
    throttle_delayed = throttle_request[:, 0:-(1 - offset)]
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
