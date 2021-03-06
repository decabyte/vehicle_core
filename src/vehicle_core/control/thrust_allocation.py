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
np.set_printoptions(precision=3, suppress=True)

# TODO: remove this dependency and make all the tc.XXX an input parameter that the pilot will pass and add pythran optimization
from vehicle_core.config import thrusters_config as tc


def tam_weighted_inverse(Ta, efficiency):
    """Weighted pseudo-inverse (generalized version) - Fossen et al.

    This function implements:
        Tw = W^-1 Tat (Ta W^-1 Tat)^-1

    :param Ta: thruster allocation matrix (as designed for the vehicle)
    :param efficiency: array of thrusters efficiency (MUST be greater or equal to 0, SHOULD be 1 in normal conditions)
    :return: Tw weighted pseudo-inverse of thruster allocation matrix Ta
    """

    W_inv = np.diag(efficiency)

    Tw = np.dot(
        W_inv,
        np.dot(
            Ta.T,
            np.linalg.pinv(
                np.dot(
                    np.dot(Ta, W_inv),
                    Ta.T
                )
            )
        )
    )

    return Tw


def tam_inverse(Ta):
    """Return the inverse of the given thruster allocation matrix (Ta).

    It is using the standard numpy pinv method (as of 1.8.x release based on SVD).

    http://matwbn.icm.edu.pl/ksiazki/amc/amc14/amc1443.pdf

    :param Ta: thruster allocation matrix (as designed for the vehicle)
    :return: Ta_inv inverse of Ta using the standard numpy approach, based on SVD decomposition.
    """

    return np.linalg.pinv(Ta)


def saturation_allocation(tau_request, inv_TAM):
    """This function prevents saturation of thrusters by scaling down their thrust request.

    The algorithm scales down the thrusts if any of any DOFs crosses the maximum available thruster's force.
    The x and y axis are coupled while z is considered independent.

    :param forces:
    :return: forces_sat
    """
    forces = np.dot(inv_TAM, tau_request)

    if np.any(forces[0:4] > tc.MAX_FORCE):
        forces[0:4] = (forces[0:4] / np.max(forces[0:4])) * tc.MAX_FORCE

    if np.any(forces[4:6] > tc.MAX_FORCE):
        forces[4:6] = (forces[4:6] / np.max(forces[4:6])) * tc.MAX_FORCE

    return forces


def priority_allocation(tau_request, inv_TAM, priorities=(2, 5, 1, 0, 4, 3)):
    """This functions assures that priority among degrees of freedom is respected.

    Default priorities:
        heave > yaw > sway > surge > pitch > roll

    :param tau_request: the body-frame forces to allocate using the current configuration
    :param inv_TAM: the inverse of the thruster allocation matrix, used for allocating the forces
    :param priorities: a python iterable of dof indexes representing the allocation priorities
    :return: forces_assigned: ndarray of (N,) forces where N is the number of thrusters
    """
    thrust_available = np.ones(inv_TAM.shape[0]) * tc.MAX_FORCE
    forces_assigned = np.zeros_like(tau_request)

    for dof in priorities:
        local_tau = np.zeros_like(tau_request)
        local_tau[dof] = tau_request[dof]

        local_request = np.dot(inv_TAM, local_tau)
        thrust_request = np.abs(local_request)

        idx_zeros = np.argwhere(thrust_available == 0)
        idx_avail = np.argwhere(thrust_available != 0)

        if np.any(thrust_request[idx_zeros] != 0):
            continue

        metric = thrust_request[idx_avail] / thrust_available[idx_avail]

        if np.any(metric > 1):
            thrust_request = thrust_request / np.max(metric)

        thrust_available -= thrust_request
        forces_assigned = forces_assigned + (np.sign(local_request) * thrust_request)

        # print('idx_avail: %s' % idx_avail.flatten())
        # print('metric: %s' % metric.flatten())
        # print('thrust_request: %s' % thrust_request)
        # print('thrust_available: %s' % thrust_available)
        # print('forces_assigned: %s\n' % forces_assigned)

    return forces_assigned


def evaluate_max_force(inv_TAM):
    """Finds a maximum absolute force for each axis given an inverse thruster allocation matrix.

    :param inv_TAM:
    :return: numpy array of shape (6) with max absolute force for each axis
    """
    # find the thruster which takes most of the load
    max_contribution = np.max(np.abs(inv_TAM), axis=0)
    idx_nonzero = np.argwhere(max_contribution != 0)

    # for what value of force requested will it saturate?
    max_u = np.zeros_like(tc.MAX_U)
    max_u[idx_nonzero] = tc.MAX_FORCE / max_contribution[idx_nonzero]

    # avoid NaN and Inf
    max_u[np.where(np.isinf(max_u))] = 0
    max_u[np.where(np.isnan(max_u))] = 0

    return max_u
