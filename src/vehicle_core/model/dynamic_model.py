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

#pythran export compute_jacobian(float, float, float)
def compute_jacobian(phi, theta, psi):
    """This functions computes the jacobian matrix used for converting body-frame to earth-frame coordinates.

    :param phi: pitch angle (k)
    :param theta: roll angle (m)
    :param psi: yaw angle (n)
    :return: J matrix (6x6)
    """
    J = np.zeros((6,6))

    # jacobian one
    J[0, 0] = np.cos(theta) * np.cos(psi)
    J[0, 1] = np.cos(psi) * np.sin(theta) * np.sin(phi) - np.sin(psi) * np.cos(phi)
    J[0, 2] = np.sin(psi) * np.sin(phi) + np.cos(psi) * np.cos(phi) * np.sin(theta)

    J[1, 0] = np.cos(theta) * np.sin(psi)
    J[1, 1] = np.cos(psi) * np.cos(phi) + np.sin(phi) * np.sin(theta) * np.sin(psi)
    J[1, 2] = np.sin(psi) * np.sin(theta) * np.cos(phi) - np.cos(psi) * np.sin(phi)

    J[2, 0] = -np.sin(theta)
    J[2, 1] = np.cos(theta) * np.sin(phi)
    J[2, 2] = np.cos(theta) * np.cos(phi)

    # jacobian two
    J[3, 3] = 1.0
    J[3, 4] = np.sin(phi) * np.tan(theta)
    J[3, 5] = np.cos(phi) * np.tan(theta)

    J[4, 3] = 0.0
    J[4, 4] = np.cos(phi)
    J[4, 5] = -np.sin(phi)

    J[5, 3] = 0.0
    J[5, 4] = np.sin(phi) / np.cos(theta)
    J[5, 5] = np.cos(phi) / np.cos(theta)

    return J


#pythran export update_jacobian(float[][], float, float, float)
def update_jacobian(J, phi, theta, psi):
    """This functions computes the jacobian matrix used for converting body-frame to earth-frame coordinates.

    :param J: jacobian matrix (6x6)
    :param phi: pitch angle (k)
    :param theta: roll angle (m)
    :param psi: yaw angle (n)
    :return: J matrix (6x6)
    """
    J = np.zeros((6,6))

    # jacobian one
    J[0, 0] = np.cos(theta) * np.cos(psi)
    J[0, 1] = np.cos(psi) * np.sin(theta) * np.sin(phi) - np.sin(psi) * np.cos(phi)
    J[0, 2] = np.sin(psi) * np.sin(phi) + np.cos(psi) * np.cos(phi) * np.sin(theta)

    J[1, 0] = np.cos(theta) * np.sin(psi)
    J[1, 1] = np.cos(psi) * np.cos(phi) + np.sin(phi) * np.sin(theta) * np.sin(psi)
    J[1, 2] = np.sin(psi) * np.sin(theta) * np.cos(phi) - np.cos(psi) * np.sin(phi)

    J[2, 0] = -np.sin(theta)
    J[2, 1] = np.cos(theta) * np.sin(phi)
    J[2, 2] = np.cos(theta) * np.cos(phi)

    # jacobian two
    J[3, 3] = 1.0
    J[3, 4] = np.sin(phi) * np.tan(theta)
    J[3, 5] = np.cos(phi) * np.tan(theta)

    J[4, 3] = 0.0
    J[4, 4] = np.cos(phi)
    J[4, 5] = -np.sin(phi)

    J[5, 3] = 0.0
    J[5, 4] = np.sin(phi) / np.cos(theta)
    J[5, 5] = np.cos(phi) / np.cos(theta)

    return J


#pythran export calc_coriolis(float[], float, float[], float[], float[])
def calc_coriolis(vel, mass, cog, inertia, added_terms):
    # unpack arguments (following Fossen's conventions)
    u, v, w, p, q, r = vel
    xg, yg, zg = cog
    ix, iy, iz = inertia
    xu_dot, yv_dot, zw_dot, kp_dot, mq_dot, nr_dot = added_terms

    # coriolis rigid body
    CRB = np.zeros((6,6), dtype=np.float64)

    CRB[0, 3] = mass * (yg * q + zg * r)
    CRB[0, 4] = -mass * (xg * q - w)
    CRB[0, 5] = -mass * (xg * r + v)

    CRB[1, 3] = -mass * (yg * p + w)
    CRB[1, 4] = mass * (zg * r + xg * p)
    CRB[1, 5] = -mass * (yg * r - u)

    CRB[2, 3] = -mass * (zg * p - v)
    CRB[2, 4] = -mass * (zg * q + u)
    CRB[2, 5] = mass * (xg * p + yg * q)

    CRB[3, 0] = -mass * (yg * q + zg * r)
    CRB[3, 1] = mass * (yg * p + w)
    CRB[3, 2] = mass * (zg * p - v)
    CRB[3, 3] = 0
    CRB[3, 4] = iz * r
    CRB[3, 5] = -iy * q

    CRB[4, 0] = mass * (xg * q - w)
    CRB[4, 1] = -mass * (zg * r + xg * p)
    CRB[4, 2] = mass * (zg * q + u)
    CRB[4, 3] = -iz * r
    CRB[4, 4] = 0
    CRB[4, 5] = -ix * p

    CRB[5, 0] = mass * (xg * r + v)
    CRB[5, 1] = mass * (yg * r - u)
    CRB[5, 2] = -mass * (xg * p + yg * q)
    CRB[5, 3] = iy * q
    CRB[5, 4] = -ix * p
    CRB[5, 5] = 0

    # coriolis added mass
    a1 = xu_dot * u
    a2 = yv_dot * v
    a3 = zw_dot * w
    b1 = kp_dot * p
    b2 = mq_dot * q
    b3 = nr_dot * r

    CA = np.array([
        [0, 0, 0, 0, -a3, a2],
        [0, 0, 0, a3, 0, -a1],
        [0, 0, 0, -a2, a1, 0],
        [0, -a3, a2, 0, -b3, b2],
        [a3, 0, -a1, b3, 0, -b1],
        [-a2, a1, 0, -b2, b1, 0]
    ])

    return CRB + CA


#pythran export calc_damping(float[], float[])
def calc_damping(vel, quadratic_coeff):
    # unpack arguments (following Fossen's conventions)
    u, v, w, p, q, r = vel
    x_uu, y_vv, z_ww, k_pp, m_qq, n_rr = quadratic_coeff

    return -np.diag([
        x_uu * np.abs(u),
        y_vv * np.abs(v),
        z_ww * np.abs(w),
        k_pp * np.abs(p),
        m_qq * np.abs(q),
        n_rr * np.abs(r),
    ])


#pythran export calc_restoring(float[], float[], float[], float, float)
def calc_restoring(pos, cog, cob, W, B):
    # unpack arguments (following Fossen's conventions)
    x, y, z, phi, theta, psi = pos
    xg, yg, zg = cog
    xb, yb, zb = cob

    return np.array([
        (W - B) * np.sin(theta),
        -(W - B) * np.cos(theta) * np.sin(phi),
        -(W - B) * np.cos(theta) * np.cos(phi),
        -(yg * W - yb * B) * np.cos(theta) * np.cos(phi) + (zg * W - zb * B) * np.cos(theta) * np.sin(phi),
        (zg * W - zb * B) * np.sin(theta) + (xg * W - xb * B) * np.cos(theta) * np.cos(phi),
        -(xg * W - xb * B) * np.cos(theta) * np.sin(phi) - (yg * W - yb * B) * np.sin(phi),
    ])


#pythran export calc_mass_term(float[], float[][])
def calc_mass_term(vel, mass_matrix):
    # unpack arguments (following Fossen's conventions)
    u, v, w, p, q, r = vel
    M = mass_matrix

    return -np.array([
        M[2,2]*w*q - M[1,1]*v*r,
        -M[2,2]*w*p + M[0,0]*u*r,
        M[1,1]*v*p - M[0,0]*u*q,
        0,
        -M[2,2]*w*u + M[0,0]*u*w - M[5,5]*r*p + M[3,3]*p*r,
        M[1,1]*v*u - M[0,0]*u*v + M[4,4]*q*p - M[3,3]*p*q
    ])


#pythran export calc_model_forward(float[], float[], float[], float[], float, float[], float, float, float[][], float[], float[])
def calc_model_forward(pos, vel, cog, cob, mass, inertia, W, B, mass_matrix, added_terms, quadratic_coeff):
    M = calc_mass_term(vel, mass_matrix)
    C = calc_coriolis(vel, mass, cog, inertia, added_terms)
    D = calc_damping(vel, quadratic_coeff)
    G = calc_restoring(pos, cog, cob, W, B)

    F_c = np.dot(C, vel.reshape((6,1))).flatten()
    F_d = np.dot(D, vel.reshape((6,1))).flatten()

    # F_c = mat_vec_mult(C, vel.flatten())
    # F_d = mat_vec_mult(D, vel.flatten())

    return M + F_c + F_d + G


#pythran export calc_other_forces(float[], float[], float[], float[], float, float[], float, float, float[], float[])
def calc_other_forces(pos, vel, cog, cob, mass, inertia, W, B, added_terms, quadratic_coeff):
    C = calc_coriolis(vel, mass, cog, inertia, added_terms)
    D = calc_damping(vel, quadratic_coeff)
    G = calc_restoring(pos, cog, cob, W, B)

    F_c = np.dot(C, vel.reshape((6,1))).flatten()
    F_d = np.dot(D, vel.reshape((6,1))).flatten()

    # F_c = mat_vec_mult(C, vel.flatten())
    # F_d = mat_vec_mult(D, vel.flatten())

    return F_c + F_d + G


#pythran export calc_coupled_forces(float[], float[], float[], float[], float[], float, float[], float, float, float[], float[])
def calc_coupled_forces(pos, vel, des_vel, cog, cob, mass, inertia, W, B, added_terms, quadratic_coeff):
    C = calc_coriolis(vel, mass, cog, inertia, added_terms)
    D = calc_damping(des_vel, quadratic_coeff)
    G = calc_restoring(pos, cog, cob, W, B)

    F_c = np.dot(C, vel.reshape((6,1))).flatten()
    F_d = np.dot(D, vel.reshape((6,1))).flatten()

    #F_c = mat_vec_mult(C, vel.flatten())
    #F_d = mat_vec_mult(D, des_vel.flatten())
    #active = np.array([0.5, 1.0, 0.2, 0.0, 0.01, 0.01])

    return  F_c + F_d + G  # active*F_c + F_d + G


#pythran export mat_vec_mult(float[][], float[])
def mat_vec_mult(A, b):
    """This function implements the matrix vector multiplication.

    This is needed because of version 0.6.0 Pythran doesn't implement yet the correct
    translation from numpy.dot between a 2-dim ndarray and a 1-dim one.

    :param A: input matrix 2-dim ndarray
    :param b: input vector 1-dim ndarray
    :return: c output vector 1-dim ndarray (like b) -- c = np.dot(A, b)
    """
    c = np.zeros_like(b)

    for i in xrange(A.shape[0]):
        for j in xrange(A.shape[1]):
            c[i] += A[i,j] * b[j]

    return c
