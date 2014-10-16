#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

from numpy import cos, sin, tan

# Vehicle Parameters
mass = 60.0         # mass (kg)     was 55.0
radius = 0.150      # radius (m)
length = 1.700      # length (m)
rho = 1025.0        # density (salt water) (kg/m^3)
volume = 0.12       # volume of the robot (m^3)
W = mass * 9.81     # weight (N)
B = W + 1           # buoyancy (N)

# NOTE:
#   the buoyancy term should be carefully adjusted cause is volatile and 
#   it is dependent on the actual trim of the robot for this reason it can be
#   assumed to be slightly different than the robot weight and leave extra corrections
#   to the gains inside the controller subsystem
#   
#   it can be computed using formula just for simulation purposes:
#       B = rho * volume * 9.81    # buoyancy (N)
#   
#   an early experiment measured this value to be 546.1106 but again it was depending on that
#   specific setup and trimming
#
#   COG and COB follow a similar approach, in this case they can be assumed to be in the same
#   geometrical location to avoid generating higher forces on pitch term that can disturb the 
#   behaviour of the controller at runtime. This terms can be adjusted for simulation purposes.

# center of gravity [m]
xg = 0.0
yg = 0.0
zg = 0.001

# center of buoyancy [m]
xb = 0.0
yb = 0.0
zb = 0.0

# inertia tensor wrt origin of vehicle [mass * length^2]
ix = mass * (radius ** 2) * (1.0 / 2.0)
iy = mass * (3 * (radius ** 2) + (length ** 2)) * (1.0 / 12.0)
iz = mass * (3 * (radius ** 2) + (length ** 2)) * (1.0 / 12.0)

# quadratic drag
x_uu = -31.8086
y_vv = -222.8960
z_ww = -263.4225
k_pp = -0.000       # was 0 (not measured during previous experiments)
m_qq = -40.5265     # z_ww / 2.0
n_rr = -40.5265     # y_vv / 2.0

# calculate added terms based on cylindrical shape
xu_dot = -0.1 * mass
yv_dot = -np.pi * rho * (radius ** 2) * length
zw_dot = -np.pi * rho * (radius ** 2) * length
kp_dot = -np.pi * rho * (radius ** 4) * (1.0 / 4.0)
mq_dot = -np.pi * rho * (radius ** 2) * (length ** 3) * (1.0 / 12.0)
nr_dot = -np.pi * rho * (radius ** 2) * (length ** 3) * (1.0 / 12.0)

# rigid body inertia
MRB = np.array([
    [mass, 0.0, 0.0, 0.0, mass * zg, -mass * yg],
    [0.0, mass, 0.0, -mass * zg, 0.0, mass * xg],
    [0.0, 0.0, mass, mass * yg, -mass * xg, 0.0],
    [0.0, -mass * zg, mass * yg, ix, 0.0, 0.0],
    [mass * zg, 0.0, -mass * xg, 0.0, iy, 0.0],
    [-mass * yg, mass * xg, 0.0, 0.0, 0.0, iz]
])

# added mass
MA = np.array([
    [xu_dot, 0, 0, 0, 0, 0],
    [0, yv_dot, 0, 0, 0, 0],
    [0, 0, zw_dot, 0, 0, 0],
    [0, 0, 0, kp_dot, 0, 0],
    [0, 0, 0, 0, mq_dot, 0],
    [0, 0, 0, 0, 0, nr_dot]
])

# dynamic equation mass matrix and inverse matrix
M = MRB - MA
inv_M = np.linalg.inv(M)



class DynamicModel(object):


    def __init__(self, **kwargs):

        # input forces and moments
        self.tau = np.zeros(6)      # tau = [X Y Z K M N]

        # init matrices and vectors
        self.C = np.zeros((6, 6))   # coriolis effects
        self.CRB = np.zeros((6, 6)) #  rigid body
        self.CA = np.zeros((6, 6))  #  added mass

        self.D = np.zeros((6, 6))   # damping (drag)
        self.G = np.zeros(6)        # restoring forces (gravity + buoyancy)

        # forces
        self.F_c = np.zeros(6)      # coriolis forces
        self.F_d = np.zeros(6)      # damping forces
        self.F_dist = np.zeros(6)   # external disturbances

        self.F_hydro = np.zeros(6)  # total hydrodynamic forces acting on vehicle
        self.F_net = np.zeros(6)    # total result forces acting on vehicle

        # config
        self.dynamic_buoyancy = bool(kwargs.get('dynamic_buoyancy', False))
        self.coriolis_added = bool(kwargs.get('coriolis_added', False))
        self.compensate_mass = bool(kwargs.get('compensate_mass', False))



    def update_hydro(self, position, velocity):
        # unpack position and velocity
        x, y, z, phi, theta, psi = position
        u, v, w, p, q, r = velocity

        # coriolis rigid body
        self.CRB[0, 3] = mass * (yg * q + zg * r)
        self.CRB[0, 4] = -mass * (xg * q - w)
        self.CRB[0, 5] = -mass * (xg * r + v)

        self.CRB[1, 3] = -mass * (yg * p + w)
        self.CRB[1, 4] = mass * (zg * r + xg * p)
        self.CRB[1, 5] = -mass * (yg * r - u)

        self.CRB[2, 3] = -mass * (zg * p - v)
        self.CRB[2, 4] = -mass * (zg * q + u)
        self.CRB[2, 5] = mass * (xg * p + yg * q)

        self.CRB[3, 0] = -mass * (yg * q + zg * r)
        self.CRB[3, 1] = mass * (yg * p + w)
        self.CRB[3, 2] = mass * (zg * p - v)
        self.CRB[3, 3] = 0
        self.CRB[3, 4] = iz * r
        self.CRB[3, 5] = -iy * q

        self.CRB[4, 0] = mass * (xg * q - w)
        self.CRB[4, 1] = -mass * (zg * r + xg * p)
        self.CRB[4, 2] = mass * (zg * q + u)
        self.CRB[4, 3] = -iz * r
        self.CRB[4, 4] = 0
        self.CRB[4, 5] = -ix * p

        self.CRB[5, 0] = mass * (xg * r + v)
        self.CRB[5, 1] = mass * (yg * r - u)
        self.CRB[5, 2] = -mass * (xg * p + yg * q)
        self.CRB[5, 3] = iy * q
        self.CRB[5, 4] = -ix * p
        self.CRB[5, 5] = 0

        # coriolis added mass
        a1 = xu_dot * u
        a2 = yv_dot * v
        a3 = zw_dot * w
        b1 = kp_dot * p
        b2 = mq_dot * q
        b3 = nr_dot * r

        self.CA = np.array([
            [0, 0, 0, 0, -a3, a2],
            [0, 0, 0, a3, 0, -a1],
            [0, 0, 0, -a2, a1, 0],
            [0, -a3, a2, 0, -b3, b2],
            [a3, 0, -a1, b3, 0, -b1],
            [-a2, a1, 0, -b2, b1, 0]
        ])

        # calculate total matrix
        if self.coriolis_added:
            self.C = self.CRB + self.CA         # to be used for a more realistic dynamic model
        else:
            self.C = self.CRB                   # to be used for a simple simulation

        # damping vector
        self.D = -np.diag([
            x_uu * np.abs(u),
            y_vv * np.abs(v),
            z_ww * np.abs(w),
            k_pp * np.abs(p),
            m_qq * np.abs(q),
            n_rr * np.abs(r),
        ])

        # restoring vector
        if self.dynamic_buoyancy:
            self.B_local = (B / 0.15) * (z + 0.15)          # reduce buoyancy when floating above the surface
            self.B_local = np.clip(self.B_local, 0, B)      # clip the linear approximation to positive values
        else:
            self.B_local = B

        self.G = np.array([
            (W - self.B_local) * sin(theta),
            -(W - self.B_local) * cos(theta) * sin(phi),
            -(W - self.B_local) * cos(theta) * cos(phi),
            -(yg * W - yb * self.B_local) * cos(theta) * cos(phi) + (zg * W - zb * self.B_local) * cos(theta) * sin(phi),
            (zg * W - zb * self.B_local) * sin(theta) + (xg * W - xb * B) * cos(theta) * cos(phi),
            -(xg * W - xb * self.B_local) * cos(theta) * sin(phi) - (yg * W - yb * self.B_local) * sin(phi),
        ])

        # calculate coriolis and dumping forces
        self.F_c = np.dot(self.C, velocity)
        self.F_d = np.dot(self.D, velocity)

        # calculate disturbances
        self.F_dist = np.zeros(6)   # 0.01 * np.random.randn(6)

        # dynamic equation (compute total force)
        self.F_hydro = self.F_c + self.F_d + self.G + self.F_dist


    def update_model(self, position, velocity):
        """
        Added mass terms (ref: Fossen - pag 35 - 2.4 Hydrodynamic Forces and Moments)

        Reference: T. Fossen - Guidance and Control of Oceanic Vehicles

        :param position:
        :param velocity:
        :return:
        """

        self.M_term = np.zeros(6)

        # if requested take into account the mass term
        if self.compensate_mass:
            u, v, w, p, q, r = velocity

            self.M_term = -np.array([
                M[2,2]*w*q - M[1,1]*v*r,
                -M[2,2]*w*p + M[0,0]*u*r,
                M[1,1]*v*p - M[0,0]*u*q,
                0,
                -M[2,2]*w*u + M[0,0]*u*w - M[5,5]*r*p + M[3,3]*p*r,
                M[1,1]*v*u - M[0,0]*u*v + M[4,4]*q*p - M[3,3]*p*q
            ])

        self.update_hydro(position, velocity)
        self.tau_model = self.M_term + self.F_c + self.F_d + self.G

        return self.tau_model


    def calculate_acceleration(self, tau, position, velocity):
        # update dynamical model
        self.update_hydro(position, velocity)

        # dynamic equation (compute total force)
        self.F_net = tau - self.F_hydro

        # calculate acceleration from forces
        return np.dot(inv_M, self.F_net)

    def __str__(self):
        pass
