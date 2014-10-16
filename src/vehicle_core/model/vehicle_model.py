#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
NOTES:
    The buoyancy term should be carefully adjusted cause is is dependent on the actual trim of the vehicle,
    for this reason a good working value can be derived from the vehicle's weight and leaving the extra corrections
    to the controller subsystem in terms of gains and offsets.

    For simulation purposes the buoyancy it can be computed using formula:
      B = rho * volume * 9.81    # buoyancy (N)

    An early experiment measured this value to be 546.1106 for Nessie VII AUV but that value was depending on the
    specific setup and trimming related to the experimental setup used for its measurement.

    The COG and COB need a similar approach, in this case, they can be assumed to be in the same geometrical position
    in order to avoid the generation of strong pitch forces when using this model with the actual vehicle. These terms
    instead can be safely adjusted for simulation purposes.
"""
from __future__ import division

import numpy as np

from vehicle_core.model import dynamic_model as dm

np.set_printoptions(precision=3, suppress=True)


class VehicleModel(object):
    """VehicleModel class provides an easy tool to deal with dynamical forces acting on the vehicle.

    This module is separated from the ROS interface for improving the testing and debug procedures as well as allowing its
    embedding in other modules (for instance to calculate navigation estimates at run-time without launching the heavy machinery
    involved for the complete nav_sim node).

    It can be used to calculate the total net forces acting on the vehicle if provided with an input force in body-frame
    coordinates and the position and velocities of the vehicle itself. This approach is used by the nav_sim modules.

    It can be used to calculate the total hydrodynamic forces if provided with the navigation information (position and
    velocities) measured with on-board sensors. This approach is used by the vehicle_controller modules.
    """

    def __init__(self, config, **kwargs):
        self.shape = config.get('shape', 'cylindrical')

        if self.shape != 'cylindrical':
            raise NotImplemented('Could not create VehicleModel for non-cylindrical shapes.')

        self.mass = config.get('mass')          # total mass of the vehicle [kg]
        self.W = config.get('weight')           # weight [N]    (it should be mass * 9.81)
        self.B = config.get('buoyancy')         # buoyancy [N]  (it should be close to W)

        self.cog = np.array(config.get('cog'))  # center of gravity [m] (xg, yg, zg)
        self.cob = np.array(config.get('cob'))  # center of buoyancy [m]  (xb, yb, zb)

        self.radius = config.get('radius')                  # radius [m]
        self.length = config.get('length')                  # length [m]
        self.volume = config.get('volume')                  # volume of the vehicle [m^3]
        self.water_rho = config.get('water_rho', 1025.0)    # density (salt water) [kg/m^3]

        # principal quadratic drag coefficients (approximation for underwater vehicles)
        #   [x_uu, y_vv, z_ww, k_pp, m_qq, n_rr]
        self.quadratic_drag = np.array(config.get('quadratic_drag'))

        # inertia tensor wrt origin of vehicle [mass * length^2]
        self.inertia = np.array([
            self.mass * (self.radius ** 2) * (1.0 / 2.0),
            self.mass * (3 * (self.radius ** 2) + (self.length ** 2)) * (1.0 / 12.0),
            self.mass * (3 * (self.radius ** 2) + (self.length ** 2)) * (1.0 / 12.0),
        ])

        # forces
        self.F_net = np.zeros(6)        # total net force acting on the vehicle
        self.F_model = np.zeros(6)      # total hydrodynamic forces acting on the vehicle
        # self.F_C = np.zeros(6)          # coriolis forces
        # self.F_D = np.zeros(6)          # damping forces
        # self.F_G = np.zeros(6)          # restoring forces


        # calculate added terms based on cylindrical shape
        self.added_terms = np.array([
            -0.1 * self.mass,                                                                           # xu_dot
            -np.pi * self.water_rho * (self.radius ** 2) * self.length,                                 # yv_dot
            -np.pi * self.water_rho * (self.radius ** 2) * self.length,                                 # zw_dot
            -np.pi * self.water_rho * (self.radius ** 4) * (1.0 / 4.0),                                 # kp_dot
            -np.pi * self.water_rho * (self.radius ** 2) * (self.length ** 3) * (1.0 / 12.0),           # mq_dot
            -np.pi * self.water_rho * (self.radius ** 2) * (self.length ** 3) * (1.0 / 12.0)            # nr_dot
        ])

        # calculate rigid body inertia
        xg, yg, zg = self.cog
        ix, iy, iz = self.inertia

        self.MRB = np.array([
            [self.mass, 0.0, 0.0, 0.0, self.mass * zg, -self.mass * yg],
            [0.0, self.mass, 0.0, -self.mass * zg, 0.0, self.mass * xg],
            [0.0, 0.0, self.mass, self.mass * yg, -self.mass * xg, 0.0],
            [0.0, -self.mass * zg, self.mass * yg, ix, 0.0, 0.0],
            [self.mass * zg, 0.0, -self.mass * xg, 0.0, iy, 0.0],
            [-self.mass * yg, self.mass * xg, 0.0, 0.0, 0.0, iz]
        ])

        # added mass (approximation for underwater vehicles)
        #
        #     [xu_dot, 0, 0, 0, 0, 0],
        #     [0, yv_dot, 0, 0, 0, 0],
        #     [0, 0, zw_dot, 0, 0, 0],
        #     [0, 0, 0, kp_dot, 0, 0],
        #     [0, 0, 0, 0, mq_dot, 0],
        #     [0, 0, 0, 0, 0, nr_dot]
        #
        self.MA = np.diag(self.added_terms)

        # dynamic equation mass matrix and inverse matrix
        self.M = self.MRB - self.MA
        self.inv_M = np.linalg.inv(self.M)


    def update_model(self, pos, vel):
        """Updates the model's forces based on the provided position and velocity vectors.

        This functions uses if available the optimized version of the dynamic model, loading the .so file, providing
        a fast computation despite its long list of parameters. It also stores the last computed value as an object
        attribute for later inspection or verbose printing.

        :param pos: current position of the vehicle [x,y,z,k,m,n]   (in meters and radians)
        :param vel: current velocity of the vehicle [u,v,w,p,q,r]   (in m/s and rad/s)
        :return: numpy.ndarray of shape (6,) with forces acting in body-frame [X,Y,Z,K,M,N]
        """

        self.F_model = dm.calc_model_forces(
            pos, vel, self.cog, self.cob, self.mass, self.inertia, self.W, self.B,
            self.M, self.added_terms, self.quadratic_drag
        )

        return self.F_model

    def update_acceleration(self, tau, pos, vel):
        # dynamic equation (compute total force)
        self.F_net = tau - self.update_model(pos, vel)

        # calculate acceleration from forces
        return np.dot(self.inv_M, self.F_net)

    def __str__(self):
        pass


if __name__ == '__main__':
    import pprint
    import yaml

    pp = pprint.PrettyPrinter(indent=2)
    model_config = dict()

    with open('../launch/vehicle_model.yaml', 'r') as conf:
        input_config = yaml.load(conf)
        model_config.update(input_config['vehicle']['model'])

    print('Model Configuration:')
    pp.pprint(model_config)
    print('')

    # create vehicle model
    vm = VehicleModel(model_config)

    tau = np.zeros(6)
    pos = np.zeros(6)
    vel = np.zeros(6)

    # compute model acceleration (apply 10 Newtons in surge while being on the surface)
    tau[0] = 10
    acc = vm.update_acceleration(tau, pos, vel)

    print('Accelerations:')
    print(acc)
    print('')

    # compute model hydrodynamic forces (while being at given depth)
    pos[2] = 10
    forces = vm.update_model(pos, vel)

    print('Forces:')
    print(forces)
    print('')
