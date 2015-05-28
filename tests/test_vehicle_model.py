#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
import pprint
import yaml

import numpy as np
import matplotlib.pyplot as plt

# pretty
np.set_printoptions(precision=3, suppress=True)
pp = pprint.PrettyPrinter(indent=2)

# fix path
sys.path.append('../src')

from vehicle_core.model import vehicle_model as vm

def main():
    model_config = dict()

    with open('../conf/vehicle_model.yaml', 'r') as conf:
        input_config = yaml.load(conf)
        model_config.update(input_config['vehicle']['model'])

    print('Model Configuration:')
    pp.pprint(model_config)
    print('')

    # create vehicle model
    model = vm.VehicleModel(model_config)

    tau = np.zeros(6)
    pos = np.zeros(6)
    vel = np.zeros(6)
    acc = np.zeros(6)

    # test linearization
    acc[0] = 0.250
    tau = model.update_tau(pos, vel, acc)
    sim = model.update_acceleration(tau, pos, vel)

    print('Forces:')
    print(tau)
    print(sim)
    print()

    # # compute model acceleration (apply 10 Newtons in surge while being on the surface)
    # tau[0] = 10
    # acc = vm.update_acceleration(tau, pos, vel)
    #
    # print('Accelerations:')
    # print(acc)
    # print('')
    #
    # # compute model hydrodynamic forces (while being at given depth)
    # pos[2] = 10
    # forces = vm.update_forward_model(pos, vel)
    #
    # print('Forces:')
    # print(forces)
    # print('')

if __name__ == '__main__':
    main()
