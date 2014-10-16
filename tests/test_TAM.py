#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys

from vehicle_core.control import thrust_allocation as ta


sys.path.append('../src')

import numpy as np
np.set_printoptions(precision=3, suppress=True)


def main():
    w = np.array([30, 1, 1, 1, 1, 1])
    tau = np.array([30, 0, 0, 0, 0, 0])

    tam = np.copy(tc.TAM)
    tam[:,0] = 0
    inv_tam = np.linalg.pinv(tam)

    inv_fau = ta.tam_weighted_inverse(tc.TAM, w)

    forces_std = np.dot(inv_tam, tau)
    forces_fau = np.dot(inv_fau, tau)

    forces_sat = ta.priority_allocation(tau, inv_fau)
    forces_sat_n = ta.saturation_allocation(forces_fau)

    body_std = np.dot(tc.TAM, forces_std)
    body_fau = np.dot(tc.TAM, forces_fau)
    body_fau_sat = np.dot(tc.TAM, forces_sat)
    body_fau_sat_2 = np.dot(tc.TAM, forces_sat_n)

    print('tau: %s' % tau)

    print('forces_std: %s' % forces_std)
    print('forces_fau: %s' % forces_fau)
    print('forces_sat: %s' % forces_sat)
    print('forces_sat_n: %s' % forces_sat_n)

    print('body_std: %s' % body_std)
    print('body_fau: %s' % body_fau)
    print('body_sat: %s' % body_fau_sat)
    print('body_sat_n: %s' % body_fau_sat_2)

    # print('inv_tam:\n%s' % inv_tam)
    print('inv_fau:\n%s' % inv_fau)


if __name__ == '__main__':
    main()
