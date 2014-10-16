#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
import numpy as np

from vehicle_core.model import vehicle_model as ds


np.set_printoptions(precision=3, suppress=True)

# fix imports
sys.path.append('../src')


def main():

    config = {}
    dt = 0.1
    init = np.array([0, 0, -10, 0, 0, 0])
    tau = np.zeros(6)

    sim = ds.VehicleModel(dt, init)
    sim.update_simulation(tau)

    # try:
    #     from pycallgraph import PyCallGraph
    #     from pycallgraph.output import GraphvizOutput
    #
    #     with PyCallGraph(output=GraphvizOutput()):
    #        sim.update(tau)
    # except ImportError:
    #     pass
    #
    # import cProfile
    # cProfile.runctx('sim.update(tau)', globals(), locals())

if __name__ == '__main__':
    main()