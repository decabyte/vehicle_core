#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
import yaml
import numpy as np

from vehicle_core.control import vehicle_controller as vc


np.set_printoptions(precision=3, suppress=True)

# fix imports
sys.path.append('../src')

# config
PILOT_CONF = '../launch/pid_sim.yaml'


def main():

    config = {}
    dt = 0.1

    # load config
    try:
        with open(PILOT_CONF, 'r') as conf:
            pilot_config = yaml.load(conf.read())
            config = pilot_config['pilot']['controller']
    except:
        print('Invalid config!')
        sys.exit(-1)

    ctrl = vc.CascadedController(dt, config)
    ctrl.des_pos = np.array([5, 0, 0, 0, 0, 0])
    ctrl.des_vel = np.array([2, 2, 2, 0, 0, 0])
    ctrl.ctrl_mode = vc.MODE_POSITION

    cur_pos = np.zeros(6)
    cur_vel = np.zeros(6)

    try:
        from pycallgraph import PyCallGraph
        from pycallgraph.output import GraphvizOutput

        with PyCallGraph(output=GraphvizOutput()):
            u_ctrl = ctrl.update(cur_pos, cur_vel)
            print(u_ctrl)
    except ImportError:
        pass

    import cProfile
    cProfile.runctx('ctrl.update(cur_pos, cur_vel)', globals(), locals())

if __name__ == '__main__':
    main()