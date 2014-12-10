#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


from vehicle_core.util import trajectory_tools as tt

def main():
    # parameters
    centre = np.array([1.0, -1.0, 1.0, 0.0, 0.0, 0.0])
    radius = 1.0
    loops = 3.0
    spacing = 0.5

    # logic
    #points = tt.interpolate_circle(centre, radius, spacing=0.75, facing_centre=False)
    points = tt.interpolate_helix(centre, radius, loops=loops, spacing=spacing, facing_centre=False)

    # output
    import yaml
    print(yaml.dump(tt.traj_as_dict(points)))

    # plots
    #tt.plot_trajectory(points)

    # plots
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(points[:, 1], points[:, 0], -points[:, 2], 'or--')
    plt.show()

if __name__ == '__main__':
    main()
