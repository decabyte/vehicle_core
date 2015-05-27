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

import scipy as sci
import scipy.interpolate

from vehicle_core.path import trajectory_tools as tt
from vehicle_core.util import conversions as cnv


# default config
DEFAULT_TOLERANCES = np.array([
    0.5,                # meters
    0.5,                # meters
    0.5,                # meters
    np.deg2rad(45),     # radians
    np.deg2rad(45),     # radians
    np.deg2rad(6)       # radians
])

LINE_SPACING = 10       # meters

FAST_SPEED = 1.0        # m/s
FAST_TOLERANCES = 3.0   # meters
FAST_LOOKAHEAD = 5.0    # meters


class PathStrategy(object):
    """PathStrategy is the base class for any path navigation strategies. It implements all the common features required
    by other modes, like initialization of waypoint lists, total and cumulative distances, tolerances and status.

    This class shouldn't be used directly as it doesn't provide any form of navigation capability.
    """

    def __init__(self, points, position, **kwargs):
        # internal status
        self.cnt = 1                    # start always from one (default to first requested waypoint)
        self.des_pos = position         # initialize with current position

        # keep track of trajectory
        #   - add the current waypoint (last known position)
        #   - (optimize) remove possible duplicates
        #   - (safety) wrap angle
        self.points = np.concatenate((position.reshape(1, 6), points), axis=0)
        self.points = tt.remove_repeated_points(self.points)
        self.points[:, 3:6] = cnv.wrap_pi(self.points[:, 3:6])

        # calculate distances
        self.cum_distances = tt.cumulative_distance(self.points, spacing_dim=3)
        self.total_distance = self.cum_distances[-1]

        # NOTE: position tolerances should be passed as a numpy array (or single scalar)
        #   thus it should be parsed by upper modules and not provided directly from the ROS messages/services as strings
        self.tolerances = kwargs.get('tolerances', DEFAULT_TOLERANCES)

        # path status
        self.path_completed = False


    def distance_left(self, position=None):
        return self.cum_distances[-1] - self.distance_completed(position)


    def distance_completed(self, position=None):
        """Calculate the distance along the trajectory covered so far. The distance between the last point (A)
        and current position (B) is calculated by projecting the displacement vector (A-B) on vector representing
        the distance between last point (A) and target point (C). The maximum of this distance is |AC|. Then
        distance from the start of the path to point A is added.

        :param position: numpy array of shape (6)
        :return: float - distance in meters
        """
        current_wp = self.points[self.cnt - 1]
        prev_wp = self.points[self.cnt - 2]
        added_distance = 0

        if position is not None:
            vehicle_direction = (position[0:3] - prev_wp[0:3])
            trajectory_direction = (current_wp[0:3] - prev_wp[0:3])

            if np.linalg.norm(trajectory_direction) != 0:
                added_distance = np.dot(vehicle_direction, trajectory_direction) / np.linalg.norm(trajectory_direction)

        return self.cum_distances[self.cnt - 2] + added_distance


    def calculate_position_error(self, current_position, desired_position):
        error = current_position - desired_position
        error[3:6] = cnv.wrap_pi(error[3:6])
        return error

    def update(self, position, velocity):
        pass

    def __str__(self):
        pass


class SimpleStrategy(PathStrategy):
    """SimpleStrategy provides the basic navigation functionality where the vehicle is requested to adjust its attitude as soon
    as the following waypoint is requested. If these are close enough this mode can help in maintaining sensor pointing.

    This mode is useful for its precise positioning. However it lacks efficiency thus its use is limited to precise manoeuvring
    during inspection tasks or short position changes.
    """

    def __init__(self, points, position, **kwargs):
        super(SimpleStrategy, self).__init__(points, position, **kwargs)

    def update(self, position, velocity):
        if self.cnt >= len(self.points):
            self.des_pos = self.points[-1]
            return

        self.des_pos = self.points[self.cnt]
        self.error_position = self.calculate_position_error(position, self.des_pos)

        if np.all(np.abs(self.error_position) < self.tolerances):
            self.cnt += 1

            if self.cnt < len(self.points):
                #print 'WP reached, moving to the next one'
                pass
            else:
                self.path_completed = True
                self.des_pos = self.points[-1]



class LineStrategy(PathStrategy):
    """LineStrategy provides a line-of-sight navigation mode where the vehicle visit all the path waypoint achieving the
    requested attitude (RPY) but travelling using forward navigation among them, thus implementing a rotate-first approach.

    This mode is useful for its good trade-off between precise positioning and efficient navigation due to
    forward navigation. It make use of the concept of waypoint proximity to adjust the requested attitude when close to
    each requested waypoint.
    """

    def __init__(self, points, position, **kwargs):
        super(LineStrategy, self).__init__(points, position, **kwargs)

        # config
        self.spacing = float(kwargs.get('spacing', LINE_SPACING))
        self.points = tt.interpolate_trajectory(self.points, self.spacing, face_goal=True)
        self.proximity = False

        # calculate distances (after interpolating the points)
        self.cum_distances = tt.cumulative_distance(self.points, spacing_dim=3)
        self.total_distance = self.cum_distances[-1]


    def update(self, position, velocity):
        if self.cnt >= len(self.points):
            self.des_pos = self.points[-1]
            return

        # select next waypoint
        self.des_pos = self.points[self.cnt]

        # calculate the error using the point in the trajectory list
        error_position = self.calculate_position_error(position, self.des_pos)


        # check if proximity condition (within two tolerances) is reached and latched:
        #   once proximity is triggered the vehicle will adjust for the final yaw
        #   until reaching the requested waypoint (this takes into account disturbances)
        if np.all(np.abs(error_position[0:2]) < 2 * self.tolerances[0:2]):
            self.proximity = True

        # if waypoint is far adjust vehicle orientation towards the next waypoint
        if not self.proximity:
            self.des_pos[5] = tt.calculate_orientation(position, self.des_pos)


        # if waypoint is reached also reset proximity flag (re-enable yaw adjustments)
        if np.all(np.abs(error_position) < self.tolerances):
            self.cnt += 1
            self.proximity = False

            if self.cnt < len(self.points):
                #print('WP reached, moving to the next one')
                pass
            else:
                self.path_completed = True
                self.des_pos = self.points[-1]
                #print('Path completed')



class FastTimeStrategy(PathStrategy):
    """FastTimeStrategy provides a rapid navigation mode by interpolating the trajectory between requested waypoint.

    This mode use the notion of time along the trajectory to request a position further ahead in the path in order to maintain
    the requested target speed. Its behaviour can be tuned using the constructor's keyword arguments like:

        - target_speed: the ideal navigation speed, waypoint requests will try to meet this requirement

    If the vehicle is to far from the requested waypoint some external effects, like real sea effects, disturbances or
    user input (cooperative control), the internal navigation timer can drift away and a compensation is needed. This effect
    is governed by two parameters:

        - distance_threshold: maximum distance between current position and requested position before adjusting the time
        - time_adjust: seconds to adjust in the navigation timer to slow give the vehicle more time to complete the current leg

    """

    def __init__(self, points, position, **kwargs):
        super(FastTimeStrategy, self).__init__(points, position, **kwargs)

        # mode config:
        #   - set a target speed for the path navigation
        #   - adjust the time if vehicle is far away from the point
        self.look_ahead = float(kwargs.get('look_ahead', FAST_LOOKAHEAD))
        self.target_speed = float(kwargs.get('target_speed', FAST_SPEED))
        self.kind = kwargs.get('interpolation_method', 'linear')

        # use dedicated tolerances
        self.tolerances = FAST_TOLERANCES

        # trajectory time
        self.t = 0.0
        self.t_interp = self.cum_distances / self.target_speed      # time at the end of each leg
        self.t_end = self.t_interp[-1]

        # interpolating assuming constant speed
        self.fc = [
            sci.interpolate.interp1d(self.t_interp, self.points[:, 0], kind=self.kind),
            sci.interpolate.interp1d(self.t_interp, self.points[:, 1], kind=self.kind),
            sci.interpolate.interp1d(self.t_interp, self.points[:, 2], kind=self.kind),
            sci.interpolate.interp1d(self.t_interp, self.points[:, 3], kind=self.kind),
            sci.interpolate.interp1d(self.t_interp, self.points[:, 4], kind=self.kind),
            sci.interpolate.interp1d(self.t_interp, self.points[:, 5], kind=self.kind)
        ]


    def update(self, position, velocity):
        curr_error = self.calculate_position_error(position, self.des_pos)

        # close enough to next point?
        if np.linalg.norm(curr_error[0:3]) < self.look_ahead:
            # compute next time for the interpolation
            self.t += self.look_ahead / self.target_speed

        if self.t < self.t_end:
            self.des_pos[0] = self.fc[0](self.t)
            self.des_pos[1] = self.fc[1](self.t)
            self.des_pos[2] = self.fc[2](self.t)
            self.des_pos[3] = self.fc[3](self.t)
            self.des_pos[4] = self.fc[4](self.t)
            self.des_pos[5] = self.fc[5](self.t)

            # adjust yaw towards next waypoint
            self.des_pos[5] = tt.calculate_orientation(position, self.des_pos)
        else:
            self.des_pos = self.points[-1]
            self.des_pos[5] = tt.calculate_orientation(position, self.des_pos)

            waypoint_error = self.calculate_position_error(position, self.points[self.cnt, :])

            if np.all(np.abs(waypoint_error) < self.tolerances):
                self.path_completed = True
                self.des_pos = self.points[-1]
                #print('Path completed')

        # update waypoint counter
        try:
            # do not allow the counter to move back
            self.cnt = np.maximum(np.argwhere(self.t_interp > self.t)[0], self.cnt)
        except:
            self.cnt = len(self.points) - 1
