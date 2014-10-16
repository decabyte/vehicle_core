#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import time
import numpy as np

from vehicle_core.util import trajectory_tools as tt


np.set_printoptions(precision=3, suppress=True)

import scipy as sci
import scipy.interpolate


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

FAST_SPEED = 1.5        # m/s
FAST_DISTANCE = 30.0    # meters
FAST_ADJUST = 1.0       # seconds
FAST_TOLERANCES = 3     # meters

FL_LOOK_AHEAD = 6          # meters
FL_LOOK_AHEAD_MIN = 3      # meters
FL_TOLERANCES = 5      # meters



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
        self.points[:, 3:6] = tt.wrap_angle(self.points[:, 3:6])

        # calculate distances
        self.cum_distances = tt.cumulative_distance(self.points, spacing_dim=3)
        self.total_distance = self.cum_distances[-1]

        # NOTE: position tolerances should be passed as a numpy array (or single scalar)
        #   thus it should be parsed by upper modules and not provided directly from the ROS messages/services as strings
        self.tolerances = kwargs.get('tolerances', DEFAULT_TOLERANCES)

        # path status
        self.path_completed = False


    def distance_completed(self, position):
        """Calculate the distance along the trajectory covered so far. The distance between the last point (A)
        and current position (B) is calculated by projecting the displacement vector (A-B) on vector representing
        the distance between last point (A) and target point (C). The maximum of this distance is |AC|. Then
        distance from the start of the path to point A is added.

        :param position: numpy array of shape (6)
        :return: float - distance in meters
        """
        current_wp = self.points[self.cnt]
        prev_wp = self.points[self.cnt-1]

        vehicle_direction = (position[0:3] - prev_wp[0:3])
        trajectory_direction = (current_wp[0:3] - prev_wp[0:3])

        if np.linalg.norm(trajectory_direction) != 0:
            added_distance = np.dot(vehicle_direction, trajectory_direction) / np.linalg.norm(trajectory_direction)
        else:
            added_distance = 0

        return self.cum_distances[self.cnt - 1] + added_distance

    def distance_left(self):
        return self.cum_distances[-1] - self.distance_completed()

    def calculate_position_error(self, current_position, desired_position):
        error = current_position - desired_position
        error[3:6] = tt.wrap_angle(error[3:6])
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
                print('WP reached, moving to the next one')
            else:
                self.path_completed = True
                self.des_pos = self.points[-1]
                print('Path completed')



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
        #   - set a threshold distance before reducing the time (protects from runaway)
        #   - adjust the time if vehicle is far away from the point
        self.target_speed = float(kwargs.get('target_speed', FAST_SPEED))
        self.thrs_dist = float(kwargs.get('distance_threshold', FAST_DISTANCE))
        self.time_adjust = float(kwargs.get('time_adjust', FAST_ADJUST))
        self.kind = kwargs.get('interpolation_method', 'linear')

        # use dedicated tolerances
        self.tolerances = FAST_TOLERANCES

        # timing
        # TODO: provide a local time axis from the path controller instead of using wall-clocks! bug with path pause!
        self.t_prev = time.time()       # wallclock when calling update()
        self.t_real = time.time()       # wallclock of previous update() call

        # time at the end of each leg
        self.t_interp = self.cum_distances / self.target_speed

        # trajectory time
        self.t = 0.0
        self.t_start = 0.0
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


    # TODO: check the effect of speed parameter on this mode!
    def update(self, position, velocity):
        # update trajectory time
        self.t_real = time.time()               # current wall-clock
        self.t += self.t_real - self.t_prev     # advance time with calculated delta
        self.t_prev = self.t_real               # save wall-clock

        # do not allow the timer to move to the previous point
        self.t = np.maximum(self.t, self.t_interp[self.cnt - 1])

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
                print('Path completed')

        # if vehicle is too far from the point stop timer
        if tt.distance_between(position, self.des_pos, spacing_dim=3) > self.thrs_dist:
            self.t -= self.time_adjust

        # update waypoint counter
        try:
            # do not allow the counter to move back
            self.cnt = np.maximum(np.argwhere(self.t_interp > self.t)[0], self.cnt)
        except:
            self.cnt = len(self.points) - 1





class FastLineStrategy(PathStrategy):
    """FastLineStrategy is trying to achieve the same results for FastTimeStrategy using the concept of distance travelled
    along the requested trajectory instead of use time to move the requested point in front of the vehicle.

    WARNING: This may be removed in the future and it use is deprecated.
    """

    def __init__(self, points, position, **kwargs):
        super(FastLineStrategy, self).__init__(points, position, **kwargs)

        self.position_interpolation = [
            sci.interpolate.interp1d(self.cum_distances, self.points[:, 0], kind='linear'),
            sci.interpolate.interp1d(self.cum_distances, self.points[:, 1], kind='linear'),
            sci.interpolate.interp1d(self.cum_distances, self.points[:, 2], kind='linear'),
            sci.interpolate.interp1d(self.cum_distances, self.points[:, 3], kind='linear'),
            sci.interpolate.interp1d(self.cum_distances, self.points[:, 4], kind='linear'),
            sci.interpolate.interp1d(self.cum_distances, self.points[:, 5], kind='linear')
        ]

        self.look_ahead = kwargs.get('look_ahead', FL_LOOK_AHEAD)
        self.look_ahead_requested = self.look_ahead
        self.tolerances = FL_TOLERANCES


    def generate_position_ahead(self, position, distance):
        position_ahead = np.zeros(6)
        distance = np.clip(distance, 0, self.cum_distances[-1])
        # if distance ahead is greater than the length of the path go to the end of the path

        position_ahead[0] = self.position_interpolation[0](distance)
        position_ahead[1] = self.position_interpolation[1](distance)
        position_ahead[2] = self.position_interpolation[2](distance)
        position_ahead[3:5] = 0
        position_ahead[5] = tt.calculate_orientation(position, position_ahead)

        return position_ahead

    def update(self, position, velocity):

        if self.cnt >= len(self.points):
            self.des_pos = self.points[-1]
            return

        # calculate the position error towards current waypoint
        previous_error = self.calculate_position_error(position, self.des_pos)

        distance = self.distance_completed(position)
        distance_ahead = distance + self.look_ahead
        distance_control = distance + self.look_ahead / 2.0

        # update requested position using the look-ahead parameter
        self.des_pos = self.generate_position_ahead(position, distance_ahead)

        if distance_control >= self.cum_distances[self.cnt]:
            self.cnt += 1

            # adaptive look-ahead (prevent drifting by gradually reducing the look-ahead)
            if not np.all(np.abs(previous_error) < self.tolerances):
                self.look_ahead -= 1
            else:
                self.look_ahead += 1

            #print('updated look-ahead: %s' % self.look_ahead)
            #print('last error: %s' % np.abs(previous_error))
            self.look_ahead = np.clip(self.look_ahead, FL_LOOK_AHEAD_MIN, self.look_ahead_requested)

            # check for end of path
            if self.cnt < len(self.points):
                #print('WP reached')
                pass
            else:
                self.path_completed = True
                self.des_pos = self.points[-1]
                #print('Path completed')




############### DEMO #############################
def plot_run(points, requested_points, vehicle_points, animate=False):
    # plots
    fig, ax = tt.plot_trajectory(points)
    tt.plot_trajectory(requested_points, fig=fig, ax=ax, p_style='b', show_orientation=False)
    tt.plot_trajectory(vehicle_points, fig=fig, ax=ax, p_style='g')

    # add vehicle
    patch = plt.Rectangle((0, 0), .6, 1.3)

    if animate:
        def init():
            patch.set_x(10000)
            patch.set_y(10000)
            ax.add_patch(patch)
            return patch,

        def animate(t):
            x = vehicle_points[t, 1]
            y = vehicle_points[t, 0]
            angle = vehicle_points[t, 5]
            patch.set_x(x - 0.3*np.cos(angle) - 0.65*np.sin(angle))
            patch.set_y(y - 0.65*np.cos(angle) + 0.3*np.sin(angle))
            patch._angle = -np.rad2deg(angle)
            return patch,

        anim = animation.FuncAnimation(
            fig, animate, init_func=init, frames=len(vehicle_points),
            interval=70, blit=True
        )

    plt.show()


def demonstrate_modes(points, mode, iterations, animate=False, step=0.3):
    if mode == 'simple':
        path_strategist = SimpleStrategy(points[1:], position=points[0])
    elif mode == 'lines':
        path_strategist = LineStrategy(points[1:], position=points[0], spacing=1)
    elif mode == 'fast':
        path_strategist = FastLineStrategy(points[1:], position=points[0], look_ahead=5)
    else:
        return

    requested_points = np.zeros((iterations-1, 6))
    vehicle_points = np.zeros((iterations, 6))
    vehicle_points[0] = points[0]

    for i in xrange(1, iterations):
        path_strategist.update(vehicle_points[i-1], [])
        requested_points[i-1] = path_strategist.des_pos
        vehicle_points[i] = move_vehicle(vehicle_points[i-1], path_strategist.des_pos, step)

    plot_run(points, requested_points, vehicle_points, animate)


def move_vehicle(current_position, desired_position, step):
    direction = desired_position[0:3] - current_position[0:3]
    modulus = np.linalg.norm((desired_position[0:3] - current_position[0:3]))

    if modulus > 0:
        direction_norm = direction/modulus
    else:
        direction_norm = 0

    next_position = np.zeros(6)
    next_position[0:3] = current_position[0:3] + step * direction_norm
    next_position[3:6] = desired_position[3:6]

    return next_position


if __name__ == '__main__':
    from matplotlib import animation
    import matplotlib.pyplot as plt
    center = np.array([0, 0, 0, 0, 0, 0])

    # points = sp.circular_pattern(center, radius=50.0)
    # points = sp.square_pattern(center, distance=30.0)
    # points = sp.nested_square_pattern(center, ext_dist=30.0, int_dist=10.0, spacing=5.0)
    points = sp.spiral_pattern(center, distance=15.0, spacing=5)

    # points = np.array([[0, 0, 0, 0, 0, 3.14],
    #                    [150, 5, 0, 0, 0, 0],
    #                    [0, 10, 0, 0, 0, 0],
    #                    [10, 25, 0, 0, 0, 0]])

    # traj = tt.interpolate_trajectory(points, 50, face_goal=False)
    # tt.plot_trajectory(traj)
    demonstrate_modes(points, 'fast', iterations=2000, animate=True)

    A = np.array([10, -10, 0, 0, 0, 0])
    B = np.array([-20, 10, 0, 0, 0, 0])

    points = np.array([
        [0, 0, 0, 0, 0, 0],
        [5, -15, 0, 0, 0, 0],
        [-5, 20, 0, 0, 0, 0],
        [0, 10, 0, 0, 0, 0]
    ])

    # traj = tt.interpolate_arc(A, B, radius=1, spacing=1, right=True)
    # tt.plot_trajectory(traj)

    # traj = tt.interpolate_bezier_cubic(points)
    # tt.plot_trajectory(traj)

    # traj = tt.interpolate_bezier_general(points)
    # tt.plot_trajectory(traj)
    #
    # plt.show()
