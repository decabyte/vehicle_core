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

"""Trajectory Utilities Module

Tasks:
    - generate trajectories
    - produce dumps of trajectories for saving/sending (ie. JSON, CSV, ...)
"""

from __future__ import division

import datetime

import numpy as np
import scipy as sci
import matplotlib.pyplot as plt

import scipy.misc
np.set_printoptions(precision=3, suppress=True)

from vehicle_core.util import conversions as cnv


def pol2cart(r, th):
    """Convert from polar coordinates to cartesian coordinates (2D)

    :param r:
    :param th:
    :return: x, y
    """
    return np.cos(th) * r, np.sin(th) * r

def cart2pol(x, y):
    """Convert from cartesian coordinates to polar coordinates (2D)

    :param x:
    :param y:
    :return: r, th
    """
    r = np.sqrt(np.sum([x**2, y**2]))
    th = np.arctan2(y, x)

    return r, th


def calculate_orientation(position, goal):
    """Returns an angle that denotes direction from position to goal in xy plane.

    :param position: numpy array of shape (2+)
    :param goal: numpy array of shape (2+)
    :return: an angle in radians
    """

    if position.ndim > 1:
        return np.arctan2((goal[1] - position[:, 1]), (goal[0] - position[:, 0]))
    else:
        return np.arctan2((goal[1] - position[1]), (goal[0] - position[0]))


# def calculate_delta(A, B):
#     """calculate distance and delta vector in body frame coordinates at point A"""
#
#     # initial orientation
#     r = A[3]    # phi
#     p = A[4]    # theta
#     y = A[5]    # psi
#
#     # rotation matrix for body frame deltas
#     ROT = np.eye(6)
#
#     # set the rotation using current attitude
#     ROT[0:2, 0:2] = [
#         [cos(p)*cos(y),    cos(r)*sin(y)+sin(r)*sin(p)*cos(y)],
#         [-cos(p)*sin(y),   cos(r)*cos(y)-sin(r)*sin(p)*sin(y)]
#     ]
#
#     # body frame rotation
#     A_body = np.dot(A, np.linalg.inv(ROT))
#     B_body = np.dot(B, np.linalg.inv(ROT))
#
#     # calculate delta vector in body reference
#     delta = np.abs(B_body - A_body)
#
#     # distance between A and B in x-y plane
#     distance = np.sqrt(delta[0]**2 + delta[1]**2)
#
#     return (delta, distance)


def distance_between(A, B, spacing_dim=2):
    """Returns a distance between two points in xy plane or xyz space.

    :param A: start point numpy array shape (6)
    :param B: end point numpy array shape (6)
    :param spacing_dim: calculate distance in xy plane (2) or in xyz space (3)
    :return: distance between the points
    """
    squared_delta = np.power((B[0:spacing_dim] - A[0:spacing_dim]), 2)
    return np.sqrt(np.sum(squared_delta))


def leg_distances(points, spacing_dim=2):
    """Produces an array of the lengths of all the intermediate trajectory legs.

    :param points: (n, 6) array of points
    :param spacing_dim: calculate distance in xy plane (2) or in xyz space (3)
    :return: an array of length (n-1) with legs' distances
    """
    # implements distance = sqrt(a^2, b^2, c^2)
    squared_delta = np.power(np.diff(points[:, 0:spacing_dim], axis=0), 2)
    return np.sqrt( np.sum(squared_delta, axis=1) )


def cumulative_distance(points, spacing_dim=2, add_zero_row=False, **kwargs):
    """Produces an array of length n=len(points) with nth entry showing the distance travelled from the start
    to nth point on the list.

    :param points: (n, 6) array of points
    :param spacing_dim: calculate distance in xy plane (2) or in xyz space (3)
    :param add_zero_row: if True the function returns array of length n with zero as the first element
    :return: an array of length (n-1) with cumulative distances
    """
    # implements distance = sqrt(a^2, b^2)
    squared_delta = np.power(np.diff(points[:, 0:spacing_dim], axis=0), 2)
    distance = np.cumsum((np.sqrt(np.sum(squared_delta, axis=1))), axis=0)

    distance = np.concatenate((np.zeros(1), distance))

    return distance


def remove_repeated_points(trajectory):
    """Makes sure no two subsequent points are equal.

    :param trajectory: (n, 6) array of points
    :return: trajectory with consecutive repeated points removed
    """
    unique_indices = [0]
    for i, point in enumerate(trajectory):
        if not np.allclose(point, trajectory[unique_indices[-1]]):
            unique_indices.append(i)
    return trajectory[unique_indices]


def traj_as_dict(points, **kwargs):
    """Utility function to create a trajectory structure from a list of point and optional fields.

    :param points: an N x M matrix of coordinates, representing a trajectory of N waypoints of M dofs
    :param kwargs: optional key-value parameters that are related with a given trajectory
    :return: the trajectory structure (backed by a dictionary)
    """
    trajectory = {}
    trajectory.update(kwargs)
    trajectory['generated'] = datetime.datetime.now().isoformat()
    trajectory['points'] = points.tolist()

    return trajectory


def plot_trajectory(points, show_orientation=True, **kwargs):
    """This function produces a 2D plot of the input trajectory.

    The coordinates are properly swapped in order to respect the vehicle coordinates conventions, namely NED style,
    with navigation angles commonly used for autonomous vehicles operations.

    :param points:
    :param arrow_length:
    :return:
    """
    # styles
    c_ar = kwargs.get('arrow_length', 1)
    c_al = kwargs.get('alpha', 1)
    c_ms = kwargs.get('ms', 6)
    p_style = kwargs.get('p_style', 'or--')
    p_label = kwargs.get('p_label', 'waypoints')

    # check if we want to overlay on an existing plot or a new one
    fig = kwargs.get('fig', None)
    ax = kwargs.get('ax', None)

    if fig is None or ax is None:
        fig, ax = plt.subplots()

    # plots the waypoints
    ax.plot(points[:,1], points[:,0], p_style, alpha=c_al, ms=c_ms, label=p_label)

    if show_orientation:
        # use polar coordinates to produce vehicle orientation arrows
        r = c_ar * np.ones(points.shape[0])     # use a fixed value for arrow length
        th = -points[:,5] + (np.pi / 2)         # convert from navigation angles

        x, y = pol2cart(r, th)
        #bx = ax + points[:,1]
        #by = ay + points[:,0]

        # plots the orientation arrows
        for n in xrange(points.shape[0]):
            ax.arrow(points[n,1], points[n,0], x[n], y[n], fc='k', ec='k', head_width=0.05, head_length=0.1)
            #ax.plot([points[n,1], bx[n]], [points[n,0], by[n]], 'b-')

    # add grid and make axis equals (prevent distortions)
    ax.grid(True)
    ax.axis('equal')
    ax.hold(True)

    # adjust limits
    xl = ax.get_xlim()
    yl = ax.get_ylim()
    xlm = (xl[1] - xl[0]) * 0.5 * 0.1
    ylm = (yl[1] - yl[0]) * 0.5 * 0.1
    xl = (xl[0] - xlm, xl[1] + xlm)
    yl = (yl[0] - ylm, yl[1] + ylm)
    ax.set_xlim(xl)
    ax.set_ylim(yl)

    ax.set_title('Trajectory Plot')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')

    return fig, ax



def interpolate_trajectory(points, spacing=10, spacing_dim=2, face_goal=False, **kwargs):
    """Generates an np.array of intermediate points out of the ndarray of points specified in input_points.

    The vehicle first rotates towards the next point. Then advances towards it in xyz. Then adjusts the final
    orientation.

    !It is possible that points are repeated.

    :param position: first point of the trajectory (optional)
    :param spacing: describes the spacing in xy or xyz between the consecutive points
    :param spacing_dim: describes whether the above spacing is in xy (2) or xyz (3)
    :param face_goal: set the orientation of the point to facing next point
    :return: numpy array of shape (n+1, 6) with points from the input, intermediate points
        and initial position point
    """
    # add initial position at the start of the array
    trajectory = np.copy(points[0].reshape((1, 6)))

    # for every pair of consecutive points find the interpolation
    for i in xrange(1, len(points)):
        interpolation = interpolate_leg(points[i-1], points[i], spacing, spacing_dim, face_goal)[1:]
        trajectory = np.concatenate((trajectory, interpolation), axis=0)

    return trajectory


def interpolate_leg(A, B, spacing, dimensions=2, face_goal=False, min_distance=1, **kwargs):
    """Generates an array of points equispaced along a line between points A and B with the rotation facing point B.

    The array includes points A and B.

    :param A: start point numpy array shape (6)
    :param B: end point numpy array shape (6)
    :param spacing: distance between waypoints
    :param dimensions: how many dimensions should be considered when calculating spacing
    :param face_goal: set the orientation of the point to facing next point
    :param min_distance: minimum distance in order to apply the interpolation (this should be small)
    :return: (n, 6) numpy array with interpolated points
    """
    # roll and pitch to zero
    A[3:5] = 0
    B[3:5] = 0

    # necessary orientation for facing the end point
    distance = distance_between(A, B, dimensions)

    if distance > min_distance:
        steps_advance = np.ceil(distance / spacing) + 1
        face_angle = calculate_orientation(A, B)
    else:
        face_angle = B[5]
        steps_advance = 0

    # allocate the memory: 1 intial point, steps_advance points for advancing and facing, 1 point for orienting
    leg = np.zeros(((steps_advance + 2), 6))
    leg[0] = A

    # if steps_advance is 0 this doesnt generate anything
    leg[1:-1, 0] = np.linspace(A[0], B[0], steps_advance)
    leg[1:-1, 1] = np.linspace(A[1], B[1], steps_advance)
    leg[1:-1, 2] = np.linspace(A[2], B[2], steps_advance)

    if face_goal:
        leg[1:-1, 5] = face_angle
    else:
        angle_diff = cnv.wrap_pi(B[5] - A[5])
        leg[1:-1, 5] = cnv.wrap_pi(np.linspace(0, angle_diff, steps_advance) + A[5])

    # reach the end point
    leg[-1] = B

    return leg


def interpolate_arc(A, B, radius, spacing, right=True, **kwargs):
    """Produces an array of points that lie on an arc between points A and B. If the radius asked is too small to
    generate a trajectory the function will return False, None.

    For a given A, B and radius there are 4 arcs that connect the points. Two are shorter or equal to half of
    the circle. The algorithm will generate one of these two depending on the value of right.For right==True path
    to the right (looking from A to B) will be generated.

    At each of the intermediate points the the orientation is parallel to the tangent of the arc at that point.
    The trajectory includes A and B.

    !It is possible that some points are repeated (start and end).

    :param A: start point numpy array shape (6)
    :param B: end point numpy array shape (6)
    :param radius: radius of the arc
    :param spacing: maximum distance between the points
    :param right: select which path should be generated
    :return: (n, 6) numpy array with trajectory points
    """
    if right is True:
        sign = 1
    else:
        sign = -1

    # only xy plane is considered in the generation of the arc
    mid_point = (A[0:2] + B[0:2]) / 2
    dir_vector = mid_point - A[0:2]
    length = np.linalg.norm(dir_vector)

    if length == 0:
        # points are overlapping
        return np.array([A, B])
    elif length > radius:
        raise ValueError('Radius specified too small')
        # cannot produce the trajectory

    unit_dir_vector = dir_vector / length
    rotation_matrix = -np.array([
        [0, -1],
        [1,  0]
    ])
    # unit vector from the midpoint between A and B to centre of the circle
    rot_unit_vector = np.dot(rotation_matrix, unit_dir_vector)
    # find length necessary to the to the centre
    distance_to_centre = np.sqrt(radius**2 - length**2)
    # there are two solutions, one can go in the positive or negative direction along the vector from the midpoint
    centre = mid_point + sign * distance_to_centre * rot_unit_vector

    # A and B referred to the centre of the circle
    A_ref_centre = A[0:2] - centre
    B_ref_centre = B[0:2] - centre

    # A and B in polar representation
    A_r, A_th = cart2pol(A_ref_centre[0], A_ref_centre[1])
    B_r, B_th = cart2pol(B_ref_centre[0], B_ref_centre[1])

    # arc and angle of the arc to travel
    arc_length = np.abs(radius * (B_th - A_th))

    # this difference is necessary so that the interpolation is in the correct direction
    angle_diff = cnv.wrap_pi(B_th - A_th)

    # number of steps in the interpolation
    steps = np.floor(arc_length/spacing) + 2

    # initialise the array, number of steps + A and B
    trajectory = np.zeros((steps+2, 6))
    trajectory[0] = A
    trajectory[-1] = B

    # generate equispaced angles on the circle to travel to starting from A_th
    angles = cnv.wrap_pi(np.linspace(0, angle_diff, steps) + A_th)

    # calculate xy coordinates corresponding to the points on the circle at a given angle
    xy_ref_centre = np.array(pol2cart(radius, angles)).T
    trajectory[1:-1, 0:2] = xy_ref_centre + centre

    # interpolate z
    trajectory[1:-1, 3] = np.linspace(A[2], B[2], steps)

    # roll, pitch to 0
    trajectory[1:-1, 3:5] = 0

    # adjust the orientation
    #   default orientation is always tangent to the arc thus +-90 degrees to the angle pointing to a given waypoint
    #   (the angle between vector from (x, y) to the centre of the circle and 0 angle)
    trajectory[1:-1, 5] = angles + (-sign * np.pi/2)

    return trajectory


def interpolate_sector(position, radius=5.0, sector=90.0, spacing=1.0, facing_center=True, **kwargs):
    """Generate a trajectory over a circular sector, defined by the starting position of the vehicle, the radius and an
    angle which defines the length of the sector.

    :param position:
    :param radius: radius of the circular shape (m)
    :param sector: circular sector (deg)
    :param spacing: distance between consecutive waypoints (m)
    :param facing_center: boolean flag, if enabled (default) the yaw is set to face the centre of the circular trajectory
    :return: (n, 6) numpy array with trajectory points
    """
    # find center or the arc
    center = np.zeros(6)
    center[0] = position[0] + radius * np.sin(-position[5] + np.pi/2)
    center[1] = position[1] + radius * np.cos(-position[5] + np.pi/2)

    steps = np.deg2rad(sector) / (spacing / radius)
    steps = np.maximum(steps, 18)

    th_start = -calculate_orientation(center, position) + np.pi/2
    th_end = th_start + np.deg2rad(sector)
    theta = np.linspace(th_start, th_end, steps)

    x, y = pol2cart(radius, theta)

    points = np.zeros((steps, 6))
    points[:, 0] = y + center[0]
    points[:, 1] = x + center[1]
    points[:, 2] = position[2]
    points[:, 5] = cnv.wrap_pi( calculate_orientation(points, center) ) #cnv.wrap_pi((-theta + np.pi/2) - np.pi)

    if not facing_center:
        points[:, 5] += (np.pi / 2)

    return points


def interpolate_circle(center, radius=5.0, spacing=1.0, facing_center=True, **kwargs):
    """Generate a circular trajectory give its central point, its radius and an optionally angle which defines the
    coverage of the circle (defaults to a full circular trajectory).

    The standard behaviour is to start from the southern point and complete a full circle around the central point.

    :param center: central waypoint, numpy array of shape (6)
    :param radius: radius of the circular shape (m)
    :param spacing: distance between consecutive waypoints (m)
    :param facing_center: boolean flag, if enabled (default) the yaw is set to face the centre of the circular trajectory
    :return: (n, 6) numpy array with trajectory points
    """
    angle = 2 * np.pi
    total_length = angle * radius

    steps = np.floor(total_length / spacing)
    theta = np.linspace(0.0, angle, num=steps) - (np.pi / 2)

    x, y = pol2cart(radius, theta)

    points = np.zeros((steps, 6))
    points[:, 0] = center[0] + y
    points[:, 1] = center[1] + x
    points[:, 2] = center[2]
    points[:, 5] = cnv.wrap_pi( calculate_orientation(points, center) )

    if not facing_center:
        points[:, 5] += (np.pi / 2)

    return points


def interpolate_helix(centre, radius=5.0, height=2.0, loops=5.0, spacing=1.0, facing_center=True, **kwargs):
    """Generate a helix trajectory give its central point, its radius, its length, and an optionally the number of loops.

    The standard behaviour is to start from the centre's depth and dive for the height provided by the user.

    :param centre: central waypoint, numpy array of shape (6)
    :param radius: radius of the helix shape (m)
    :param height: height of the helix shape (m), as default it increases the depth starting from the centre's depth
    :param loops: number of complete loops around the centre (number)
    :param spacing: distance between consecutive waypoints (m)
    :param facing_center: boolean flag, if enabled (default) the yaw is set to face the centre of the helix trajectory
    :param kwargs:
    :return:(n, 6) numpy array with trajectory points
    """
    angle = (2 * np.pi) * loops
    total_length = angle * radius

    steps = np.floor(total_length / spacing)
    theta = np.linspace(0.0, angle, num=steps) - (np.pi / 2)

    x, y = pol2cart(radius, theta)

    points = np.zeros((steps, 6))
    points[:, 0] = centre[0] + y
    points[:, 1] = centre[1] + x
    points[:, 2] = np.linspace(centre[2], centre[2] + height, num=steps)
    points[:, 5] = cnv.wrap_pi( calculate_orientation(points, centre) )

    if not facing_center:
        points[:, 5] += (np.pi / 2)

    return points


def format_bezier_input(start, p1, p2, end, degrees=False, **kwargs):
    """Generates input points for interpolate_bezier functions out of user-friendly description of the points.

    :param start: standard waypoint, numpy array of shape (6)
    :param p1: steepness of the curve and orientation at the start point - (r, th)
    :param p2: steepness of the curve and orientation at the end point - (r, th)
    :param end: standard waypoint, numpy array of shape (6)
    :param degrees: True for input in degrees
    :return: array (4, 6) of cartesian waypoints that can be passed to bezier interpolation
    """
    P1 = p1.astype(float)
    P2 = p2.astype(float)
    if degrees is True:
        P1[1] = np.deg2rad(P1[1])
        P2[1] = np.deg2rad(P2[1])

    P = np.zeros((4, 6))
    P[0] = start
    P[1, 0:2] = start[0:2] + pol2cart(*P1)
    P[2, 0:2] = end[0:2] - pol2cart(*P2)
    P[3] = end
    return P


def interpolate_bezier(points, steps=100, **kwargs):
    """Generates an array of waypoints which lie on a 2D Bezier curve described by n (x, y) points. The trajectory is
    guaranteed to include the start and end points though only on (x, y, z) axes.

    The curve generated is of the nth degree, where n = len(points) - 1

    1st point is the start point.
    2nd point indicates the orientation at the start point.
    (n-1)th point indicates the orientation at the end point.
    nth point is the end point.

    For information about Bezier curve look at:
    - http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html

    :param points: (n, 2+) array of waypoints
    :return: trajectory with interpolated points
    """
    n = len(points) - 1
    t = np.linspace(0, 1, steps).reshape((steps, 1))
    B = np.zeros((steps, 6))

    # could be vectorised:
    # r = range(0, n+1)
    # coefs = sci.misc.comb(n, r)
    # t_1_pow = np.power(np.tile(t-1, (1, 6)), np.tile(r, (steps, 1)))
    # t_pow = np.power(np.tile(t, (1, 6)), np.tile(r, (steps, 1)))
    # etc
    for i in xrange(n+1):
        B[:, 0:2] += sci.misc.comb(n, i) * np.dot(((1-t)**(n-i) * t**i).reshape(100, 1), points[i, 0:2].reshape((1, 2)))
        # coef = sci.misc.comb(n, i)
        # B[:, 0] += coef * (1-t)**(n-i) * t**i * points[i, 0]
        # B[:, 1] += coef * (1-t)**(n-i) * t**i * points[i, 1]
    B[:, 2] = np.linspace(points[0, 2], points[-1, 2], steps)
    B[:, 3:5] = 0
    der_x = np.diff(B[:, 0])
    der_y = np.diff(B[:, 1])
    B[1:, 5] = np.arctan2(der_y, der_x)
    B[0] = points[0]
    return B


def interpolate_bezier_cubic(points, steps=100, **kwargs):
    """Equivalent to interpolate_bezier with 4 input points (degree 3)

    :param points: (n, 2+) array of waypoints
    :return: trajectory with interpolated points
    """
    P0, P1, P2, P3 = points
    t = np.linspace(0, 1, steps)
    B = np.zeros((steps, 6))

    B[:, 0] = (1-t)**3 * P0[0] + 3*(1-t)**2 * t * P1[0] + 3*(1-t)*t**2 * P2[0] + t**3 * P3[0]
    B[:, 1] = (1-t)**3 * P0[1] + 3*(1-t)**2 * t * P1[1] + 3*(1-t)*t**2 * P2[1] + t**3 * P3[1]
    B[:, 2] = np.linspace(P0[2], P3[2], steps)
    B[:, 3:5] = 0

    # calculate the xy slope at each point of the curve
    der_x = np.diff(B[:, 0])
    der_y = np.diff(B[:, 1])
    B[1:, 5] = np.arctan2(der_y, der_x)

    return B
