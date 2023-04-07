"""
    MEAM 517 Final Project - LQR Steering Control - LQR class
    Author: Derek Zhou & Tancy Zhao
    References: https://github.com/f1tenth/f1tenth_gym/tree/main/examples
"""

import numpy as np
import math


def calc_nearest_point(point, trajectory):
    """
    Return the nearest point along the given piecewise linear trajectory.

    Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
    not be an issue so long as trajectories are not insanely long.

        Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)

    point: size 2 numpy array
    trajectory: Nx2 matrix of (x,y) trajectory waypoints
        - these must be unique. If they are not unique, a divide by 0 error will destroy the world
    """
    diffs = trajectory[1:, :] - trajectory[:-1, :]  # piecewise distances y between every 2 points
    l2s = diffs[:, 0] ** 2 + diffs[:, 1] ** 2

    # this is equivalent to the elementwise dot product
    # dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
    dots = np.empty((trajectory.shape[0] - 1,))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])  # =|x|*|y|*cos(θ), x is curr pos to waypoint i

    t = dots / l2s  # =|x|*cos(θ)/|y| project vector x on vector y, as projection proportion
    t[t < 0.0] = 0.0  # x & y have 90° angle or more
    t[t > 1.0] = 1.0  # x & y have 45° angle or less
    # t = np.clip(dots / l2s, 0.0, 1.0)

    projections = trajectory[:-1, :] + (t * diffs.T).T  # |x|*cos(θ) on waypoint i -- x's projection on y
    # dists = np.linalg.norm(point - projections, axis=1)
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]  # vector
        dists[i] = np.sqrt(np.sum(temp * temp))  # distance between current point and waypoint
    index = np.argmin(dists)  # get index of min distance

    return projections[index], dists[index], t[index], index


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi
