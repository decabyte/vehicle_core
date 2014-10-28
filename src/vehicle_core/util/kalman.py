#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

KF_STR = """
x: %s,
A: %s,
P: %s,
Q: %s,
S: %s,
K: %s,
H: %s,
Z: %s,
I: %s,
y: %s
"""

class KalmanFilter(object):
    """This class implements the standard Kalman filter using Numpy ndarrays and a simple API.

    The filter requires two parameters to be created, the number of states and the number of observations. The internal
    variables can be set after the filter instantiation in order to get the required behaviour.

    As default the filter allocates the intermediate variables and set them to identity matrices and zeros vectors.

    See also:
        [1]: http://balzer82.github.io/Kalman/
        [2]: http://nbviewer.ipython.org/github/balzer82/Kalman/blob/master/Kalman-Filter-CV.ipynb?create=1
    """

    def __init__(self, n_state, n_observation, **kwargs):
        """Creates a new Kalman filter object with default values for the internal parameters.

        :param n_state: the number of the internal states
        :param n_observation: the number of the external measurements
        :param kwargs: optional values (like the matrices A, Q, R, H, etc.)
        """

        self.n_state = n_state
        self.n_observation = n_observation

        self.A = np.eye(n_state)
        self.x = np.zeros((n_state, 1))
        self.P = np.eye(n_state)
        self.Q = np.eye(n_state)

        self.R = np.eye(n_observation)
        self.I = np.eye(n_state)
        self.H = np.eye(n_observation, n_state)
        self.K = np.zeros((n_state, n_observation))

        self.Z = np.zeros((n_observation, 1))

        # auxiliary
        self.S = np.zeros_like(self.R)
        self.y = np.zeros_like(self.Z)

        # parse keyword arguments (ie. set initial values)
        # ...


    def update(self, measurements, **kwargs):
        """This function runs the filter over a new set of measurements, executing the two steps of "prediction" and
        "correction" using the measurements given by the caller.

        :param measurements: an ndarray of measurements with shape (n_observation, 1).
        :return: x, P: the filter means, an ndarray with shape (n_state, 1), and the filter covariances, an ndarray of shape (n_state, n_state).
        """
        # Time Update (Prediction)
        # ========================
        # Project the state ahead
        self.x = np.dot(self.A, self.x)                                             # x = A * x

        # Project the error covariance ahead
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q                  # P = A * P * A.T + Q

        # Measurement Update (Correction)
        # ===============================
        # Compute the Kalman Gain
        self.S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R                  # S = H * P * H.T + R
        self.K = np.dot(np.dot(self.P, self.H.T), np.linalg.pinv(self.S))           # K = P * H.T * inv(S)

        # Update the estimate via z
        self.Z = measurements.reshape((self.n_observation, 1))
        self.y = self.Z - np.dot(self.H, self.x)                                    # y = Z - H * x
        self.x = self.x + np.dot(self.K, self.y)                                    # x = x + K * y

        # Update the error covariance
        self.P = np.dot((self.I - np.dot(self.K, self.H)), self.P)                  # P = (I - K * H) * P

        return self.x, self.P


    def __str__(self):
        return KF_STR % (self.x, self.A, self.P, self.Q, self.S, self.K, self.H, self.Z, self.I, self.y)


# kalman filter test
if __name__ == '__main__':
    import matplotlib.pyplot as plt

    # config
    n_state = 1
    n_observation = 2
    n_samples = 100

    # kf params
    kf = KalmanFilter(n_state, n_observation)
    kf.H[0,0] = 1
    kf.H[1,0] = 1
    kf.Q[0,0] = 1000.0
    kf.R[0,0] = 2.0
    kf.R[1,1] = 1.0

    # data
    t = np.arange(n_samples)
    measurements = np.zeros((n_samples, n_observation))
    measurements[:,0] = 10 * np.sin(t) + np.random.random(n_samples) * 10.0
    measurements[:,1] = 10 * np.sin(t) + np.random.random(n_samples) * 5.0


    fusion = np.zeros((n_samples, n_state))
    covariances = np.zeros((n_samples, n_state, n_state))

    for n in np.arange(measurements.shape[0]):
        fusion[n, :], covariances[n, :, :] = kf.update(measurements[n, :])
        print('step: %s -- k: %s' % (n, kf.K))


    fig, ax = plt.subplots()
    ax.plot(t, measurements[:, 0], 'b--', label='sensor #1')
    ax.plot(t, measurements[:, 1], 'g--', label='sensor #2')
    ax.plot(t, fusion[:, 0], 'r', label='filter output')
    ax.legend()

    plt.show()
