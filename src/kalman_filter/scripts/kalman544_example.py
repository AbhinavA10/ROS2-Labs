#!/usr/bin/env python3

import numpy as np
from numpy.random import randn
import matplotlib.pyplot as plt

## Example code written by: Christian Mele, adapted to Python by Yue Hu

class KalmanFilter():
    def __init__(self):
    
        # Time step of analysis
        self.dt = 0.1 # (relatively slow refresh rate)

        ## Prediction estimation - this is your "Priori"
        # we will be using IMU values for the project, however in this example we
        # use a block with a spring attached to it
        self.xhat = np.matrix([0.5, 1, 0]).transpose() # mean(mu) estimate for the "first" step
        self.P = np.identity(3) # covariance initial estimation between the
        # the covariance matrix P is the weighted covariance between the
        # motion model definition - establish your robots "movement"
        k = 1 # spring value
        m = 10 # mass

        self.A = np.matrix([[1, self.dt, 1 / 2 * self.dt ** 2], [0, 1, self.dt], [k / m, 0, 0]]) # this is the state space
        self.B = np.matrix([0, 0, 1/m]).transpose() # this is the "input"
        self.B = np.matrix([0, 0, 1]).transpose() # we can also treat the input as the "acceleration" for that step as calculated by an IMU!
        self.Q = np.identity(3)*0.05 # this is the model variance/covariance (usually we treat it as the input matrix squared).
                                     # these are the discrete components, found by performing a taylor's
                                     # expansion on the continuous model

        ## Measurement Model - Update component
        self.C = np.matrix([[-1, 0, 0], [0, 0.6, 0]]) # what this represents is our "two" sensors, both with linear relationships
                                                      # to position and velocity respectively

        # in actual environments, what this does is translate our measurement to a
        # voltage or some other metric that can be ripped directly from the sensor
        # when taking online measurements. We compare those values as our "error"

        self.R = np.matrix([[0.05, 0], [0, 0.05]]) # this is the sensor model variance-usually characterized to accompany
                                                  # the sensor model already starting

    def run_kalman_filter(self, Tfinal):
        T = np.arange(0, Tfinal, self.dt)
        xhat_S = np.zeros([3, len(T) + 1])
        x_S = np.zeros([3, len(T) + 1])
        x = np.zeros([3, len(T) + 1])
        x[:, [0]] = self.xhat
        y = np.zeros([2, len(T)])
        for k in range(len(T)):
            u = 0.01 # normally you'd initialise this above

            #### Simulate motion with random motion disturbance ####
            w = np.matrix([self.Q[0, 0] * randn(1), self.Q[1, 1] * randn(1), self.Q[2, 2] * randn(1)])

            # update state - this is a simulated motion and is PURELY for fake
            # sensing and would essentially be
            x[:, [k + 1]] = self.A * x[:, [k]] + self.B * u + w

            # taking a measurement - simulating a sensor
            # create our sensor disturbance
            v = np.matrix([self.R[0, 0] * randn(1), self.R[1, 1] * randn(1)])
            # create this simulated sensor measurement
            y[:, [k]] = self.C*x[:, [k+1]] + v

            #########################################
            ###### Kalman Filter Estimation #########
            #########################################
            # Prediction update
            xhat_k = self.A * self.xhat + self.B * u # we do not put noise on our prediction
            P_predict = self.A*self.P*self.A.transpose() + self.Q
            # this co-variance is the prediction of essentially how the measurement and sensor model move together
            # in relation to each state and helps scale our kalman gain by giving
            # the ratio. By Definition, P is the variance of the state space, and
            # by applying it to the motion model we're getting a motion uncertainty
            # which can be propogated and applied to the measurement model and
            # expand its uncertainty as well

            # Measurement Update and Kalman Gain
            K = P_predict * self.C.transpose()*np.linalg.inv(self.C*P_predict*self.C.transpose() + self.R)
            # the pseudo inverse of the measurement model, as it relates to the model covariance
            # if we don't have a measurement for velocity, the P-matrix tells the
            # measurement model how the two should move together (and is normalised
            # in the process with added noise), which is how the kalman gain is
            # created --> detailing "how" the error should be scaled based on the
            # covariance. If you expand P_predict out, it's clearly the
            # relationship and cross-projected relationships, of the states from a
            # measurement and motion model perspective, with a moving scalar to
            # help drive that relationship towards zero (P should stabilise).

            self.xhat = xhat_k + K * (y[:, [k]] - self.C * xhat_k)
            self.P = (np.identity(3) - K * self.C) * P_predict # the full derivation for this is kind of complex relying on
                                             # some pretty cool probability knowledge

            # Store estimate
            xhat_S[:, [k]] = xhat_k
            x_S[:, [k]] = self.xhat

        return x, xhat_S, x_S

    def plot_results(self, Tfinal, x, xhat_S, x_S):
        T = np.arange(0, Tfinal, self.dt)
        plt.figure()
        plt.plot(T, x_S[0,0:-1])
        plt.plot(T, x_S[1,0:-1])
        plt.plot(T, x_S[2,0:-1])
        plt.plot(T, x[0,1:])
        plt.plot(T, x[1,1:])
        plt.plot(T, x[2,1:])
        plt.legend(['position est.','vel estimate', 'accel est', 'true pos', 'true vel', 'true accel'])

        plt.figure()
        plt.plot(T, xhat_S[0,0:-1])
        plt.plot(T, xhat_S[1,0:-1])
        plt.plot(T, xhat_S[2,0:-1])
        plt.plot(T, x[0,1:])
        plt.plot(T, x[1,1:])
        plt.plot(T, x[2,1:])
        plt.legend(['position pred.','vel pred.', 'accel pred.', 'true pos', 'true vel', 'true accel'])

        plt.show()

def main():
    print("MTE544 Final Project - Kalman Filter")
    kf = KalmanFilter()
    Tfinal = 10
    x, xhat_S, x_S = kf.run_kalman_filter(Tfinal)
    kf.plot_results(Tfinal, x, xhat_S, x_S)

if __name__ == '__main__':
    main()