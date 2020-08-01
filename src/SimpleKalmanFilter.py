import os
import sys
import math

class SimpleKalmanFilter(object):
    
    def __init__(self, mea_e, est_e, q):
        self.err_measure = mea_e
        self.err_estimate = est_e
        self.q = q
        self.kalman_gain = 0.0
        self.last_estimate = 0.0
        self.current_estimate = 0.0

    def updateEstimate(self, mea):
        self.kalman_gain = self.err_measure/(self.err_estimate + self.err_measure)
        self.current_estimate = self.last_estimate + self.kalman_gain * (mea - self.last_estimate)
        self.err_estimate = (1.0 - self.kalman_gain) * self.err_estimate + math.fabs(self.last_estimate - self.current_estimate) * self.q
        self.last_estimate = self.current_estimate

        return self.current_estimate

    def setMesurementError(self, mea_e):
        self.err_measure = mea_e

    def setEstimateError(self, est_e):
        self.err_estimate = est_e

    def setProcessNoise(self, q):
        self.q = q

    def getKalmanGain(self):
        return self.kalman_gain

    def getEstimateError(self):
        return self.err_estimate

