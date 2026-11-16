"""delay_comp.py -- predict the drone state forward over a known command delay (Smith predictor).

A command sent at time s acts on the drone at s + delay (radio, flight-controller filtering,
motors). The measured state at time t has therefore not yet seen the commands sent in
[t - delay, t]. Integrating the tracker's own model from the measurement with those commands
gives the state at t + delay, where the command solved now will start to act. Pure: no ROS.
"""
from collections import deque

import numpy as np


class DelayPredictor:
    def __init__(self, f_dyn, delay_s, step_s=0.01):
        """f_dyn: the tracker's casadi x_dot(x17, u_dot4, p9)."""
        self.f = f_dyn
        self.delay = float(delay_s)
        self.h = float(step_s)
        self.hist = deque()                 # (t, u4) of every command sent

    def record(self, t, u):
        self.hist.append((float(t), np.asarray(u, float).copy()))
        while len(self.hist) > 2 and self.hist[1][0] < t - 2.0 * self.delay - 0.1:
            self.hist.popleft()

    def _u_at(self, t):
        """The command in force at time t: the last one sent at or before t (zeros before any)."""
        u = np.zeros(4)
        for ts, us in self.hist:
            if ts > t:
                break
            u = us
        return u

    def predict(self, t, x13, p9):
        """x13 = [p, q(wxyz), v, w] measured at t -> the model state at t + delay."""
        x = np.concatenate([np.asarray(x13, float), np.zeros(4)])
        if self.delay <= 0.0 or not self.hist:
            return np.asarray(x13, float)
        n = max(1, int(round(self.delay / self.h)))
        h = self.delay / n
        zero = np.zeros(4)
        for k in range(n):
            s = t + k * h
            # the command acting at s was sent at s - delay; hold it across the step
            x[13:17] = self._u_at(s - self.delay)

            def fx(xx):
                return np.array(self.f(xx, zero, p9)).flatten()
            k1 = fx(x)
            k2 = fx(x + 0.5 * h * k1)
            k3 = fx(x + 0.5 * h * k2)
            k4 = fx(x + h * k3)
            x = x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
            x[3:7] /= max(np.linalg.norm(x[3:7]), 1e-9)
        return x[:13]
