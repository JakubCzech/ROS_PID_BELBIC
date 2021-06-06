#!/usr/bin/env python


class PID_CONTROLLER:

    def __init__(self, p_coef, i_coef, d_coef, limit_out):

        self.kp = p_coef
        self.ki = i_coef
        self.kd = d_coef
        self._limit_out = limit_out
        self._previous_error = 0

    def set_current_error(self, error):

        output0 = error * self.kp

        error_diff = error - self._previous_error
        outpu1 = self.kd * error_diff

        error_intr = error + self._previous_error
        outpu2 = self.ki * error_intr

        self._previous_error = error

        output = output0 + outpu1 + outpu2

        if output > self._limit_out:
            output = self._limit_out
        elif output < (-self._limit_out):
            output = (-self._limit_out)

        return output
