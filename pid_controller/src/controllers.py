#!/usr/bin/env python3

import time


class PD_Controller:
    def __init__(self, p_coef: float, d_coef: float) -> None:
        self.kp = p_coef
        self.kd = d_coef
        self._previous_error: float = 0.0
        self._is_error_initialized = False

    def set_REW(self, error: float):

        output = error * self.kp

        if self._is_error_initialized:

            error_diff = error - self._previous_error
            output += self.kd * error_diff
            self._previous_error = error

        else:

            self._previous_error = error
            self._is_error_initialized = True

        return output


class PID_Belbic(PD_Controller):
    def __init__(self, p_coef, d_coef, i_coef, limit_out):
        super().__init__(p_coef, d_coef)
        self.ki = i_coef
        self._limit_out = limit_out
        self._last_time = 0.0
        self.error_integ = 0.0
        self._is_error_initialized_PID = False

    def set_SI(self, error):

        cur_time = time.time()
        output = self.kp * error

        if self._is_error_initialized_PID:

            dt = cur_time - self._last_time
            self._last_time = cur_time
            self.error_integ += error * dt

            error_diff = error - self._previous_error
            self._previous_error = error

            derivativa = self.kd * error_diff
            integral = self.ki * self.error_integ

            if integral > self._limit_out:
                integral = self._limit_out
            elif integral < (-self._limit_out):
                integral = -self._limit_out

            output += derivativa + integral

        else:

            self._previous_error = error
            self._last_time = cur_time
            self._is_error_initialized_PID = True

        return output
