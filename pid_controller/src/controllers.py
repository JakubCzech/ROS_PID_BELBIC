#!/usr/bin/env python3

import time


class PD_controller:
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


class PID_controller(PD_controller):
    def __init__(self, p_coef: float, i_coef: float, d_coef: float, limit_out: float):
        super().__init__(p_coef, d_coef)

        self.ki = i_coef
        self._limit_out = limit_out

    def set_current_error(self, error: float):

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
            output = -self._limit_out

        return output


class PID_Belbic(PID_controller):
    def __init__(self, p_coef, d_coef, i_coef):
        super().__init__(p_coef, d_coef, i_coef)

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
