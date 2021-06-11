#!/usr/bin/env python

import time

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

class PID_CONTROLLER_BELBIC:

	def __init__(self, p_coef, d_coef, i_coef):

		self.kp = p_coef
		self.kd = d_coef
		self.ki = i_coef

		self._last_time      = 0.0
		self.error_integ     = 0.0
		self._previous_error = 0.0

		self._i_max =  5.0  # The integral upper limit.
		self._i_min = -5.0  # The integral lower limit.


		self._is_error_initialized_PID = False


	def set_SI (self, error):

		cur_time = time.time()
		output   = self.kp * error

		if self._is_error_initialized_PID:

			dt                   = cur_time - self._last_time
			self._last_time      = cur_time
			self.error_integ     += error * dt


			#error_diff           = (error - self._previous_error) / dt
			error_diff           = error - self._previous_error
			self._previous_error = error

			derivativa           = self.kd *  error_diff
			integral             = self.ki *  self.error_integ

			if  integral  > self._i_max:
				integral  = self._i_max
			elif integral < self._i_min:
				integral  = self._i_min

			output +=  derivativa + integral

		else:

			self._previous_error           = error
			self._last_time                = cur_time
			self._is_error_initialized_PID = True

		return output
