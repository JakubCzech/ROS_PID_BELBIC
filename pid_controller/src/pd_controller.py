class PD_CONTROLLER:
    def __init__(self, p_coef, d_coef):

        self.kp = p_coef
        self.kd = d_coef
        self._previous_error = 0.0
        self._is_error_initialized = False

    def set_REW(self, error):

        output = error * self.kp

        if self._is_error_initialized:

            error_diff = error - self._previous_error
            output += self.kd * error_diff
            self._previous_error = error

        else:

            self._previous_error = error
            self._is_error_initialized = True

        return output
