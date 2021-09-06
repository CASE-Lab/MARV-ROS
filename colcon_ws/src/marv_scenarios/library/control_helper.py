import numpy as np

class FirFilter:
    def __init__(self, filter_coeff: np.ndarray, buffer_init = 0):
        self._buffer = np.zeros(len(filter_coeff)) * buffer_init
        self._filter_coeff = np.flip(filter_coeff,0)
        self.last_filtered_value = None

    def filter(self, input: float) -> float:
        # push new input into buffer
        # https://stackoverflow.com/questions/42771110/fastest-way-to-left-cycle-a-numpy-array-like-pop-push-for-a-queue
        self._buffer[:-1] = self._buffer[1:]
        self._buffer[-1] = input
        # apply FIR filter to buffer 
        self.last_filtered_value = np.sum(self._buffer * self._filter_coeff)
        return self.last_filtered_value

class RateLimiter:
    def __init__(self, sample_rate, rate_limit, inital_value = 0):
        self._discrete_rate_limit = 1/sample_rate * rate_limit
        self._prev_value = inital_value
    
    def limit(self, input):
        if input - self._prev_value > self._discrete_rate_limit:
            self._prev_value = self._prev_value + self._discrete_rate_limit
        elif input - self._prev_value < -self._discrete_rate_limit:
            self._prev_value = self._prev_value - self._discrete_rate_limit
        else:
            self._prev_value = input
        return self._prev_value

class Unwrapper:
    def __init__(self, initial_value = 0, tol = np.pi+0.1):
        self._tol = tol
        self._prev_value = initial_value

    def unwrap(self, input):
        self._prev_value = np.unwrap([self._prev_value, input] , discont = self._tol)[1]
        return self._prev_value

class Integrator:
    '''
    Discrete integrator expected to be called at fixed rate
    equal to sample_rate (hz)
    '''
    def __init__(self, sample_rate, inital_value = 0):
        self.current_value = inital_value
        self.sample_rate = sample_rate

    def integrate(self, input):
        self.current_value = self.current_value +  1/self.sample_rate * input
        return self.current_value

class AntiWindup:
    '''
    Attempt at discrete anti-windup
    '''
    def __init__(self, gain, initial_value = 0):
        self._saturation_diff = initial_value
        self.gain = gain

    def get_feedback(self):
        return self._saturation_diff * self.gain

    def update(self, saturation_diff):
        self._saturation_diff = saturation_diff

def wrapToPi(input):
    # https://stackoverflow.com/questions/15927755/opposite-of-numpy-unwrap/15927914
    return (input + np.pi) % (2 * np.pi) - np.pi