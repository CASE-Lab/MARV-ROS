import numpy as np
import control_helper as ch

class ThrustController:
    '''
    Thrust controller should take following inputs every itteration
    Surge reference     m/s
    Measured surge      m/s

    Controller takes following configuratino parameters
    Reference shaping FIR filter coeficcients
    thottle_limit (m/s) given as [min, max]
    surge_rate limit given in m/s
    Sample rate (hz)
    Controller gain [k1 k2]
    Anti windup gain (needs to be negative)
    '''
    def __init__(self,
                 fir_coeff: np.ndarray,
                 rate_limit,
                 sample_rate,
                 controller_gain: list,
                 thottle_limit: list,
                 anti_windup_gain):

        self._rate_limiter = ch.RateLimiter(sample_rate, rate_limit, 0)
        self._shaping_filter = ch.FirFilter(fir_coeff,0)
        self._gain = controller_gain
        self._thottle_limit = thottle_limit
        self._anti_windup = ch.AntiWindup(anti_windup_gain, 0)
        self._integrator = ch.Integrator(sample_rate, 0)
    

    def run_controller(self, thrust_ref, thrust_meas):
        # apply refrence shaping:
        thrust_reference = self._rate_limiter.limit(thrust_ref)
        thrust_reference = self._shaping_filter.filter(thrust_reference)

        # calcualte heading error:
        thrust_error = thrust_reference - thrust_meas

        # calculate integrated heading error and subtract anti-windup
        thrust_error_int = self._integrator.integrate(thrust_error + self._anti_windup.get_feedback())

        # calcualte control output
        control = self._gain[0] * thrust_error_int + self._gain[1] * thrust_error

        # calcualte saturated control ouput
        control_saturated = np.clip(control, self._thottle_limit[0], self._thottle_limit[1] )

        # update anti windup input for next itteration
        self._anti_windup.update(control - control_saturated)

        return control_saturated