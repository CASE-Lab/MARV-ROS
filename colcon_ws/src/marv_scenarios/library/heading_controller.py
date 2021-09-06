import numpy as np
import control_helper as ch

class HeadingController:
    '''
    Heading controller should take following inputs every itteration
    Heading reference     [-pi, pi] rad
    Measured heading      [-pi, pi] rad
    Measured yaw_rate     rad/s

    Heading controller takes following configuratino parameters
    Reference shaping FIR filter coeficcients
    Reference shaping rate limit (rad/s)
    Sample rate (hz)
    Controller gain [k1 k2 k3]
    '''
    def __init__(self,
                 fir_refshape: np.ndarray,
                 fir_yaw: np.ndarray,
                 rate_limit,
                 sample_rate,
                 controller_gain: list,
                 steering_limit,
                 anti_windup_gain,
                 initial_heading):

        self._unwrapper = ch.Unwrapper()
        self._rate_limiter = ch.RateLimiter(sample_rate, rate_limit, initial_heading)
        self._shaping_filter = ch.FirFilter(fir_refshape,initial_heading)
        self._yaw_filter = ch.FirFilter(fir_yaw, 0)
        self._gain = controller_gain
        self._steering_limit = steering_limit
        self._anti_windup = ch.AntiWindup(anti_windup_gain, 0)
        self._integrator = ch.Integrator(sample_rate, 0)
    
    def heading_diff(self, head_ref, head_meas) -> float:
        '''
        Calculates shortest turn direction and magnitude based on
        current measured heading and heading reference.
        All angles are in radians
        '''
        heading_diff = head_ref - head_meas

        if heading_diff < - np.pi:
            heading_error = heading_diff + 2*np.pi
        elif heading_diff >= np.pi:
            heading_error = heading_diff - 2*np.pi
        else:
            heading_error = heading_diff
        
        return heading_error

    def run_controller(self, head_ref, head_meas, yaw_rate):
        # apply yaw filtering
        yaw_filtered = self._yaw_filter.filter(yaw_rate)
        # apply refrence shaping:
        heading_reference = self._unwrapper.unwrap(head_ref)
        heading_reference = self._rate_limiter.limit(heading_reference)
        heading_reference = self._shaping_filter.filter(heading_reference)
        heading_reference = ch.wrapToPi(heading_reference)

        # calcualte heading error:
        heading_error = self.heading_diff(heading_reference, head_meas)

        # calculate integrated heading error and subtract anti-windup
        heading_error_int = self._integrator.integrate(heading_error + 1*self._anti_windup.get_feedback())

        # calcualte control output
        control = self._gain[0] * heading_error_int + self._gain[1] * heading_error - self._gain[2] * yaw_filtered

        # calcualte saturated control ouput
        control_saturated = np.clip(control, -self._steering_limit, self._steering_limit )

        # update anti windup input for next itteration
        self._anti_windup.update(control - control_saturated)

        return np.rad2deg(control_saturated)