from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, dbw_node):
        self.dbw_node = dbw_node
        self.min_speed = 0.1
        self.yaw_control = YawController(self.dbw_node.wheel_base,
                                         self.dbw_node.steer_ratio,
                                         self.min_speed,
                                         self.dbw_node.max_lat_accel,
                                         self.dbw_node.max_steer_angle)

        # Parameters taken from PID project (but that was for steering!)
        self.kp = 0.2
        self.ki = 0.001
        self.kd = 0.85
        self.pid_controller = PID(self.kp, self.ki, self.kd)
        
        # FIXME Need real values for tau and ts
        tau = 0.1
        ts  = 0.02
        self.lp_filter = LowPassFilter(tau, ts)

    def control(self, plv, pav, clv):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        #throttle = pid_controller()
        throttle = 0.5
        steering = self.yaw_control.get_steering(plv, pav, clv)
        brake = 0.0
        return throttle, brake, steering
