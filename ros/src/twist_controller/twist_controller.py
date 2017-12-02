from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
CONTROL_RATE = 50.0  # Htz
CONTROL_PERIOD_ = 1.0 / CONTROL_RATE


class Controller(object):
    def __init__(self, dbw_node):
        self.dbw_node = dbw_node
        self.min_speed = 0.1
        
        self.yaw_control = YawController(self.dbw_node.wheel_base,
                                         self.dbw_node.steer_ratio,
                                         self.min_speed,
                                         self.dbw_node.max_lat_accel,
                                         self.dbw_node.max_steer_angle)


        # Values of Kp, Ki, and Kd are from DataSpeed example
        # This is really only a proportional filter
        self.velo_pid = PID(2.0, 0.0, 0.0)
        
        # Throttle is between 0.0 and 1.0
        # Values of Kp, Ki, and Kd are from DataSpeed example
        self.accel_pid = PID(0.4, 0.1, 0.0, 0.0, 1.0)
        
        # for accel
        self.lp_filter = LowPassFilter(0.5, 0.02)

    def control(self, proposed_linear, proposed_angular,
                current_linear, current_angular):

        throttle = 0.6
        # just add in a little proportional control
        steering = self.yaw_control.get_steering(proposed_linear,
                                                 proposed_angular,
                                                 current_linear)
        brake = 0.0

        # vel_error = proposed_linear_velocity - current_linear_velocity
        # if proposed_linear_velocity < self.min_speed:
        #     self.velo_pid.reset()

        # accel_cmd = self.velo_pid.step(vel_error, CONTROL_PERIOD)
        # if accel_cmd >= 0:
        #     #throttle = self.accel_pid.step(accel_cmd - lpf_accel_.get(), CONTROL_PERIOD)
        #     th = self.accel_pid.step(accel_cmd, CONTROL_PERIOD)
        # else:
        #     accel_pid.reset()
        #     #throttle = 0.0
        
        return throttle, brake, steering
