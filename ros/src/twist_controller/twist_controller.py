import rospy
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
CONTROL_RATE = 50  # Htz
CONTROL_PERIOD = 1.0 / CONTROL_RATE
PASSENGER_MASS = 150
SPEED_EPS = 0.1

class Controller(object):
    def __init__(self, dbw_node):
        self.dbw_node = dbw_node
        self.min_speed = 0.1

        # This assumes that the gas tank is always full
        self.vehicle_mass = self.dbw_node.vehicle_mass + \
                            GAS_DENSITY * self.dbw_node.fuel_capacity + \
                            PASSENGER_MASS
        
        self.yaw_control = YawController(self.dbw_node.wheel_base,
                                         self.dbw_node.steer_ratio,
                                         self.min_speed,
                                         self.dbw_node.max_lat_accel,
                                         self.dbw_node.max_steer_angle)


        # Values of Kp, Ki, and Kd are from DataSpeed example
        # This is really only a proportional filter
        # (curiously, it looks like DS sets both min and max to 9.8
        self.velo_pid = PID(2.0, 0.0, 0.0, -9.8, 9.8)
        
        # this set of values worked OK for 30mph, but that is too high
        # for this project
        #self.velo_pid = PID(2.0, 0.0, 0.1, -9.8, 9.8)
        
        # Throttle is between 0.0 and 1.0
        # Values of Kp, Ki, and Kd are from DataSpeed example
        self.accel_pid = PID(0.4, 0.1, 0.0, 0.0, 1.0)
        #self.accel_pid = PID(0.4, 0.1, 0.2, 0.0, 1.0)
        

    # This only does yaw control at constant throttle.  Now used
    # for debugging only
    def simple_control(self, proposed_linear, proposed_angular,
                       current_linear, current_angular):
        #rospy.logwarn("current_linear: %f" % current_linear)
        throttle = 0.6
        steering = self.yaw_control.get_steering(proposed_linear,
                                                 proposed_angular,
                                                 current_linear)
        brake = 0.0
        return throttle, brake, steering
        

    
    def control(self, proposed_linear, proposed_angular,
                     current_linear, current_angular):

        # First, get the steering angle
        steering = self.yaw_control.get_steering(proposed_linear,
                                                 proposed_angular,
                                                 current_linear)

        velo_error = proposed_linear - current_linear


        if abs(velo_error) < SPEED_EPS or proposed_linear < self.min_speed:
            self.velo_pid.reset()

        accel_est = self.velo_pid.step(velo_error, CONTROL_PERIOD)

        # rospy.logwarn("proposed: %f  current: %f  error: %f  accel: %f" % \
        #               (proposed_linear, current_linear, velo_error, accel_est))
        
        if accel_est >= 0:
            filtered_accel = self.dbw_node.lp_filter.get()
            delta_accel = accel_est - filtered_accel
            tctrl = self.accel_pid.step(delta_accel, CONTROL_PERIOD)
            throttle = tctrl
            #rospy.logwarn("  filtered: %f   delta: %f   throttle: %f" % (filtered_accel,
            #                                                            delta_accel, tctrl))
        else:
            self.accel_pid.reset()
            throttle = 0.0

        if accel_est < -self.dbw_node.brake_deadband:
            # braking takes a positive value
            calc_brake = -accel_est * self.vehicle_mass * self.dbw_node.wheel_radius
            brake = min(calc_brake, -self.dbw_node.decel_limit)
            #rospy.logwarn("CALC_BRAKING: %f   BRAKE %f" % (calc_brake, brake))
        else:
            brake = 0.0
            
        # dbw not engaged, don't accumulate error
        if  not self.dbw_node.dbw_enabled:
            self.velo_pid.reset()
            self.accel_pid.reset()
            
        return throttle, brake, steering
