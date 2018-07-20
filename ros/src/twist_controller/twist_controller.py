from __future__ import print_function
import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        
        # Controllers
        yaw_param = {}
        for key in ["wheel_base", "steer_ratio", "min_speed", "max_lat_accel","max_steer_angle"]:
            yaw_param[key] = kwargs[key]

        self.yaw_controller = YawController(**yaw_param)
        k = [x*5 for x in [1, 0.1, 0.001]] # PID coefficients
        output_range = [-0.5, 1.0]
        pid_param = k + output_range
        self.throttle_controller = PID(*pid_param)

        # Low pass filters
        tau = 0.5 # Cutoff frequency
        ts = 0.02 # Sampling period
        self.vel_lpf = LowPassFilter(tau, ts)

        # Vehicle parameters
        self.vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]
        self.brake_deadband = kwargs["brake_deadband"]
        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]
        self.wheel_radius = kwargs["wheel_radius"]  

        # Timestamp
        self.last_time = rospy.get_time()

    def control(self, *args, **kwargs):
        
        if not kwargs["dbw_enabled"]:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # Extracting data
        v_ref = kwargs["twist_cmd"].twist.linear.x
        omega_ref = kwargs["twist_cmd"].twist.angular.z
        v = kwargs["current_velocity"]
        dt = kwargs["dt"]
        
        # Updating throttle/brake
        dv = self.vel_lpf.filt(v_ref - v)
        throttle = self.throttle_controller.step(dv, dt)
        if throttle > 0:
            brake = 0
        else:
            decel = -throttle
            throttle = 0
            if decel < self.brake_deadband:
                decel = 0.
            
            brake = decel * self.wheel_radius * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)
        
        steering = self.yaw_controller.get_steering(v_ref, omega_ref, v)

        return throttle, brake, steering
