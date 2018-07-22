from __future__ import print_function
import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import numpy as np

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        
        # Controllers
        yaw_param = {}
        for key in ["wheel_base", "steer_ratio", "min_speed", "max_lat_accel","max_steer_angle"]:
            yaw_param[key] = kwargs[key]

        self.yaw_controller = YawController(**yaw_param)
        k = [x*0.462 for x in [1, 0.08, 0.006]] # PID coefficients
        output_range = [kwargs["decel_limit"], kwargs["accel_limit"]]
        pid_param = k + output_range
        self.throttle_controller = PID(*pid_param)

        # Low pass filters
        self.ts = 1./(kwargs["DBW_FREQ"]) # Sampling period
        self.tau = 2*self.ts # Cutoff frequency
        self.vel_lpf = LowPassFilter(self.tau, self.ts)

        # Vehicle parameters
        self.vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]
        self.brake_deadband = kwargs["brake_deadband"]
        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]
        self.wheel_radius = kwargs["wheel_radius"]  
        self.min_speed = kwargs["min_speed"]  

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
        if v_ref < self.min_speed:
            # Forcing a stop in case of very low target speeds
            brake = 700. 
            throttle = 0.
            steering = 0.
        else:
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
