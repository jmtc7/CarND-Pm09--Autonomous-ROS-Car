# DONE: Added dependencies
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    # DONE
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        # Steering controller
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # Throttle controller
        kp = 3
        ki = 0.1
        kd = 0.0
        mn = 0.0 # Minimum throttle value
        mx = 0.2 # Maximum throttle
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # Low pass filter (for the noise in the velocity messages)
        tau = 0.5 # Cutoff frequency = 1/(2pi*tau)
        ts = 0.02 # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        # Store the rest of relevant data (and time)
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    # DONE
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # Check if DBW is enabled
        ## It should be disabled when the car is stopped. Otherwise the integral part
        ## of the PID will accumulate error and the car will behave erratically after.
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        # Reduce noise in input velocity with the low pass filter
        current_vel = self.vel_lpf.filt(current_vel)

        # Compute appropiate steering angle
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # Compute error in velocity and update last velocity
        ## "linear_vel" is the desired one
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        # Compute ellapsed time and update last time stamp
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # Compute appropiate throttle
        throttle = self.throttle_controller.step(vel_error, sample_time)
        
        # Compute braking torque
        brake = 0.0
        # If the system is stopped and wants to keep being like that
        ## Goal velocity 0 and current velocity is very small
        if linear_vel==0.0 and current_vel<0.1:
            throttle = 0.0
            brake = 700 # Apply 700 N*m of braking torque (Carla is automatic, need brakes to be static)
        elif throttle<0.1 and vel_error<0.0:
            throttle = 0.0
            decel = max(vel_error, self.accel_limit)
            brake = abs(decel) * self.wheel_radius * self.vehicle_mass

        return throttle, brake, steering
