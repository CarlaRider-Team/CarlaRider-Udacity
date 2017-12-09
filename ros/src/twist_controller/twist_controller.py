from  yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy
import math

GAS_DENSITY = 2.858

# Gains - can be tuned!
KP = 0.5
KI = 0.05
KD = 0.1



class Controller(object):
	def __init__(self, *args, **kwargs):
		self.yaw_controller = YawController(kwargs['wheel_base'], 
						kwargs['steer_ratio'],
						kwargs['min_speed'],
						kwargs['max_lat_accel'],
						kwargs['max_steer_angle'])

		self.accel_limit = kwargs['accel_limit']
		self.decel_limit = kwargs['decel_limit']
		self.min_speed = kwargs['min_speed']
		self.brake_deadband = kwargs['brake_deadband']
		self.wheel_radius = kwargs['wheel_radius']

		self.prev_time = rospy.get_time()
		self.total_mass = kwargs['vehicle_mass'] + kwargs['fuel_capacity']*GAS_DENSITY
		

		self.pid_controller = PID(KP, KI, KD, self.decel_limit, self.accel_limit)
		self.lpf = LowPassFilter(tau = 3, ts = 1)

		
	def control(self, *args, **kwargs):
		target_vel_lin_x = args[0]
		target_vel_ang_z = args[1]
		current_vel_lin_x = args[2]
		current_vel_ang_z = args[3]
		dbw_enabled = args[4]
		throttle = 0.0
		brake = 0.0

		# If not dbw, reset and return 0's
		if not dbw_enabled:
			self.pid_controller.reset()
			return 0, 0, 0


		# Steering through yaw controller
		steering = self.yaw_controller.get_steering(target_vel_lin_x, target_vel_ang_z,
							current_vel_lin_x)		



		diff_vel = target_vel_lin_x - current_vel_lin_x

		current_time = rospy.get_time()
		dt = current_time - self.prev_time if self.prev_time else 0.05
		self.prev_time = current_time

		### Not sure how to use the PID controller here, so doing manually!!!
		#acceleration = self.pid_controller.step(diff_vel, dt)
        	#acceleration = self.lpf.filt(acceleration)

        	acceleration = diff_vel / dt #accel_time
	        if acceleration > 0:
        	    acceleration = min(self.accel_limit, acceleration)
        	else:
        	    acceleration = max(self.decel_limit, acceleration)


        	if acceleration > 0.0:
            		throttle = acceleration
            		brake = 0.0
        	else:
            		throttle = 0.0
            		deceleration = -acceleration

            		if deceleration < self.brake_deadband:
                		deceleration = 0.0

            		brake = deceleration * self.total_mass * self.wheel_radius
		
		rospy.loginfo("TwistController dt %s, acceleration: %s, throttle %s, brake %s", dt, acceleration, throttle, brake)
		
		return throttle, brake, steering
