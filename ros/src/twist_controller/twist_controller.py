from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy


class Controller(object):
    def __init__(self, **kwargs):
        yaw_contoller_min_speed = 1.0
        self.yaw_controller = YawController(kwargs['wheel_base'],
                                            kwargs['steer_ratio'],
                                            yaw_contoller_min_speed,
                                            kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'])

        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.brake_deadband = kwargs['brake_deadband']
        self.wheel_radius = kwargs['wheel_radius']

        self.prev_time = rospy.get_time()

        pid_p = 0.5
        pid_i = 0.0005
        pid_d = 0.1
        self.pid_controller = PID(pid_p, pid_i, pid_d, self.decel_limit, self.accel_limit)
        self.lpf = LowPassFilter(tau=0.1, ts=0.2)

        GAS_DENSITY = 2.858
        total_mass = kwargs['vehicle_mass'] + kwargs['fuel_capacity'] * GAS_DENSITY
        self.torque = total_mass * kwargs['wheel_radius']

    def control(self, target_velocity_linear_x, target_velocity_angular_z,
                current_velocity_linear_x, current_velocity_angular_z,
                dbw_enabled, time_delta):
        if not dbw_enabled:
            self.pid_controller.reset()
            return 0.0, 0.0, 0.0

        linear_x_delta = target_velocity_linear_x - current_velocity_linear_x

        throttle = self.pid_controller.step(linear_x_delta, time_delta)

        throttle = self.lpf.filt(throttle)

        brake = 0.0

        if throttle < 0.0:
            if -throttle > self.brake_deadband:
                brake = -throttle * self.torque
            throttle = 0.0

        steer = self.yaw_controller.get_steering(target_velocity_linear_x, target_velocity_angular_z,
                                                 current_velocity_linear_x)
        return throttle, brake, steer
