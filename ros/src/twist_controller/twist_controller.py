
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
from geometry_msgs.msg import TwistStamped
import math
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                                     wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        min_speed_yaw = 0.0
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed_yaw, max_lat_accel, max_steer_angle)

        self.pid = PID(0.2, 0.001, 3.5, decel_limit, accel_limit)

        self.error = 0.0
        self.previous_time = rospy.Time.now()
        # self.low_pass = LowPassFilter(tau, ts)

    def average_from_vector(self, vec):
        return (vec.x * vec.x + vec.y * vec.y + vec.z*vec.z) ** (1. / 3)

    def control(self, proposed_linear, proposed_angular, current_linear, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # tw = TwistStamped()
        # tw.twist.linear.x
        rospy.logwarn("Controlling")
        rospy.logwarn("dbw_enabled: %s", str(dbw_enabled))

        if dbw_enabled.data is True:
            rospy.logwarn("enabled is true")
            steer = self.yaw_controller.get_steering(self.average_from_vector(proposed_linear),
                                                     self.average_from_vector(proposed_angular),
                                                     self.average_from_vector(current_linear))
            rospy.logwarn("steer: %s", str(steer))

            current_time = rospy.Time.now()
            sample_time = (current_time - self.previous_time).to_sec()
            self.error = self.average_from_vector(proposed_linear) - self.average_from_vector(current_linear)
            acceleration = self.pid.step(self.error, sample_time)
            self.previous_time = current_time
            rospy.logwarn("Acceleration: %s ", acceleration)
            rospy.logwarn("Error: %s, Sample time: %s", str(self.error), str(sample_time))
            rospy.logwarn("Proposed Speed: %s, Current Speed: %s", str(self.average_from_vector(proposed_linear)),
                          str(self.average_from_vector(current_linear)))

            if (acceleration > 0):
                throttle = acceleration
                brake = 0.0
                rospy.logwarn("Accelerating")
            else :
                brake = - acceleration
                throttle = 0.0
                rospy.logwarn("Braking")

            return throttle, brake, steer

        else:
            self.error = 0.0
            # Return throttle, brake, steer
            return 1., 0., 0.


