import rospy

from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# TODO: Tune these.
SPEED_KP = 0.5
SPEED_KI = 0.005
SPEED_KD = 0.05

YAW_CONTROLLER_MIN_SPEED = 1.0

# Watching `rostopic echo /vehicle/*_report`:
# Throttle: ranges from 0 to 1
# Brake: ranges from 0 to 20,000
# Steering: left/right ranges over -/+ 0.44724
# However, the car seems to actually need a much larger input --- giving it
# a value of 8 causes it to steer left roughly like a hard steer in manual
# mode.


class Controller(object):
    def __init__(self, vehicle_mass, wheel_radius,
                 wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.twist_cmd = None
        self.current_velocity = None
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius

        self.speed_pid = PID(SPEED_KP, SPEED_KI, SPEED_KD)
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, YAW_CONTROLLER_MIN_SPEED,
            max_lat_accel, max_steer_angle)

        self.enabled = False

    def disable(self):
        print('disabled')
        self.enabled = False

    def enable(self):
        print('enabled')
        self.enabled = True
        self.twist_cmd = None
        self.current_velocity = None
        self.speed_pid.reset()
        self.t = rospy.get_rostime()

    def ready(self):
        return self.enabled and \
            self.twist_cmd is not None and \
            self.current_velocity is not None

    def control(self):
        # These appear to be m/s and rad/s.
        target_speed = self.twist_cmd.twist.linear.x
        target_angular_velocity = self.twist_cmd.twist.angular.z
        current_speed = self.current_velocity.twist.linear.x

        t = rospy.get_rostime()
        dt = (t - self.t).to_sec()

        speed_error = target_speed - current_speed
        speed_control = self.speed_pid.step(speed_error, dt)
        if speed_control >= 0:
            throttle = speed_control
            brake = 0
        else:
            # If we are braking, the control is the torque to be applied by
            # the brakes. We know the braking force we want from F=ma where
            # m is the mass of the vehicle and a is the control value, so
            # multiply by the wheel radius to get the required torque.
            throttle = 0
            brake = -speed_control * self.vehicle_mass * self.wheel_radius

        rospy.logwarn_throttle(
            1,
            'target=%.2f t=%.2f b=%.2f' % (target_speed, throttle, brake))

        steer = self.yaw_controller.get_steering(
            target_speed, target_angular_velocity, current_speed)

        self.t = t

        return throttle, brake, steer
