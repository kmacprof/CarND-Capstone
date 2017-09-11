import rospy

from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# TODO: Tune these.
SPEED_KP = 0.5
SPEED_KI = 0.005
SPEED_KD = 0.05

YAW_KP = 1.0
YAW_KI = 0.001
YAW_KD = 0.001

# Watching `rostopic echo /vehicle/*_report`:
# Throttle: ranges from 0 to 1
# Brake: ranges from 0 to 20,000
# Steering: left/right ranges over -/+ 0.44724


class Controller(object):
    def __init__(self):
        self.speed_pid = PID(SPEED_KP, SPEED_KI, SPEED_KD)
        self.yaw_pid = PID(YAW_KP, YAW_KI, YAW_KD)
        self.enabled = False
        self.i = 0

    def disable(self):
        print('disabled')
        self.enabled = False

    def enable(self):
        print('enabled')
        self.enabled = True
        self.twist_cmd = None
        self.current_velocity = None
        self.speed_pid.reset()
        self.yaw_pid.reset()
        self.t = rospy.get_rostime()

    def ready(self):
        return self.enabled and \
            self.twist_cmd is not None and \
            self.current_velocity is not None

    def control(self):
        # These appear to be m/s and rad/s.
        target_speed = self.twist_cmd.twist.linear.x
        target_yaw_rate = self.twist_cmd.twist.angular.z
        current_speed = self.current_velocity.twist.linear.x
        current_yaw_rate = self.current_velocity.twist.angular.z

        t = rospy.get_rostime()
        dt = (t - self.t).to_sec()

        speed_control = self.speed_pid.step(target_speed - current_speed, dt)
        yaw_control = self.yaw_pid.step(target_yaw_rate - current_yaw_rate, dt)

        if speed_control >= 0:
            throttle = speed_control
            brake = 0
        else:
            throttle = 0
            brake = -speed_control

        if self.i % 30 == 0:
            print('speed err', target_speed - current_speed,
                  'speed ctrl', speed_control,
                  'yaw err', target_yaw_rate - current_yaw_rate,
                  'yaw ctrl', yaw_control)
        self.i += 1

        self.t = t

        return throttle, brake, yaw_control
