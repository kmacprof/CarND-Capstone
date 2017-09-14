import rospy

from pid import PID, MIN_NUM, MAX_NUM
from twiddler import Twiddler

DEFAULT_DURATION = 20  # seconds per tuning epoch
DEFAULT_DELTA_FACTOR = 0.2


class TuningPID(PID):
    """
    A PID controller that tunes itself by measuring performance over an epoch
    of fixed size and then tuning the gains.
    """
    def __init__(self, kp, ki, kd,
                 mn=MIN_NUM, mx=MAX_NUM,
                 epoch_duration=DEFAULT_DURATION,
                 deltas=None):
        PID.__init__(self, kp, ki, kd, mn, mx)
        values = [kp, ki, kd]
        if deltas is None:
            deltas = [k * DEFAULT_DELTA_FACTOR for k in values]
        self.twiddler = Twiddler(values, deltas)
        self.epoch_duration = epoch_duration
        self.__start_epoch()

    def reset(self):
        """
        If we get reset, the reason is probably that we had to manually
        intervene, so make sure we tell the twiddler it was bad.
        """
        self.total_absolute_error = float("inf")
        self.__finish_epoch()
        PID.reset(self)

    def step(self, error, sample_time):
        self.total_absolute_error += abs(error) * sample_time
        self.t += sample_time
        if self.t >= self.epoch_duration:
            self.__finish_epoch()

        return PID.step(self, error, sample_time)

    def __start_epoch(self):
        self.t = 0
        self.total_absolute_error = 0

    def __finish_epoch(self):
        self.twiddler.twiddle(self.total_absolute_error)
        self.kp, self.ki, self.kd = self.twiddler.values
        rospy.loginfo("TuningPID: err=%.3f gain=%s delta=%s",
                      self.total_absolute_error,
                      self.twiddler.values,
                      self.twiddler.deltas)
        self.__start_epoch()
