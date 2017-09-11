import sys
import unittest

from styx_msgs.msg import Waypoint

sys.path.insert(0, '..')
from waypoint_planner import WaypointPlanner


class TestWaypointPlanner(unittest.TestCase):

    def test_find_waypoints_no_position(self):
        """
        If the vehicle's position hasn't been provided yet, return None.
        """
        planner = WaypointPlanner([])
        self.assertIsNone(planner.plan(1))


if __name__ == '__main__':
    import rostest
    rostest.unitrun(
        'waypoint_updater',
        'test_waypoint_planner', TestWaypointPlanner)
