import math


def position_distance(position0, position1):
    """
    Squared distance between two positions.
    """
    delta_x = position1.x - position0.x
    delta_y = position1.y - position0.y
    delta_z = position1.z - position0.z
    return math.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z)


class WaypointPlanner(object):

    def __init__(self, base_waypoints):
        self.base_waypoints = base_waypoints
        self.position = None

    def plan(self, num_waypoints):
        """
        For now, find the closest base waypoint and return the next
        `num_waypoints` base waypoints.

        TODO: Does it have to be the closest waypoint ahead of the vehicle, or
        is it just the closest one? Guidance seems ambiguous.
        """
        if self.position is None:
            return None

        min_distance = float('inf')
        min_index = 0
        for index in range(len(self.base_waypoints)):
            waypoint_position = self.base_waypoints[index].pose.pose.position
            distance = position_distance(self.position, waypoint_position)
            if distance < min_distance:
                min_distance = distance
                min_index = index

        # TODO: handle wraparound
        max_index = min_index + num_waypoints
        return self.base_waypoints[min_index:max_index]
