import copy
import math
from drone_world.planner.low_level.local_search_goal.node import Node
from drone_world.drone_world import DroneWorld

class DroneWorldGoal(object):
    @staticmethod
    def generate_search_node(goal_x, goal_y, goal_z, world):
        """Generate a Node based of the world and a (x, y, z) location.
        Note that world will be copied meaning that the reference provided will not be updated.
        """
        if not isinstance(world, DroneWorld):
            raise TypeError("World object must be of type DroneWorld")
        world_copy = DroneWorldGoal(goal_x, goal_y, goal_z, copy.deepcopy(world))
        return Node(world_copy , None, None, 0)

    def __init__(self, goal_x, goal_y, goal_z, drone_world):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_z = goal_z
        self.drone_world = drone_world

        # Set the current drone location
        self.drone_x, self.drone_y, self.drone_z = self.drone_world.get_drone_location()

    def actions(self):
        """Get a list of (x, y, z) locations from the drone world
        """
        return self.drone_world.actions()

    def apply_action(self, action):
        """Apply the action to the drone world.
        """
        dx, dy, dz = action
        self.drone_world.move(dx, dy, dz)
        self.drone_x, self.drone_y, self.drone_z = self.drone_world.get_drone_location()

    def h(self):
        """Base cost on the distance between the drone and the goal
        """
        distance = math.pow(self.goal_x - self.drone_x, 2)
        distance += math.pow(self.goal_y - self.drone_y, 2)
        distance += math.pow(self.goal_z - self.drone_z, 2)
        return math.sqrt(distance)

    def is_goal_met(self):
        """If drone location is goal location, return true.
        """
        return self.goal_x == self.drone_x and self.goal_y == self.drone_y and self.goal_z == self.drone_z

    def __eq__(self, other):
        if isinstance(other, DroneWorldGoal):
            return self.drone_x == other.drone_x and self.drone_y == other.drone_y and self.drone_z == other.drone_z
        else:
            raise TypeError("Object must by of type DroneWorldGoal")