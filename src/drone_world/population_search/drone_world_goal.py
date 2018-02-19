import copy
import math
import time
import random

from drone_world.drone_world import DroneWorld
from drone_world.population_search.crow import CrowSearch
from drone_world.object.drone_world_object import DroneWorldObjectId

class TowerPlannerCrow(object):
    def __init__(self, world, height=30):
        """Construct a tower at the given (x, y, z) location.
        """
        if not isinstance(world, DroneWorld):
            raise TypeError("World object must be of type DroneWorld")
        self.height = height
        self.world = world
        self.start_time = None
        self.end_time = None
        self.moves = 0
        self.drone_pos = None
        self.blocks = []

    @property
    def runtime(self):
        return self.end_time - self.start_time

    def initialize_planner(self):
        states = self.world.state()
        for state in states:
            obj_id, x, y, z = state
            if obj_id == DroneWorldObjectId.DRONE:
                self.drone_pos = x, y, z
            else:
                self.blocks.append(state)
    
    def generate_goal(self):
        if not len(self.blocks) >= self.height:
            raise RuntimeError("not enought blocks in the world")
        goal_blocks = random.sample(self.blocks, self.height)
        return goal_blocks

    def run(self):
        # Enter
        self.start_time = time.time()
        
        self.initialize_planner()
        goal = self.generate_goal()

        search = CrowSearch(self.drone_pos, self.blocks, goal)
        fitness, solution = search.run()

        # Exit
        self.end_time = time.time()
        return fitness, solution


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