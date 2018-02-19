import copy
import math
import time
import random

from drone_world.drone_world import DroneWorld
from drone_world.population_search.crow import CrowSearch
from drone_world.local_search.tabu import TabuSearch
from drone_world.local_search.node import Node
from drone_world.object.drone_world_object import DroneWorldObjectId

class TabuTowerPlannerRunner(object):
    def __init__(self, planner, drone_world, x=0, z=0, debug=True):
        # Verify arguments
        if not isinstance(drone_world, DroneWorld):
            raise TypeError("World must be a drone world type")
        if x < drone_world.x_min or x > drone_world.x_max:
            raise ValueError("X location outside the world")
        if z < drone_world.z_min or z > drone_world.z_max:
            raise ValueError("Z location outside the world")
        if not isinstance(planner, list):
            raise TypeError("Planner must a list object")

        self._planner = planner
        self._drone_world = drone_world
        self._x = x
        self._z = z
        self._cur_height = 0
        self._debug = debug
        self.moves = 0
        self.start_time = None
        self.end_time = None
    
    @property
    def runtime(self):
        return self.end_time - self.start_time

    def _generate_attach_goal(self, state):
        """Generate an attach goal.
        """
        obj_id, x, y, z = state
        if obj_id == DroneWorldObjectId.DRONE or x == self._x and z == self._z:
            raise RuntimeError("Invalid goal position")
        else:
            if not self._drone_world.verify_world_bounds(x, y + 1, z):
                raise RuntimeError("Goal position cannot be achieved")
            return x, y + 1, z

    def _generate_release_goal(self):
        """Generate a release goal.
        """
        if not self._drone_world.verify_world_bounds(self._x, self._cur_height + 1, self._z):
            raise RuntimeError(
                "Goal position cannot be achieved: {}".format((self._x, self._cur_height + 1, self._z)))
        return self._x, self._cur_height + 1, self._z

    def run(self):
        self._cur_height = 0

        self.start_time = time.time()
        for block in self._planner:

            # Generate an attach goal
            x, y, z = self._generate_attach_goal(block)
            attach_goal_node = DroneWorldGoal.generate_search_node(x, y, z, self._drone_world)

            # Run Tabu local_search
            tabu = TabuSearch(attach_goal_node, 100)
            solution = tabu.run()

            # Update drone world with results
            actions = solution.get_actions()
            for action in actions:
                self.moves += 1
                x, y, z = action
                self._drone_world.move(x, y, z)

            # Attach to the block
            if self._debug:
                print("Drone attach location: {}".format(self._drone_world.get_drone_location()))
            self._drone_world.attach()

            # Generate goal to release the block
            x, y, z = self._generate_release_goal()
            release_goal_node = DroneWorldGoal.generate_search_node(x, y, z, self._drone_world)

            # Run Tabu local_search
            tabu = TabuSearch(release_goal_node, 100)
            solution = tabu.run()

            # Update drone world with results
            actions = solution.get_actions()
            for action in actions:
                self.moves += 1
                x, y, z = action
                self._drone_world.move(x, y, z)

            # Release the block
            if self._debug:
                print("Drone release location: {}".format(self._drone_world.get_drone_location()))
            self._drone_world.release()

            # Increment stack height
            self._cur_height += 1

        self.end_time = time.time()
        return self.moves

class TowerPlannerCrow(object):
    def __init__(self, world, height=30):
        """Construct a tower at the given (x, y, z) location.
        """
        if not isinstance(world, DroneWorld):
            raise TypeError("World object must be of type DroneWorld")
        if height > world.y_max-1:
            raise ValueError("Caannot build the tower with height ", height)
        self.height = height # height of the goal tower
        self.world = world
        self.start_time = None
        self.end_time = None
        self.drone_pos = None
        self.blocks = []

    @property
    def runtime(self):
        return self.end_time - self.start_time

    def _initialize_planner(self):
        states = self.world.state()
        for state in states:
            obj_id, x, y, z = state
            if obj_id == DroneWorldObjectId.DRONE:
                self.drone_pos = x, y, z
            else:
                self.blocks.append(state)

    def _generate_goal(self):
        if not len(self.blocks) >= self.height:
            raise RuntimeError("not enought blocks in the world")
        goal_blocks = random.sample(self.blocks, self.height)
        return goal_blocks

    def run(self):
        # Enter
        self.start_time = time.time()

        self._initialize_planner()
        goal = self._generate_goal() # randomly generated goal

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