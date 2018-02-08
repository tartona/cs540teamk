import math
import copy
import time
from drone_world import DroneWorld
from drone_world_object import DroneWorldObjectId
from search.node import Node
from search.tabu import TabuSearch
from search.simulated_annealing import SimulatedAnnealingSearch

class TowerPlannerSimulateAnnealing(object):
    def __init__(self, x, y, z, world):
        """Construct a tower at the given (x, y, z) location.
        """
        if not isinstance(world, DroneWorld):
            raise TypeError("World object must be of type DroneWorld")
        self.goal_x = x
        self.goal_y = y
        self.goal_z = z
        self.height = 0
        self.world = world
        self.start_time = None
        self.end_time = None
        self.moves = 0

    @property
    def runtime(self):
        return self.end_time - self.start_time

    def generate_attach_goal(self):
        states = self.world.state()
        for state in states:
            obj_id, x, y, z = state
            if obj_id == DroneWorldObjectId.DRONE or x == self.goal_x and z == self.goal_z:
                continue
            else:
                if not self.world.verify_world_bounds(x, y + 1, z):
                    raise RuntimeError("Goal position cannot be achieved")
                return x, y + 1, z

    def generate_release_goal(self):
        if not self.world.verify_world_bounds(self.goal_x, self.height + 2, self.goal_z):
            raise RuntimeError("Goal position cannot be achieved")
        return self.goal_x, self.height + 1, self.goal_z

    def run(self):
        self.start_time = time.time()
        while self.height != self.goal_y:

            # Generate an attach goal
            x, y, z = self.generate_attach_goal()
            attach_goal_node = DroneWorldGoal.generate_search_node(x, y, z, self.world)

            # Run simulate annealing search
            simulate_annealing = SimulatedAnnealingSearch(attach_goal_node, 1000.0, 0.01)
            solution = simulate_annealing.run()

            # Update drone world with results
            actions = solution.get_actions()
            for action in actions:
                x, y, z = action
                self.world.move(x, y, z)
            self.moves += len(actions)

            # Attach to the block
            self.world.attach()

            # Generate goal to release the block
            x, y, z = self.generate_release_goal()
            release_goal_node = DroneWorldGoal.generate_search_node(x, y, z, self.world)

            # Run Tabu search
            simulate_annealing = SimulatedAnnealingSearch(release_goal_node, 1000.0, 0.01)
            solution = simulate_annealing.run()

            # Update drone world with results
            actions = solution.get_actions()
            for action in actions:
                x, y, z = action
                self.world.move(x, y, z)
            self.moves += len(actions)

            # Release the block
            self.world.release()

            # Increment stack height
            self.height += 1

        # Exit
        self.end_time = time.time()
        return

class TowerPlannerTabu(object):
    def __init__(self, x, y, z, world):
        """Construct a tower at the given (x, y, z) location.
        """
        if not isinstance(world, DroneWorld):
            raise TypeError("World object must be of type DroneWorld")
        self.goal_x = x
        self.goal_y = y
        self.goal_z = z
        self.height = 0
        self.world = world
        self.start_time = None
        self.end_time = None
        self.moves = 0

    @property
    def runtime(self):
        return self.end_time - self.start_time

    def generate_attach_goal(self):
        states = self.world.state()
        for state in states:
            obj_id, x, y, z = state
            if obj_id == DroneWorldObjectId.DRONE or x == self.goal_x and z == self.goal_z:
                continue
            else:
                if not self.world.verify_world_bounds(x, y + 1, z):
                    raise RuntimeError("Goal position cannot be achieved")
                return x, y + 1, z

    def generate_release_goal(self):
        if not self.world.verify_world_bounds(self.goal_x, self.height + 2, self.goal_z):
            raise RuntimeError("Goal position cannot be achieved")
        return self.goal_x, self.height + 1, self.goal_z

    def run(self):
        self.start_time = time.time()
        while self.height != self.goal_y:

            # Generate an attach goal
            x, y, z = self.generate_attach_goal()
            attach_goal_node = DroneWorldGoal.generate_search_node(x, y, z, self.world)

            # Run Tabu search
            tabu = TabuSearch(attach_goal_node, 5)
            solution = tabu.run()

            # Update drone world with results
            actions = solution.get_actions()
            for action in actions:
                x, y, z = action
                self.world.move(x, y, z)
            self.moves += len(actions)

            # Attach to the block
            self.world.attach()

            # Generate goal to release the block
            x, y, z = self.generate_release_goal()
            release_goal_node = DroneWorldGoal.generate_search_node(x, y, z, self.world)

            # Run Tabu search
            tabu = TabuSearch(release_goal_node, 5)
            solution = tabu.run()

            # Update drone world with results
            actions = solution.get_actions()
            for action in actions:
                x, y, z = action
                self.world.move(x, y, z)
            self.moves += len(actions)

            # Release the block
            self.world.release()

            # Increment stack height
            self.height += 1

        # Exit
        self.end_time = time.time()
        return


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