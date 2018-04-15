import random
import math
import copy
import re
from drone_world.drone_world import DroneWorld
from abc import ABCMeta, abstractmethod

class PopulationAlgorithm(object):
    __metaclass__ = ABCMeta

    @staticmethod
    def filter_out_drone(objects):
        """Given a set of drone world objects, filter out the drone.
        :param objects:
        :return: objects
        """
        filtered_object = []
        for item in objects:
            color, x, y, z = item
            if not re.search("black", color, re.IGNORECASE):
                filtered_object.append((color, x, y, z))
        return filtered_object

    @staticmethod
    def sort_goal_y_ascending(goal):
        """Sort a goal ascneding the y-values
        :param goal: Block vector
        :return: Sorted block vector
        """

        # Each entry in goal list is (color, x, y, z) so sort on y value
        sorted_goal = copy.deepcopy(goal)
        sorted(sorted_goal, key=lambda x: x[2])
        return sorted_goal

    @staticmethod
    def sort_goal_y_descending(goal):
        """Sort a goal descending the y-values
        :param goal: Block vector
        :return: Sorted block vector
        """

        # Each entry in goal list is (color, x, y, z) so sort on y value
        sorted_goal = PopulationAlgorithm.sort_goal_y_ascending(goal)
        sorted_goal.reverse()
        return sorted_goal

    @staticmethod
    def fill_goal(goal):
        """Give a goal vector, fill in any missing blocks.
        Blocks cannot be floating.
        :param goal: Block vector
        :return:  Filled block vector
        """

        filled_goal = PopulationAlgorithm.sort_goal_y_ascending(goal)

        # Need to pull all xyz cordinates from sorted_goal
        xyz_goal = []
        for block in filled_goal:
            color, x, y, z = block
            xyz_goal.append((x, y, z))

        # Fill in any missing pieces
        xyz_filled_goal = copy.deepcopy(xyz_goal)
        for xyz in xyz_goal:
            x, y, z = xyz

            while y > 0:
                if (x, y - 1, z) not in xyz_filled_goal:
                    xyz_filled_goal.append((x, y - 1, z))
                    filled_goal.append(("?", x, y - 1, z))
                y -= 1

        return PopulationAlgorithm.sort_goal_y_ascending(filled_goal)

    def __init__(self, drone_world, blocks, goal, debug=False):
        """Abstract tower planner class.
        NOTE - Covered blocks will not be added into solution.
        NOTE - Tower location will be built at the current x, y, z location of the drone
        :param blocks: List of all blocks in the world.
        :param goal: List of block colors and location to accomplish the goal.
        :param debug: Extra debug statements.
        """
        if not isinstance(drone_world, DroneWorld):
            raise TypeError("drone_world must be of type DroneWorld")

        self.s_best = None
        self.raw_blocks = PopulationAlgorithm.filter_out_drone(blocks)
        self.goal = PopulationAlgorithm.fill_goal(goal)
        self.debug = debug
        self.blocks = {}
        self.drone_world = drone_world
        self._organize_blocks()

    def _organize_blocks(self):
        """Organize blocks into 4 different colors.
        """
        for block in self.raw_blocks:
            color, x, y, z = block
            if color not in self.blocks:
                self.blocks[color] = []
            self.blocks[color].append(block)

    def _generate_random_solution(self):
        """Create a solution with random blocks.
        """

        # Copy the blocks and randomized them
        blocks = copy.deepcopy(self.blocks)
        for color in self.blocks:
            random.shuffle(self.blocks[color])

        # Find blocks of a specific color for end goal
        priority_blocks = {}

        for block in self.goal:
            color, x, y, z = block

            if re.search("\?", color):
                continue

            if len(blocks[color]) == 0:
                raise RuntimeError("No blocks to satisify objective... looking for color {}".format(color))

            priority_block = blocks[color].pop()
            if color not in priority_blocks:
                priority_blocks[color] = []
            priority_blocks[color].append(priority_block)

        # Now build an actual solution
        solution = []
        for block in self.goal:
            color, x, y, z = block

            solution_block = None
            if re.search("\?", color):

                # Grab a random block from the non priority list
                keys = blocks.keys()
                random.shuffle(keys)
                for color_key in blocks:
                    if len(blocks[color_key]) == 0:
                        continue
                    else:
                        solution_block = blocks[color_key].pop()
                        break
                else:
                    raise RuntimeError("Failed to find block for solution")
            else:
                solution_block = priority_blocks[color].pop()

            # Add solution block to the list
            solution.append(solution_block)

        return solution

    def _distance(self, x1, y1, z1, x2, y2, z2):
        """Calculate Euclidean distance between two points.
        """
        distance = 0.0
        distance += math.pow(x1 - x2, 2)
        distance += math.pow(y1 - y2, 2)
        distance += math.pow(z1 - z2, 2)
        return math.sqrt(distance)

    def _evaluate_fitness(self, solution):
        """Return a fitness value which is the total distance traveled
        """

        if len(solution) != len(self.goal):
            raise RuntimeError("Solution differs in length from goal")

        fitness = 0.0
        for i in range(0, len(solution)):
            color, x, y, z = solution[i]
            goal_color, goal_x, goal_y, goal_z = self.goal[i]

            # Typically drone_y should be equal to zero
            fitness += self._distance(x, y, z, goal_x, goal_y, goal_z) #Drone current location to block
        return fitness

    @abstractmethod
    def run(self):
        pass

    def get_actions(self):
        """Generate a list of drone destinations based off the best solution and the goal
        :return: list of actions
        """

        if len(self.s_best) != len(self.goal):
            raise RuntimeError("Solution differs in length from goal")

        actions = []
        for i in range(0, len(self.s_best)):
            block_color, block_x, block_y, block_z = self.s_best[i]
            goal_color, goal_x, goal_y, goal_z = self.goal[i]

            # Add one to the y locaiton since that is where the drone needs to go
            block_location = (block_x, block_y + 1, block_z)
            goal_location = (goal_x, goal_y + 1, goal_z)
            actions.append((block_location, goal_location))
        return actions