import random
import math
from abc import ABCMeta, abstractmethod
from ..object.drone_world_object import DroneWorldObjectId

class PopulationAlgorithm(object):
    __metaclass__ = ABCMeta

    def __init__(self, drone_pos, blocks, goal, debug=False):
        """Abstract tower planner class.
        NOTE - Covered blocks will not be added into solution.
        NOTE - Tower location will be built at the current x, y, z location of the drone
        :param drone_pos: Current (X, Y, Z) location of the drone.
        :param blocks: List of all blocks in the world.
        :param goal: List of block colors and location to accomplish the goal.
        :param debug: Extra debug statements.
        """

        self.drone_pos = drone_pos
        self.blocks = blocks
        self.goal = goal
        self.debug = debug
        self.red_blocks = []
        self.green_blocks = []
        self.blue_blocks = []
        self.yellow_blocks = []
        self._organize_blocks()


    def _organize_blocks(self):
        """Organize blocks into 4 different colors.
        """
        for block in self.blocks:
            obj_id, x, y, z = block
            if obj_id == DroneWorldObjectId.RED:
                self.red_blocks.append(block)
            elif obj_id == DroneWorldObjectId.GREEN:
                self.green_blocks.append(block)
            elif obj_id == DroneWorldObjectId.BLUE:
                self.blue_blocks.append(block)
            elif obj_id == DroneWorldObjectId.YELLOW:
                self.yellow_blocks.append(block)

    def _generate_random_solution(self):
        """Create a solution with random blocks.
        """
        solution = []
        for block in self.goal:
            found = False
            obj_id, x, y, z = block
            while not found:
                if obj_id == DroneWorldObjectId.RED:
                    red_b = random.choice(self.red_blocks)
                    if red_b not in solution:
                        solution.append(red_b)
                        found = True
                    else:
                        continue
                elif obj_id == DroneWorldObjectId.GREEN:
                    green_b = random.choice(self.green_blocks)
                    if green_b not in solution:
                        solution.append(green_b)
                        found = True
                    else:
                        continue
                elif obj_id == DroneWorldObjectId.BLUE:
                    blue_b = random.choice(self.blue_blocks)
                    if blue_b not in solution:
                        solution.append(blue_b)
                        found = True
                    else:
                        continue
                elif obj_id == DroneWorldObjectId.YELLOW:
                    yellow_b = random.choice(self.yellow_blocks)
                    if yellow_b not in solution:
                        solution.append(yellow_b)
                        found = True
                    else:
                        continue
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

        # The distance from drone to the first block in the list
        drone_x, drone_y, drone_z = self.drone_pos

        fitness = 0.0
        for i in range(0, len(solution)):
            obj_id, x, y, z = solution[i]

            # Typically drone_y should be equal to zero
            fitness += self._distance(x, y, z, drone_x, drone_y + i, drone_z) #Drone current location to block
            fitness += self._distance(x, y, z, drone_x, drone_y + i + 1, drone_z)  # Block to goal location
        return fitness

    @abstractmethod
    def run(self):
        pass