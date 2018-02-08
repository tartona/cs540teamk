import random
import math
from node import Node

class SimulatedAnnealingSearch(object):
    def __init__(self, init_node, temp, rate):
        if not isinstance(init_node, Node):
            raise ValueError("init_node must be a Node object")
        if rate >= 1.0:
            raise ValueError("Temperature cannot increase (rate must be less than 1)")
        self.best = init_node
        self.temp = float(temp)
        self.rate = float(rate)
        self.iterations = 0

    def acceptance_probability(self, cur_energy, new_energy):
        if new_energy < cur_energy:
            return 1.0
        else:
            if self.temp == 0.0:
                return 0.0
            else:
                return math.exp(-(float(new_energy - cur_energy) / self.temp))

    def run(self):

        # Only run until the goal is met
        while not self.best.is_goal_met():
            # Increment the iteration counter
            self.iterations += 1

            # Get the neighbors and randomize
            neighbors = self.best.expand()
            random.shuffle(neighbors)

            while neighbors:
                # Adjust the temperature
                self.temp *= self.rate

                # Get a random neighbor
                neighbor = neighbors.pop(0)

                # Check to see if the random neighbor should be accepted
                if self.acceptance_probability(self.best.fitness, neighbor.fitness) >= random.uniform(0, 1):
                    self.best = neighbor
                    break

        # Return the best solution
        return self.best