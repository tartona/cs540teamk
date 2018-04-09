import copy
import math
import random

from ..object.drone_world_object import DroneWorldObjectId


class DroneWorldNest(object):
    def __init__(self, blocks, goal, drone_pos):
        self.blocks = blocks
        self.red_blocks = []
        self.green_blocks = []
        self.blue_blocks = []
        self.yellow_blocks = []
        self.goal = goal
        self.drone_pos = drone_pos

        self._organize_blocks()

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
        self.solution = solution
        self.fitness = self.evaluate_fitness(self.solution)

    def _organize_blocks(self):
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

    def _distance(self, x1, y1, z1, x2, y2, z2):
        distance = 0.0
        distance += math.pow(x1 - x2, 2)
        distance += math.pow(y1 - y2, 2)
        distance += math.pow(z1 - z2, 2)
        return math.sqrt(distance)

    # return a fitness value which is the total distance traveled
    def evaluate_fitness(self, solution):
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



class CuckooSearch(object):
    def __init__(self, drone_pos, blocks, goal):
        # World Constructs
        self.drone_pos = drone_pos
        self.blocks = blocks
        self.goal = goal
        self.red_blocks = []
        self.green_blocks = []
        self.blue_blocks = []
        self.yellow_blocks = []
        # Cuckoo Search Constructs
        
        #new_nest = DroneWorldNest(x, blocks)
        #self.s_best = new_nest
        self.iterations = 0
        self.NP = 20 # number of nests
        self.keep = 2  # elitism factor... how many will we keep
        self.MaxGen = 40  # maximum generations
        self.generation = 1
        self.nests = []
        self.KEEP_best_cuckoos = []
        self.moves = []
        self.s_best = DroneWorldNest(self.blocks, self.goal, self.drone_pos)

        """

                Levy Functon provided Mark N. Read -- 
                This program is free software: you can redistribute it and/or modify
                it under the terms of the GNU General Public License as published by
                the Free Software Foundation, either version 3 of the License, or
                (at your option) any later version.

                This program is distributed in the hope that it will be useful,
                but WITHOUT ANY WARRANTY; without even the implied warranty of
                MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

                You should have received a copy of the GNU General Public License
                along with this program.  If not, see <http://www.gnu.org/licenses/>.

                Mark N. Read, 2016.
        """
    def levy_function(self):
       mu = 2.0  # Mu is based on powers & can vary between 1 and 3
       ''' From the Harris Nature paper. '''
       # uniform distribution, in range [-0.5pi, 0.5pi]
       x = random.uniform(-0.5 * math.pi, 0.5 * math.pi)

       # y has a unit exponential distribution.
       y = -math.log(random.uniform(0.0, 1.0))

       a = math.sin((mu - 1.0) * x) / (math.pow(math.cos(x), (1.0 / (mu - 1.0))))
       b = math.pow((math.cos((2.0 - mu) * x) / y), ((2.0 - mu) / (mu - 1.0)))

       z = a * b
       return z

        # Use Levy Flight (vs random)
    def levy_flight(self, nest):

        solution = []
        new_nest = copy.deepcopy(nest)
        for block in nest.solution:
            obj_id, x, y, z = block
            if obj_id == DroneWorldObjectId.RED:
                x = int(abs(self.levy_function()* len(new_nest.red_blocks)))
                if x > len(new_nest.red_blocks) - 1:
                    x = x % len(new_nest.red_blocks)
                solution.append(new_nest.red_blocks[x])
            if obj_id == DroneWorldObjectId.GREEN:
                x = int(abs(self.levy_function()*len(new_nest.green_blocks)))
                if x > len(new_nest.green_blocks) - 1:
                    x = x % len(new_nest.green_blocks)
                solution.append(new_nest.green_blocks[x])
            if obj_id == DroneWorldObjectId.BLUE:
                x = int(abs(self.levy_function()*len(new_nest.blue_blocks)))
                if x > len(new_nest.blue_blocks) - 1:
                    x = x % len(new_nest.blue_blocks)
                solution.append(new_nest.blue_blocks[x])
            if obj_id == DroneWorldObjectId.YELLOW:
                x = int(abs(self.levy_function()*len(new_nest.yellow_blocks)))
                if x > len(new_nest.yellow_blocks) - 1:
                    x = x % len(new_nest.yellow_blocks)
                solution.append(new_nest.yellow_blocks[x])
        fitness = new_nest.evaluate_fitness(solution)
        new_nest.solution = copy.deepcopy(solution)
        return new_nest



    def initialize_nests(self):
        for i in range(0, self.NP):
            new_nest = DroneWorldNest(self.blocks, self.goal, self.drone_pos)


            self.nests.append(new_nest)

    def sort_population(self):
        self.nests.sort(key=lambda node: node.fitness, reverse=False)


    def keep_best_cuckoos(self):
        for i in range(0, self.keep):
            new_nest = copy.deepcopy(self.nests[i])
            self.KEEP_best_cuckoos.append(new_nest)
        self.KEEP_best_cuckoos.sort(key=lambda node: node.fitness, reverse=False)




    # Replace the KEEP worst cuckoos with the KEEP best cuckoos
    def replace_worst_cuckoos(self):
        for i in range(0, self.keep):
            self.nests.pop(len(self.nests) -1 )
        for i in range(0, self.keep):
            new_nest = copy.deepcopy(self.KEEP_best_cuckoos[i])
            self.nests.append(new_nest)


    # Define whether to select this cuckoo for the mutation function
    def select_cuckoo(self, cuckoo):
        v = random.randint(0, self.NP - 1)
        if v == cuckoo:
            return True
        else:
            return False







    def run(self):

        best_candidate = self.s_best

        self.initialize_nests()
        self.generation = 1

        while self.generation < self.MaxGen:
            self.sort_population()
            self.keep_best_cuckoos()
            for i in range(1, self.NP):
                if (self.select_cuckoo(i)):
                    # Have the cuckoo choose a new nest
                    nest = self.nests[i]
                    new_nest = self.levy_flight(nest)
                    if new_nest.fitness < nest.fitness:
                        self.nests.pop(i)
                        self.nests.append(new_nest)

            self.sort_population()
            self.replace_worst_cuckoos()

            self.generation = self.generation + 1
            # finally get the best solution
            self.sort_population()
            best_candidate = self.nests[0]
            if best_candidate.fitness < self.s_best.fitness:
                self.s_best = copy.deepcopy(best_candidate)

        return self.s_best.fitness, self.s_best
