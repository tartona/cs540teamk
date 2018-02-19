import random
import math
import copy
import datetime
from drone_world.object.block import Block
from drone_world.object.drone_world_object import DroneWorldObjectId

class CrowSearch(object):
    def __init__(self, drone_pos, blocks, goal):
        self.s_best = None
        self.maxiter = 50
        self.NC = 100 # number of crows
        self.FL = 1.3 # flight length it cannot be used here
        self.AP = 0.4 # awareness probability
        self.Mem = [] # memory for food position
        self.Pos = [] # current position of crows
        self.drone_pos = drone_pos
        self.blocks = blocks
        self.goal = goal
        self.red_blocks = []
        self.green_blocks = []
        self.blue_blocks = []
        self.yellow_blocks = []

    # organize blocks into 4 different colors
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

    # create a solution with random blocks
    def _generate_random_solution(self):
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

    # randomly assign a crow to the search space
    def _initialize_crows(self):
        for i in range(0, self.NC):
            random_s = self._generate_random_solution()
            self.Mem.append(copy.deepcopy(random_s))
            self.Pos.append(copy.deepcopy(random_s))

    def _distance(self, x1, y1, z1, x2, y2, z2):
        distance = 0.0
        distance += math.pow(x1 - x2, 2)
        distance += math.pow(y1 - y2, 2)
        distance += math.pow(z1 - z2, 2)
        return math.sqrt(distance)

    # retrun a fitness value which is the total distance traveled
    def _evaluate_fitness(self, solution):
        fitness = 0

        # The distance from drone to the first block in the list
        x0, y0, z0 = self.drone_pos
        obj_id2, x2, y2, z2 = solution[0]
        fitness += self._distance(x2, y2, z2, x0, y0, z0)

        # cumulating distances between block pairs
        for i in range(0, len(solution)-1):
            obj_id1, x1, y1, z1 = solution[i]
            obj_id2, x2, y2, z2 = solution[i+1]
            fitness += self._distance(x0, y0, z0, x1, y1, z1) # block to goal
            fitness += self._distance(x2, y2, z2, x0, y0, z0) # goal to block

        obj_id1, x1, y1, z1 = solution[len(solution)-1]
        fitness += self._distance(x0, y0, z0, x2, y2, z2) # block to goal
        return fitness

    # run crow search
    def run(self):

        self._organize_blocks()
        self._initialize_crows()

        for i in range(0, self.maxiter):
            # re-assigning crows to the new position
            for crow in range(0, self.NC):
                # if a random number is greater than the awareness probability
                if random.uniform(0, 1) >= self.AP:
                    target_crow = random.choice(self.Mem) # fly toward the other crow's memory
                    flight_length = random.uniform(0, 1)*self.FL # might need to be decayed for the better result
                    replace_n = 0 # number of elements to replace

                    if flight_length <= 1.0: # local search
                        replace_n = int(flight_length*len(self.goal))
                        indexes = random.sample(range(0, len(self.goal)), replace_n)
                        # replace the element in the current crow's position to the corresponding element in the target crow's memory
                        for j in indexes:
                            # ensure no duplicates
                            if target_crow[j] not in self.Pos[crow]:
                                #if target_crow[j] == self.Pos[crow][j]: print("Error")
                                self.Pos[crow][j] = target_crow[j]

                    else: # flight_length > 1.0 global search
                        replace_n = int((flight_length-1.0)*len(self.goal))
                        if replace_n == 0:
                            replace_n = 1
                        # since the crow travels further than the target crow's memory,
                        # start from the target crow's memory instead of the current crow's position
                        self.Pos[crow] = copy.deepcopy(target_crow)
                        indexes = random.sample(range(0, len(self.goal)), replace_n)
                        # replace the element in the current crow's position to the random element
                        for j in indexes:
                            done = False
                            obj_id, x, y, z = self.Pos[crow][j]
                            while not done:
                                tmp_b = None
                                if obj_id == DroneWorldObjectId.RED:
                                    tmp_b = random.choice(self.red_blocks)
                                elif obj_id == DroneWorldObjectId.GREEN:
                                    tmp_b = random.choice(self.green_blocks)
                                elif obj_id == DroneWorldObjectId.BLUE:
                                    tmp_b = random.choice(self.blue_blocks)
                                elif obj_id == DroneWorldObjectId.YELLOW:
                                    tmp_b = random.choice(self.yellow_blocks)
                                else:
                                    continue
                                if (tmp_b != None) and (tmp_b not in self.Pos[crow]):
                                    self.Pos[crow][j] = tmp_b
                                    done = True
                                else:
                                    continue

                            # ensure no duplicates
                            if target_crow[j] not in self.Pos[crow]:
                                #if target_crow[j] == self.Pos[crow][j]: print("Error")
                                self.Pos[crow][j] = target_crow[j]

                # if a random number is lesser than the awareness probability
                else:
                    self.Pos[crow] = self._generate_random_solution()

            # If the fitness of the current position is better than the one the crow remember, then update the memory
            for crow in range(0, self.NC):
                if self._evaluate_fitness(self.Pos[crow]) < self._evaluate_fitness(self.Mem[crow]):
                    self.Mem[crow] = copy.deepcopy(self.Pos[crow])

        # Find the solution with the best fitness
        new_best = random.choice(self.Mem)
        for i in range(0, len(self.Mem)):
            # print(self._evaluate_fitness(self.Mem[i]))
            if self._evaluate_fitness(new_best) > self._evaluate_fitness(self.Mem[i]):
                new_best = self.Mem[i]

        self.s_best = new_best

        return self._evaluate_fitness(self.s_best), self.s_best