import random
import math
import copy
import datetime
from drone_world.object.block import Block
from drone_world.object.drone_world_object import DroneWorldObjectId
from drone_world.population_search.population_algorithm import PopulationAlgorithm

class DiscreteCuckoo(PopulationAlgorithm):
    def __init__(self, drone_pos, blocks, goal):
        self.s_best = None
        self.maxgen = 40 # number of generation
        self.population = 20 # number of cuckoos(nests)
        self.nests = []
        self.pa = int(0.2*self.population) # percentage of nests that will be abandoned
        self.pc = int(0.6*self.population) # percentage of nests used for new eggs
        self.step_threshold = 1.8 # threshold for choosing double bridege over two opt

        super(DiscreteCuckoo, self).__init__(drone_pos, blocks, goal, False)

    # randomly assign a nests to the search space
    def _initialize_nests(self):
        for i in range(0, self.population):
            random_s = self._generate_random_solution()
            self.nests.append(copy.deepcopy(random_s))

    def _levy_flight(self, u):
        return math.pow(u, -1.0/3.0)

    def _randF(self):
        return random.uniform(0.0001, 0.9999)

    def _swap(self, solution, i, j):
        temp = solution[i]
        solution[i]=solution[j]
        solution[j]=temp

    # small movement in the search space
    def _two_opt_move(self, solution):
        new_s = copy.deepcopy(solution)
        done = False
        while not done:
            i = random.randint(0, len(new_s)-1)
            obj_id1, x1, y1, z1 = new_s[i]
            same_colored_blocks = []
            for s in new_s:
                obj_id2, x2, y2, z2 = s
                if obj_id1 == obj_id2 and new_s[i] != s:
                    same_colored_blocks.append(s)
            if len(same_colored_blocks)==0:
                continue
            else:
                self._swap(new_s, i, random.randint(0, len(same_colored_blocks)))
                done = True
        return new_s

    # large movement in the search space
    def _double_bridge_move(self, solution):
        new_s = copy.deepcopy(solution)
        for i in range(0, 4):
            self._two_opt_move(new_s)
        return new_s

    # run discrete cuckoo search
    def run(self):

        self._initialize_nests()

        self.nests.sort(key=lambda x: self._evaluate_fitness(x))

        for i in range(self.maxgen):

            # Select a solution from the top portion of the cuckoos (nests)
            for cuckoo in range(0, self.pc):
                new_egg = self.nests[random.randint(0, self.pc-1)]
                if self._levy_flight(self._randF()) > self.step_threshold:
                    new_egg = self._double_bridge_move(new_egg) # bigger step
                else:
                    new_egg = self._two_opt_move(new_egg) # small step
                target_nest = self.nests[random.randint(0, self.population-1)] # place new egg to any nests
                if self._evaluate_fitness(new_egg) < self._evaluate_fitness(target_nest):
                    target_nest = new_egg

            self.nests.sort(key=lambda x: self._evaluate_fitness(x))

            # moving around some of the bad solutions
            for j in range(self.pc, self.population-self.pa):
                self.nests[j] = self._two_opt_move(self.nests[j])

            # replace bottom portion of the nests with a random solution
            for j in range(self.population-self.pa, self.population):
                self.nests[j] = self._generate_random_solution()

            self.nests.sort(key=lambda x: self._evaluate_fitness(x))

        self.s_best = self.nests[0]

        return self._evaluate_fitness(self.s_best), self.s_best