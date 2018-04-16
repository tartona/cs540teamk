import random
import copy
from drone_world.planner.high_level.population_search.population_search import PopulationAlgorithm

class CrowSearch(PopulationAlgorithm):
    def __init__(self, blocks, goal, world):
        self.maxiter = 40
        self.NC = 20 # number of crows
        self.FL = 1.3 # flight length
        self.AP = 0.4 # awareness probability
        self.Mem = [] # memory for food position
        self.Pos = [] # current position of crows

        super(CrowSearch, self).__init__(blocks, goal, world, False)

    # randomly assign a crow to the search space
    def _initialize_crows(self):
        for i in range(0, self.NC):
            random_s = self._generate_random_solution()
            self.Mem.append(copy.deepcopy(random_s))
            self.Pos.append(copy.deepcopy(random_s))

    # run crow search
    def run(self):

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
                            color, x, y, z = self.Pos[crow][j]

                            exit_counter = 50
                            while not done and exit_counter > 0:
                                exit_counter -= 1
                                tmp_b = random.choice(self.blocks[color])
                                if (tmp_b != None) and (tmp_b not in self.Pos[crow]):
                                    self.Pos[crow][j] = tmp_b
                                    done = True
                                else:
                                    continue

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