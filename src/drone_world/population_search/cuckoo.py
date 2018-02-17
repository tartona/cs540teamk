import random
import math
from ..local_search.node import Node


class CuckooSearch(object):
    def __init__(self, init_node, short_mem_limit):
        if not isinstance(init_node, Node):
            raise ValueError("init_node must be a Node object")
        self.s_best = init_node
        self.short_mem_limit = short_mem_limit
        self.iterations = 0

        """ Through various experiments in
                        Sect. 4.2, it was found that HMCR is set to 0.9 and PAR to
                        0.1 that can generate the optimal solutions."""
        self.HMCR = 0.9
        self.PAR = 0.1
        self.NP = 40  # number of nests
        self.keep = 2  # elitism factor... how many will we keep
        self.MaxGen = 10  # maximum generations
        self.generation = 1
        self.nests = []
        self.KEEP_best_cuckoos = []

    def initialize_nests(self, nest_locations):
        for i in range(0, self.NP):
            index = random.randint(0, len(nest_locations) - 1)
            nest = nest_locations[index]
            self.nests.append(nest)

    def sort_population(self):
        self.nests.sort(key=lambda node: node.fitness, reverse=False)

    def keep_best_cuckoos(self):
        for i in range(1, self.keep):
            self.KEEP_best_cuckoos.append(self.nests[i])
        self.KEEP_best_cuckoos.sort(key=lambda node: node.fitness, reverse=False)
        while (len(self.KEEP_best_cuckoos) > self.keep):
            self.KEEP_best_cuckoos.pop(len(self.KEEP_best_cuckoos) - 1)

    # Replace the KEEP worst cuckoos with the KEEP best cuckoos
    def replace_worst_cuckoos(self):

        for i in range(1, self.keep):
            self.nests.pop()
        for i in range(1, self.keep):
            self.nests.append(self.KEEP_best_cuckoos[i - 1])

    # Define whether to select this cuckoo for the mutation function
    def select_cuckoo(self, cuckoo):
        v = random.randint(0, self.NP - 1)
        if v == cuckoo:
            return True
        else:
            return False

    """ levy_flight_harmony_search """

    def choose_new_nest(self, cuckoo):
        xi1 = random.random()
        xi2 = random.random()
        beta = 3 / 2
        neighbors = self.s_best.expand()
        bw = len(neighbors)/2  #
        #  Levy Function from https://github.com/7ossam81/EvoloPy/blob/master/CS.py
        sigma = (math.gamma(1 + beta) * math.sin(math.pi * beta / 2) / (
            math.gamma((1 + beta) / 2) * beta *
            2 ** ((beta - 1) / 2))) ** (1 / beta)

        if xi1 < self.HMCR:
            step = cuckoo + int(sigma * random.random() * len(neighbors) / random.random() * len(neighbors))
            if xi2 < self.PAR:
                step = cuckoo + int(bw * (2 * random.random() - 1))
            if (step >= len(neighbors)):
                step = step % len(neighbors)
            new_nest = neighbors[step]
        else:
            index = random.randint(0, len(neighbors) - 1)
            new_nest = neighbors[index]
        return new_nest

    def run(self):

        # Set best candidate to the init_node
        best_candidate = self.s_best

        # Loop through until s_best (which is a Node) is successful
        while not self.s_best.is_goal_met():
            # initialize the nests from the available nesting locations
            neighbors = best_candidate.expand()
            self.initialize_nests(neighbors)
            self.generation = 1

            # evaluation of cuckoos is done implicitly via the fitness property

            while self.generation < self.MaxGen:

                # sort and pick two best cuckoos
                self.sort_population()
                self.keep_best_cuckoos()

                # Get a cuckoo randomly and replace its solution by Levy flights
                for i in range(1, self.NP):
                    if (self.select_cuckoo(i)):
                        # Have the cuckoo choose a new nest
                        new_nest = self.choose_new_nest(i)
                        self.nests.pop(i)
                        self.nests.append(new_nest)

                    # sort and replace worst cuckoos
                    self.sort_population()
                    self.replace_worst_cuckoos()

                #print self.s_best.fitness
                self.generation = self.generation + 1

            # finally get the best solution
            self.sort_population()
            best_candidate = self.nests.pop(0)
            if best_candidate < self.s_best:
                self.s_best = best_candidate
            #print self.s_best.fitness
        return self.s_best
