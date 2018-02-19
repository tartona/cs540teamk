import copy
import random
from ..object.drone_world_object import DroneWorldObjectId
from population_algorithm import PopulationAlgorithm

class GeneticAlgorithm(PopulationAlgorithm):
    def __init__(self, drone_pos, blocks, goal,  pop_size=10, mutate_prob=0.01, iters=500, debug=True):
        if pop_size < 2:
            raise ValueError("Initial population cannot be less than 2")
        if pop_size % 2 > 0:
            raise ValueError("Initial population size must be even")
        if iters < 1:
            raise ValueError("Must run at least one iteration")
        if mutate_prob < 0 or mutate_prob > 100:
            raise ValueError("Invalid muation probability")

        # Genetic algorithm variables
        self._pop_size = pop_size
        self._iters = iters
        self._mutate_prob = mutate_prob
        self._population = []

        super(GeneticAlgorithm, self).__init__(drone_pos, blocks, goal, debug=debug)

    def _initialize_population(self):
        """Initialize the population.
        """
        self._population = []
        while len(self._population) != self._pop_size:
            self._population.append(self._generate_random_solution())
        return


    def _selection(self):
        """Perform selection using proportional selection.
        """
        selection = []

        # Setup the probabilities
        selection_probability = []
        fitness_sum = 0.0
        for string in self._population:
            fitness = self._evaluate_fitness(string)
            selection_probability.append(fitness)
            fitness_sum += fitness
        for i in range(0, len(selection_probability)):
            selection_probability[i] /= fitness_sum

        # Compile the selection list
        index = 0
        while len(selection) != len(self._population):

            # The higher the probability, the better chance of selection
            if random.uniform(0, 1) <= selection_probability[index]:
                selection.append(copy.deepcopy(self._population[index]))

            # Move to the next index in the probability selection list checking to see if it
            # needs to be reset
            index += 1
            if index == len(selection_probability):
                index = 0
        return selection


    def _recombination(self, parent_a, parent_b):
        """Recombination using reduced surrogate crossover.
        """

        # Perform crossover if the parents are different
        if parent_a != parent_b:

            # Generate potential cross over points
            crossover_points = []
            for j in range(0, len(parent_a)):
                if parent_a[j] != parent_b[j]:
                    crossover_points.append(j)

            # Select a random crossover point
            random.shuffle(crossover_points)
            crossover_point = crossover_points.pop(0)

            tmp = copy.deepcopy(parent_a)

            parent_a = parent_a[:crossover_point]
            for element in parent_b[crossover_point:]:
                while element in parent_a or self._is_block_covered(element):
                    obj_id, x, y, z = element
                    if obj_id == DroneWorldObjectId.RED:
                        element = self.red_blocks[random.randint(0, len(self.red_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.BLUE:
                        element = self.blue_blocks[random.randint(0, len(self.blue_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.GREEN:
                        element = self.green_blocks[random.randint(0, len(self.green_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.YELLOW:
                        element = self.yellow_blocks[random.randint(0, len(self.yellow_blocks) - 1)]
                parent_a.append(element)

            parent_b = parent_b[:crossover_point]
            for element in tmp[crossover_point:]:
                while element in parent_b or self._is_block_covered(element):
                    obj_id, x, y, z = element
                    if obj_id == DroneWorldObjectId.RED:
                        element = self.red_blocks[random.randint(0, len(self.red_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.BLUE:
                        element = self.blue_blocks[random.randint(0, len(self.blue_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.GREEN:
                        element = self.green_blocks[random.randint(0, len(self.green_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.YELLOW:
                        element = self.yellow_blocks[random.randint(0, len(self.yellow_blocks) - 1)]
                parent_b.append(element)

        return parent_a, parent_b


    def _mutate(self, string):
        """Mutate a string.
        """
        for i in range(0, len(string)):
            if random.uniform(0, 1) <= self._mutate_prob:
                element = string[i]
                while element in string or self._is_block_covered(element):
                    obj_id, x, y, z = element
                    if obj_id == DroneWorldObjectId.RED:
                        element = self.red_blocks[random.randint(0, len(self.red_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.BLUE:
                        element = self.blue_blocks[random.randint(0, len(self.blue_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.GREEN:
                        element = self.green_blocks[random.randint(0, len(self.green_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.YELLOW:
                        element = self.yellow_blocks[random.randint(0, len(self.yellow_blocks) - 1)]
                string[i] = element
        return string


    def _print_population_fitness(self):
        """Dump the fitnes of all the population.
        """
        for i in range(0, len(self._population)):
            print "String {}: {}".format(i, self._evaluate_fitness(self._population[i]))

    def get_best_solution(self):
        best = self._population[0]
        for i in range(1, len(self._population)):
            potential_best = self._population[i]
            if self._evaluate_fitness(potential_best) < self._evaluate_fitness(best):
                best = potential_best
        return best

    def run(self):

        # Initialize the population
        self._initialize_population()

        # Run for N iterations
        for n in range(0, self._iters):

            # Dump population fitness if debug
            if self.debug:
                self._print_population_fitness()

            # Generate the selection
            selection = self._selection()

            # Wipe the current population since the selection phase is over
            self._population = []

            # Apply crossover via reduced surrogate combination
            for i in range(0, len(selection) / 2):
                parent_a = selection[i]
                parent_b = selection[i * 2]

                # Perform recombination
                parent_a, parent_b = self._recombination(parent_a, parent_b)

                # Time to mutate elements
                parent_a = self._mutate(parent_a)
                parent_b = self._mutate(parent_b)

                # Add to population
                self._population.append(parent_a)
                self._population.append(parent_b)

        # Find the best solution
        best = self.get_best_solution()
        fitness = self._evaluate_fitness(best)

        return fitness, best