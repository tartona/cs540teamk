import copy
import random
import math
from ..drone_world import DroneWorld
from ..object.drone_world_object import DroneWorldObjectId
from ..local_search.tabu import TabuSearch
from ..local_search.drone_world_goal import DroneWorldGoal

class TabuTowerPlannerRunner(object):
    def __init__(self, planner, drone_world, x=0, z=0, debug=True):
        # Verify arguments
        if not isinstance(drone_world, DroneWorld):
            raise TypeError("World must be a drone world type")
        if x < drone_world.x_min or x > drone_world.x_max:
            raise ValueError("X location outside the world")
        if z < drone_world.z_min or z > drone_world.z_max:
            raise ValueError("Z location outside the world")
        if not isinstance(planner, list):
            raise TypeError("Planner must a list object")

        self._planner = planner
        self._drone_world = drone_world
        self._x = x
        self._z = z
        self._cur_height = 0
        self._debug = debug

    def _generate_attach_goal(self, state):
        """Generate an attach goal.
        """
        obj_id, x, y, z = state
        if obj_id == DroneWorldObjectId.DRONE or x == self._x and z == self._z:
            raise RuntimeError("Invalid goal position")
        else:
            if not self._drone_world.verify_world_bounds(x, y + 1, z):
                raise RuntimeError("Goal position cannot be achieved")
            return x, y + 1, z

    def _generate_release_goal(self):
        """Generate a release goal.
        """
        if not self._drone_world.verify_world_bounds(self._x, self._cur_height + 1, self._z):
            raise RuntimeError(
                "Goal position cannot be achieved: {}".format((self._x, self._cur_height + 1, self._z)))
        return self._x, self._cur_height + 1, self._z

    def run(self):
        self._cur_height = 0
        for block in self._planner:

            # Generate an attach goal
            x, y, z = self._generate_attach_goal(block)
            attach_goal_node = DroneWorldGoal.generate_search_node(x, y, z, self._drone_world)

            # Run Tabu local_search
            tabu = TabuSearch(attach_goal_node, 100)
            solution = tabu.run()

            # Update drone world with results
            actions = solution.get_actions()
            for action in actions:
                x, y, z = action
                self._drone_world.move(x, y, z)

            # Attach to the block
            if self._debug:
                print "Drone attach location: {}".format(self._drone_world.get_drone_location())
            self._drone_world.attach()

            # Generate goal to release the block
            x, y, z = self._generate_release_goal()
            release_goal_node = DroneWorldGoal.generate_search_node(x, y, z, self._drone_world)

            # Run Tabu local_search
            tabu = TabuSearch(release_goal_node, 100)
            solution = tabu.run()

            # Update drone world with results
            actions = solution.get_actions()
            for action in actions:
                x, y, z = action
                self._drone_world.move(x, y, z)

            # Release the block
            if self._debug:
                print "Drone release location: {}".format(self._drone_world.get_drone_location())
            self._drone_world.release()

            # Increment stack height
            self._cur_height += 1

        return

class GeneticAlgorithmTowerPlanner(object):
    def __init__(self, drone_world, pop_size=10, mutate_prob=0.01, iters=500, x=0, z=0, height=40, pattern="rbgy",
                 debug=True):
        # Verify arguments
        if not isinstance(drone_world, DroneWorld):
            raise TypeError("World must be a drone world type")
        if x < drone_world.x_min or x > drone_world.x_max:
            raise ValueError("X location outside the world")
        if z < drone_world.z_min or z > drone_world.z_max:
            raise ValueError("Z location outside the world")
        if height > drone_world.y_max - 1:
            raise ValueError("Invalid height value")
        if pop_size < 2:
            raise ValueError("Initial population cannot be less than 2")
        if pop_size % 2 > 0:
            raise ValueError("Initial population size must be even")
        if iters < 1:
            raise ValueError("Must run at least one iteration")
        if mutate_prob < 0 or mutate_prob > 100:
            raise ValueError("Invalid muation probability")

        # Display print statements
        self._debug = debug

        # Setup class variables
        self._drone_world = drone_world
        self._x = x
        self._z = z
        self._height = height
        self._cur_height = 0
        self._red_blocks = []
        self._blue_blocks = []
        self._green_blocks = []
        self._yellow_blocks = []

        # Genetic algorithm variables
        self._pop_size = pop_size
        self._iters = iters
        self._mutate_prob = mutate_prob
        self._population = []

        # Setup the pattern
        self._pattern = []
        while len(self._pattern) != height:
            for color in list(pattern):
                if color == "r":
                    self._pattern.append(DroneWorldObjectId.RED)
                elif color == "b":
                    self._pattern.append(DroneWorldObjectId.BLUE)
                elif color == "g":
                    self._pattern.append(DroneWorldObjectId.GREEN)
                elif color == "y":
                    self._pattern.append(DroneWorldObjectId.YELLOW)

        # Process the drone world states looking for unplaced blocks
        for state in drone_world.state():
            obj_id, x, y, z = state
            if obj_id == DroneWorldObjectId.RED:
                self._red_blocks.append(state)
            elif obj_id == DroneWorldObjectId.BLUE:
                self._blue_blocks.append(state)
            elif obj_id == DroneWorldObjectId.GREEN:
                self._green_blocks.append(state)
            elif obj_id == DroneWorldObjectId.YELLOW:
                self._yellow_blocks.append(state)

    def _distance(self, x1, y1, z1, x2, y2, z2):
        distance = 0.0
        distance += math.pow(x1 - x2, 2)
        distance += math.pow(y1 - y2, 2)
        distance += math.pow(z1 - z2, 2)
        return math.sqrt(distance)

    def _fitness(self, string):
        """Calculate the fitness of a string.
        """
        fitness = 0.0
        for i in range(0, len(string)):
            obj_id, x, y, z = string[i]
            fitness += self._distance(x, y, z, self._x, i, self._z)
        return fitness

    def get_best_plan(self):
        """Get the best string (solution)
        """
        if len(self._population) == 0:
            raise RuntimeError("Population does not exist due to planner not be run")

        best_string = self._population[0]
        for i in range(1, len(self._population)):
            string = self._population[i]
            if self._fitness(string) < self._fitness(best_string):
                best_string = string
        return best_string

    def _is_covered(self, block):
        """Check if a block is coverd.
        """
        obj_id, x, y, z = block
        if self._drone_world.can_move_object(x, y + 1, z):
            return False
        return True

    def _initialize_population(self):
        """Initialize the population.
        """
        self._population = []
        while len(self._population) != self._pop_size:
            # String to add to population
            string = []

            # Need to copy blocks since we need to pop them off
            red_blocks = copy.deepcopy(self._red_blocks)
            blue_blocks = copy.deepcopy(self._blue_blocks)
            green_blocks = copy.deepcopy(self._green_blocks)
            yellow_blocks = copy.deepcopy(self._yellow_blocks)

            # Randomize the block (introduce stochastic factor for population)
            random.shuffle(red_blocks)
            random.shuffle(blue_blocks)
            random.shuffle(green_blocks)
            random.shuffle(yellow_blocks)

            # Generate the string and avoid covered blocks
            for color in self._pattern:
                if color == DroneWorldObjectId.RED:
                    block = red_blocks.pop(0)
                    while self._is_covered(block):
                        block = red_blocks.pop(0)
                    string.append(block)

                elif color == DroneWorldObjectId.BLUE:
                    block = blue_blocks.pop(0)
                    while self._is_covered(block):
                        block = blue_blocks.pop(0)
                    string.append(block)

                elif color == DroneWorldObjectId.GREEN:
                    block = green_blocks.pop(0)
                    while self._is_covered(block):
                        block = green_blocks.pop(0)
                    string.append(block)

                elif color == DroneWorldObjectId.YELLOW:
                    block = yellow_blocks.pop(0)
                    while self._is_covered(block):
                        block = yellow_blocks.pop(0)
                    string.append(block)

            # Make sure that initial population is unique
            if string not in self._population:
                self._population.append(string)
        return

    def _selection(self):
        """Perform selection using proportional selection.
        """
        selection = []

        # Setup the probabilities
        selection_probability = []
        fitness_sum = 0.0
        for string in self._population:
            fitness = self._fitness(string)
            selection_probability.append(fitness)
            fitness_sum += fitness
        for i in range(0, len(selection_probability)):
            selection_probability[i] /= fitness_sum

        # Compile the selection list
        index = 0
        while len(selection) != len(self._population):

            # The higher the probability, the better chance of selection
            if random.uniform(0, 1) <= selection_probability[index]:
                selection.append(self._population[index])

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
                while element in parent_a or self._is_covered(element):
                    obj_id, x, y, z = element
                    if obj_id == DroneWorldObjectId.RED:
                        element = self._red_blocks[random.randint(0, len(self._red_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.BLUE:
                        element = self._blue_blocks[random.randint(0, len(self._blue_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.GREEN:
                        element = self._green_blocks[random.randint(0, len(self._green_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.YELLOW:
                        element = self._yellow_blocks[random.randint(0, len(self._yellow_blocks) - 1)]
                parent_a.append(element)


            parent_b = parent_b[:crossover_point]
            for element in tmp[crossover_point:]:
                while element in parent_b or self._is_covered(element):
                    obj_id, x, y, z = element
                    if obj_id == DroneWorldObjectId.RED:
                        element = self._red_blocks[random.randint(0, len(self._red_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.BLUE:
                        element = self._blue_blocks[random.randint(0, len(self._blue_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.GREEN:
                        element = self._green_blocks[random.randint(0, len(self._green_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.YELLOW:
                        element = self._yellow_blocks[random.randint(0, len(self._yellow_blocks) - 1)]
                parent_b.append(element)

        return parent_a, parent_b

    def _mutate(self, string):
        """Mutate a string.
        """
        for i in range(0, len(string)):
            if random.uniform(0, 1) <= self._mutate_prob:
                element = string[i]
                while element in string or self._is_covered(element):
                    obj_id, x, y, z = element
                    if obj_id == DroneWorldObjectId.RED:
                        element = self._red_blocks[random.randint(0, len(self._red_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.BLUE:
                        element = self._blue_blocks[random.randint(0, len(self._blue_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.GREEN:
                        element = self._green_blocks[random.randint(0, len(self._green_blocks) - 1)]
                    elif obj_id == DroneWorldObjectId.YELLOW:
                        element = self._yellow_blocks[random.randint(0, len(self._yellow_blocks) - 1)]
                string[i] = element
        return string

    def _print_population_fitness(self):
        """Dump the fitnes of all the population.
        """
        for i in range(0, len(self._population)):
            print "String {}: {}".format(i, self._fitness(self._population[i]))

    def run(self):
        # Initialize the population
        self._initialize_population()

        # Run for N iterations
        for n in range(0, self._iters):

            # Dump population fitness if debug
            if self._debug:
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
        return