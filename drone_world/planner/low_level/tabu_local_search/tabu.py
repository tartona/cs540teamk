from drone_world.planner.low_level.local_search_goal.node import Node

class TabuSearch(object):
    """Generic Tabu local search given a Node object and a memory limit.
    """

    def __init__(self, init_node, short_mem_limit, max_iters=0):
        """Initialize a Tabu search local search.
        :param init_node: Starting node object.
        :param short_mem_limit: Depth of the short term Tabu memory
        :param max_iters: Number of iterations to run before giving up. Value of zero means forever.
        """
        if not isinstance(init_node, Node):
            raise ValueError("init_node must be a Node object")
        self.s_best = init_node
        self.short_mem_limit = short_mem_limit
        self.max_iters = max_iters

    def run(self):
        """Run a Tabu tabu_local_search.
         Note that this Tabu tabu_local_search is based on lower cost meaning a lower value is better.
         :return On if max_iters is not succeeded, the best solution. Else, None.
        """

        # Set best candidate to the init_node
        best_candidate = self.s_best

        # Setup tabu memory structures
        tabu_short_term_mem = []

        # Add init_node to tabu structures
        tabu_short_term_mem.append(best_candidate)

        # Loop through until s_best (which is a Node) is successful
        iterations = 0
        while not self.s_best.is_goal_met():

            # Get all the neighbors
            neighbors = best_candidate.expand()

            # Change the best_candidate to head of neighbors
            best_candidate = neighbors.pop(0)

            # Search through remaining neighbors looking for a better candidate
            for s_candidate in neighbors:
                if s_candidate not in tabu_short_term_mem and s_candidate < best_candidate:
                    best_candidate = s_candidate

            # Is best_candidate is better than s_best, adjust s_best to best_candidate
            if best_candidate < self.s_best:
                self.s_best = best_candidate

            # Put best_candidate on Tabu list as to not revisit it within a defined limit
            tabu_short_term_mem.append(best_candidate)

            # Check to see if values need to be popped of Tabu list
            if len(tabu_short_term_mem) > self.short_mem_limit:
                tabu_short_term_mem.pop(0)

            # Up the iteration counters
            iterations += 1

            # Early termination if max iterations is exceeded. In addition, return a None object
            # which signifies a solution could not be found
            if iterations == self.max_iters:
                return None

        # Return the best solution
        return self.s_best