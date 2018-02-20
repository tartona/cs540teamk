from drone_world.local_search.node import Node

class TabuSearch(object):
    def __init__(self, init_node, short_mem_limit):
        if not isinstance(init_node, Node):
            raise ValueError("init_node must be a Node object")
        self.s_best = init_node
        self.short_mem_limit = short_mem_limit
        self.iterations = 0

    def run(self):
        """Run a Tabu local_search.
         Note that this Tabu local_search is based on lower cost meaning a lower value is better.
        """

        # Set best candidate to the init_node
        best_candidate = self.s_best

        # Setup tabu memory structures
        tabu_short_term_mem = []

        # Add init_node to tabu structures
        tabu_short_term_mem.append(best_candidate)

        # Loop through until s_best (which is a Node) is successful
        while not self.s_best.is_goal_met():

            # Up the iteration counters
            self.iterations += 1

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

        # Return the best solution
        return self.s_best