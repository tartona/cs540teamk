import time
from tower_runner import TowerRunner
from ..local_search.tabu import TabuSearch
from ..local_search.drone_world_goal import DroneWorldGoal

class TabuTowerRunner(TowerRunner):
    def __init__(self, drone_world, tower_blocks, short_term_mem_limit=100, x=0, z=0, debug=True):
        """Given a list tower blocks, move the blocks in the world to construct a tower.
        The tower MUST be buildable (drone and tower must not exceed world bounds).

        NOTE!!! At this point, blocks MUST not cover each other.
        :param drone_world: Drone world object
        :param tower_blocks: Ordered list of blocks (the plan) to be put into a tower.
        :param short_term_mem_limit: Limit to the number of entries in the short term Tabu memory.
        :param x: X-location to start tower
        :param z: Z-location to start tower
        """
        if short_term_mem_limit < 1:
            raise ValueError("Short term tabu memory limit must be greater than zero")
        self._short_term_mem_limit = short_term_mem_limit

        super(TabuTowerRunner, self).__init__(drone_world, tower_blocks, x=x, z=z, debug=debug)

    def run(self):
        # Set the start time
        self._start_time = time.time()

        self._cur_height = 0
        for block in self._tower_blocks:

            # Generate an attach goal
            x, y, z = self._generate_attach_goal(block)
            attach_goal_node = DroneWorldGoal.generate_search_node(x, y, z, self._drone_world)

            # Run Tabu local_search
            tabu = TabuSearch(attach_goal_node, self._short_term_mem_limit)
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
            tabu = TabuSearch(release_goal_node, self._short_term_mem_limit)
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

        # Set the end time
        self._end_time = time.time()

        return