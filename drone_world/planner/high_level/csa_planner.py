import csv
import time
import re
from drone_world.drone_world import DroneWorld
from drone_world.planner.high_level.population_search.crow_search import CrowSearch
from drone_world.planner.low_level.tabu_planner import TabuPlanner

class CrowSearchPlanner(object):
    """High level planner use crow search algorithm (CSA).

    CSA is used to generate optimal location of blocks in the world. In addition, it is responsible
    for defining what drone actions should occur at each location. It is NOT responsible for move
    the drone to the desired location. That responsibility is in the low-level planner. CSA planner
    will use the low level Tabu planner for this.
    """

    def __init__(self, world):
        """Set the drone world for the CSA planner
        :param world: Drone world
        """
        if not isinstance(world, DroneWorld):
            raise TypeError("World must be of type DroneWorld")
        self.drone_world = world
        self.raw_objects = []
        self.used_blocks = []
        self.unused_blocks = []
        self.goal_objectives = []
        self.dump_objectives = []

        # Metric counters
        self.runtime = None
        self.moves = None

    def get_runtime(self):
        return self.runtime

    def get_moves(self):
        return self.drone_world.get_drone_move_counter()

    def initialize(self, filename):
        """Read in a list of objectives from a file.
        :param filename: File to be read in
        """
        with open(filename, "rt") as csv_file:
            reader = csv.reader(csv_file, delimiter=" ")
            for row in reader:
                self.raw_objects.append((int(row[0]), int(row[1]), int(row[2]), str(row[3])))

    def _parse_objects(self):
        """Parse the raw objects in goal and dump objects.

        A goal object is a block which must be used to complete the objective. A dump objective
        is a block which must be moved (dumped) to another location in order to uncover a goal
        objective block. In addition, a list of used and unused blocks may be maintained.

        An objective is broken up into two components:
        1. Attach component plus (x, y, z) location
        2. Release component plus (x, y, z) location

        The attach component means the drone should move to the desired location and perform an
        attach operation. The release component means the drone should move to the desired location
        and release the block. If the release component is None (NULL), the drone should treat the
        attach component as only a move operation instead.

        NOTE: This is where CSA should be implemented.
        """

        # Assume that the goals do not contain a question mark
        # TODO: Fix the above assumption

        # TODO: Uncover blocks
        # TODO: Swap incorrect color blocks

        # Separate block from drone goal
        block_goals = []
        for item in self.raw_objects:
            x, y, z, color = item
            if not re.search("drone", color, re.IGNORECASE):
                block_goals.append((color, x, y, z))

        # Run crow search
        csa = CrowSearch(self.drone_world.state(), block_goals, self.drone_world)
        fitness, best = csa.run()

        self.goal_objectives = csa.get_actions()

        return


    def _run_objective(self, objective):
        """Run a single objective to completion
        :param objective: Attach and release objective
        """
        attach_component = objective[0]
        release_component = objective[1]
        if not attach_component:
            raise RuntimeError("No attach/move component with the dump objective")

        # Perform attach component of the objective...
        # Set max_iters to zero... this will cause the tabu search to run forever unless it
        # finds the goal state.
        # Set the Tabu structure to a reasonable size to help navigate obstacles
        x, y, z = int(attach_component[0]), int(attach_component[1]), int(attach_component[2])
        attach_tabu_search = TabuPlanner(x, y, z, self.drone_world, mem_limit=100, max_iters=0)
        attach_tabu_search.run()
        if not release_component:
            return
        self.drone_world.attach()

        # Perform the release component
        x, y, z = int(release_component[0]), int(release_component[1]), int(release_component[2])
        release_tabu_search = TabuPlanner(x, y, z, self.drone_world, mem_limit=100, max_iters=0)
        release_tabu_search.run()
        self.drone_world.release()

    def run(self):
        """Run the entire planner.

        Running the entire planner consists of executing all the dump and goal objectives.
        All dump objectives should be ran first.
        """

        start_time = time.time()
        self._parse_objects()

        # Run the dump objectives
        for dump_objective in self.dump_objectives:
            self._run_objective(dump_objective)

        # Run the goal objectives
        for goal_objective in self.goal_objectives:
            self._run_objective(goal_objective)

        # Goal objective should now be complete
        self.runtime = time.time() - start_time
