import csv
import time
import re
from drone_world.drone_world import DroneWorld
from drone_world.planner.high_level.population_search.crow_search import CrowSearch
from drone_world.planner.low_level.tabu_planner import TabuPlanner
from drone_world.planner.low_level.tabu_planner import LLDumpSubroutine
from drone_world.planner.low_level.tabu_planner import LLDroneNoop
from drone_world.planner.low_level.tabu_planner import LLPathNotFound

class CrowSearchPlanner(object):
    """High level planner use crow search algorithm (CSA).

    CSA is used to generate optimal location of blocks in the world. In addition, it is responsible
    for defining what drone actions should occur at each location. It is NOT responsible for move
    the drone to the desired location. That responsibility is in the low-level planner. CSA planner
    will use the low level Tabu planner for this.
    """

    def __init__(self, world, debug=False):
        """Set the drone world for the CSA planner
        :param world: Drone world
        :param debug: Run with extra print statements
        """
        if not isinstance(world, DroneWorld):
            raise TypeError("World must be of type DroneWorld")
        self.drone_world = world
        self.raw_objects = []
        self.goal_objectives = []
        self.drone_objective = None

        # Metric counters
        self.runtime = None
        self.moves = None

        # Debug arg
        self.debug = debug

    def get_runtime(self):
        return self.runtime

    def get_moves(self):
        return self.drone_world.get_drone_move_counter()

    """First cut at a hack address wildcard entries"""
    def _parse_raw_objects(self, item):
        item_list = list(item)
        coord_index = 0
        for coordinate in item_list:
            if (coordinate == '?'):
                item_list[coord_index] = 0
            coord_index = coord_index + 1
        return tuple(item_list)



    def initialize(self, filename):
        """Read in a list of objectives from a file.
        :param filename: File to be read in
        """
        with open(filename, "rt") as csv_file:
            reader = csv.reader(csv_file, delimiter=" ")
            for row in reader:
                if '?' in row:
                    parsed_row = self._parse_raw_objects(row)
                    self.raw_objects.append((int(parsed_row[0]), int(parsed_row[1]), int(parsed_row[2]), str(parsed_row[3])))
                else:
                    self.raw_objects.append((int(row[0]), int(row[1]), int(row[2]), str(row[3])))


    def _parse_objects(self):
        """Parse the raw objects in goal objects.

        A goal object is a block which must be used to complete the objective.

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

        # Separate block from drone goal
        block_goals = []
        for item in self.raw_objects:
            x, y, z, color = item
            if not re.search("drone", color, re.IGNORECASE):
                block_goals.append((color, x, y, z))
            else:
                self.drone_objective = (x, y, z)

        # Run crow search
        csa = CrowSearch(self.drone_world, self.drone_world.state(), block_goals)
        fitness, best = csa.run()

        self.goal_objectives = csa.get_actions()
        self.goal_objectives.append((self.drone_objective, None))
        return

    def _run_objective(self, objective):
        """Run a single objective to completion
        :param objective: Attach and release objective
        """
        attach_component = objective[0]
        release_component = objective[1]

        if not attach_component:
            raise RuntimeError("No attach/move component with the dump objective")
        if attach_component == release_component:
            return

        # Perform attach component of the objective...
        # Set max_iters to zero... this will cause the tabu search to run forever unless it
        # finds the goal state.
        # Set the Tabu structure to a reasonable size to help navigate obstacles
        x, y, z = int(attach_component[0]), int(attach_component[1]), int(attach_component[2])

        if self.debug and release_component:
            print "HL: Attach command at ({} {} {})".format(x, y, z)
        elif self.debug:
            print "HL: Move command to ({} {} {})".format(x, y, z)

        attach_tabu_search = TabuPlanner(x, y, z, self.drone_world, mem_limit=100, max_iters=0, debug=self.debug)
        try:
            attach_tabu_search.run()
        except LLDroneNoop:
            pass
        if not release_component:
            return
        self.drone_world.attach()

        # Perform the release component
        x, y, z = int(release_component[0]), int(release_component[1]), int(release_component[2])

        if self.debug:
            print "HL: Release command at ({} {} {})".format(x, y, z)

        release_tabu_search = TabuPlanner(x, y, z, self.drone_world, mem_limit=100, max_iters=0, debug=self.debug)
        try:
            release_tabu_search.run()
        except (LLDumpSubroutine, LLPathNotFound):
            raise
        except LLDroneNoop:
            pass
        finally:
            self.drone_world.release()

    def run(self):
        """Run the entire planner.

        Running the entire planner consists of executing all the dump and goal objectives.
        All dump objectives should be ran first.
        """
        start_time = time.time()

        # Run the goal objectives
        success = False
        while not success:
            self._parse_objects()

            for goal_objective in self.goal_objectives:
                try:
                    self._run_objective(goal_objective)
                except (LLDumpSubroutine, LLPathNotFound) as e:
                    if self.debug:
                        print str(e)
                        print "HL: Performing a HL planner re-plan"
                    break
            else:
                # Success is set true if for loop completes (the break call is not hit)
                success = True

        # Goal objective should now be complete
        self.runtime = time.time() - start_time
