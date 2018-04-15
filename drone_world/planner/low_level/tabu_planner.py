from drone_world.drone_world import DroneWorld
from drone_world.planner.low_level.local_search_goal.drone_world_goal import DroneWorldGoal
from drone_world.planner.low_level.local_search_goal.node import Node
from drone_world.planner.low_level.tabu_local_search.tabu import TabuSearch

class PathNotFound(Exception):
    def __init__(self, msg):
        super(PathNotFound, self).__init__(msg)

class LLDumpSubroutine(Exception):
    def __init__(self, msg):
        super(LLDumpSubroutine, self).__init__(msg)

class LLFatalError(Exception):
    def __init__(self, msg):
        super(LLFatalError, self).__init__(msg)

class LLDroneNoop(Exception):
    def __init__(self, msg):
        super(LLDroneNoop, self).__init__(msg)

class TabuPlanner(object):
    """Low level planner using Tabu local search.

    The local search algorithm used to move the Drone from current location to desired location is
    a greedy search. Collision avoidance is implemented using a Tabu structure.
    """

    def __init__(self, goal_x, goal_y, goal_z, drone_world, mem_limit=100, max_iters=0):
        """Build a Tabu search local planner.
        :param goal_x: Desired drone x location
        :param goal_y: Desired drone y location
        :param goal_z: Desired drone z location
        :param drone_world: World the drone is in
        :param mem_limit: Max short term memory limit of Tabu structure (default 100)
        :param max_iters: Max number of iterations before giving up (default 0 meaning don't give up)
        """
        if not isinstance(drone_world, DroneWorld):
            raise TypeError("Drone world must be of type DroneWorld")
        if goal_x < drone_world.x_min or goal_x > drone_world.x_max:
            raise ValueError("Out-of-bounds x goal")
        if goal_y < drone_world.y_min or goal_y > drone_world.y_max:
            raise ValueError("Out-of-bounds y goal")
        if goal_z < drone_world.z_min or goal_z > drone_world.z_max:
            raise ValueError("Out-of-bounds z goal")
        if mem_limit < 0:
            raise ValueError("Invalid memory limit")

        self.drone_world = drone_world
        self.max_iters = max_iters
        self.mem_limit = mem_limit
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_z = goal_z

    def _run_tabu_search(self, x, y, z):
        """Helper function to run tabu search
        """
        init_node = DroneWorldGoal.generate_search_node(x, y, z, self.drone_world)
        tabu_search = TabuSearch(init_node, self.mem_limit, max_iters=self.max_iters)
        return tabu_search.run()

    def _apply_solution(self, solution):
        """Helper functio to apply tabu solution to drone world
        """
        if not isinstance(solution, Node):
            raise TypeError("Expected solution to be of type Node")
        actions = solution.get_actions()
        for action in actions:
            x, y, z = action
            self.drone_world.move(x, y, z)

    def _dump_block_unattached(self, goal_x, goal_y, goal_z):
        """Attempt to move blocks covering the given x, y, z location.
        """

        # Check to see how many blocks needs to be moved
        y_height = 0
        while not self.drone_world.can_move_object(goal_x, goal_y + y_height, goal_z):
            y_height += 1

        while y_height >= 0:

            # Attempt to find a dump location
            dump_x = None
            dump_y = self.drone_world.y_max - 1
            dump_z = None
            for x in range(goal_x + 1, self.drone_world.x_max):
                for z in range(goal_z + 1, self.drone_world.z_max):
                    if self.drone_world.can_move_object(x, dump_y, z):
                        dump_x = x
                        dump_z = z
                        break
                if dump_x is not None:
                    break

            if dump_x is None or dump_z is None:
                for x in range(goal_x - 1, self.drone_world.x_min, -1):
                    for z in range(goal_z - 1, self.drone_world.z_min, -1):
                        if self.drone_world.can_move_object(x, dump_y, z):
                            dump_x = x
                            dump_z = z
                            break
                    if dump_x is not None:
                        break

            if dump_x is None or dump_z is None:
                raise LLFatalError("Could not find a dump location")

            # Run tabu search to the block to be dumped
            solution = self._run_tabu_search(goal_x, goal_y + y_height, goal_z)
            if not solution:
                raise LLFatalError("Unable to find a path")
            self._apply_solution(solution)
            self.drone_world.attach()

            # Run tabu search to dump the block
            solution = self._run_tabu_search(dump_x, dump_y, dump_z)
            if not solution:
                raise LLFatalError("Unable to find a path")
            self._apply_solution(solution)
            self.drone_world.release()

            y_height -= 1

        # Done
        return

    def _dump_block_attached(self, goal_x, goal_y, goal_z):
        """Attempt to move blocks covering the given x, y, z location but the drone is attached.
        """
        cached_location = self.drone_world.get_drone_location()
        self.drone_world.release()

        self._dump_block_unattached(goal_x, goal_y, goal_z)

        # So far the dump was a success... move back to the cached location
        solution = self._run_tabu_search(cached_location[0], cached_location[1], cached_location[2])
        if not solution:
            raise LLFatalError("Unable to find a path")
        self._apply_solution(solution)
        self.drone_world.attach()

        # Done
        return

    def _dump_block(self):
        """Run the dump block subroutine.
        """
        is_attached = self.drone_world.is_drone_attached()
        if is_attached:
            self._dump_block_attached(self.goal_x, self.goal_y, self.goal_z)
        else:
            self._dump_block_unattached(self.goal_x, self.goal_y + 1, self.goal_z)

    def run(self):
        """Run the Tabu local planner.

        If no solution is found within the max number of iterations, a PathNotFound exception
        is generated. Else, the drone will be moved to the generated solution.
        """
        if self.drone_world.get_drone_location() == (self.goal_x, self.goal_y, self.goal_z):
            raise LLDroneNoop("Drone is at the desired location")

        # Need to know if a block is attached
        is_attached = self.drone_world.is_drone_attached()

        # Check to see if drone can move to the location
        if is_attached:
            can_move = self.drone_world.can_move_object(self.goal_x, self.goal_y - 1, self.goal_z)
        else:
            can_move = self.drone_world.can_move_object(self.goal_x, self.goal_y, self.goal_z)

        # If drone cannot move to the location, need to run the dump subroutine
        if not can_move:
            self._dump_block()

        # Get the solution
        solution = self._run_tabu_search(self.goal_x, self.goal_y, self.goal_z)
        if not solution:
            raise PathNotFound("Unable to find solution within {} iterations".format(self.max_iters))
        if not isinstance(solution, Node):
            raise TypeError("Expected solution to be of type Node")

        # Apply the solution to actual drone world
        self._apply_solution(solution)

        # The dump subroutine modifies the drone world. This can/will mess up the high level
        # planner. So, raise a exception to notify the high level planner that this subroutine was
        # ran.
        if not can_move:
            raise LLDumpSubroutine("LL dump subroutine was ran")