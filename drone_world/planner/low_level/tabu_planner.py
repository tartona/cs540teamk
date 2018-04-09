from drone_world.drone_world import DroneWorld
from drone_world.planner.low_level.local_search_goal.drone_world_goal import DroneWorldGoal
from drone_world.planner.low_level.local_search_goal.node import Node
from drone_world.planner.low_level.tabu_local_search.tabu import TabuSearch

class PathNotFound(Exception):
    def __init__(self, msg):
        super(PathNotFound, self).__init__(msg)

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

        init_node = DroneWorldGoal.generate_search_node(goal_x, goal_y, goal_z, drone_world)
        self.tabu_search = TabuSearch(init_node, mem_limit, max_iters=max_iters)
        self.drone_world = drone_world
        self.max_iters = max_iters

    def run(self):
        """Run the Tabu local planner.

        If no solution is found within the max number of iterations, a PathNotFound exception
        is generated. Else, the drone will be moved to the generated solution.
        """

        # Get the solution
        solution = self.tabu_search.run()
        if not solution:
            raise PathNotFound("Unable to find solution within {} iterations".format(self.max_iters))
        if not isinstance(solution, Node):
            raise TypeError("Expected solution to be of type Node")

        # Apply the solution to actual drone world
        actions = solution.get_actions()
        for action in actions:
            x, y, z = action
            self.drone_world.move(z, y, z)