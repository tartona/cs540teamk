from drone_world.search.tabu import TabuSearch
from drone_world.drone_world import DroneWorld
from drone_world.drone_world_goal import DroneWorldGoal
from drone_world.drone_world_object import DroneWorldObjectId

if __name__ == "__main__":
    # Initialize the world
    world = DroneWorld()
    world.initialize("worlds/50_red.csv")

    # Set the goal position to be the first read block that is not covered
    red_block_goal = None
    for state in world.state():
        obj_id, x, y, z = state
        if obj_id == DroneWorldObjectId.RED:
            red_block_goal = (x, y, z)

    # Adjust the read_block_goal position to a drone attached position
    x, y, z = red_block_goal
    y = y + 1

    # Verify that the location can be reach
    if not world.verify_world_bounds(x, y, z):
        raise RuntimeError("Goal position cannot be achieved")

    # Get the initial node for the search
    init_node = DroneWorldGoal.generate_search_node(x, y, z, world)
    search = TabuSearch(init_node, 20)

    # Run the search
    solution = search.run()

    # From the solution node, get the actions and move the drone
    actions = solution.get_actions()
    for action in actions:
        x, y, z = action
        world.move(x, y, z)

    exit(0)