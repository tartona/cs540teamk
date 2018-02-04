import copy
from search.cost_node import CostNode
from search.tabu import TabuSearch
from drone_world.drone_world import DroneWorld
from drone_world.drone_world_object import DroneWorldObjectId

if __name__ == "__main__":
    # Initialize the world
    world = DroneWorld()
    world.initialize("worlds/test_world.txt")

    # Set the goal position to be the first read block that is not covered
    red_block_goal = None
    for state in world.state():
        obj_id, x, y, z = state
        if obj_id == DroneWorldObjectId.RED:
            if not world.is_block_covered(x, y, z):
                red_block_goal = (x, y, z)

    # Adjust the read_block_goal position to a drone attached position
    x, y, z = red_block_goal
    red_block_goal = world.attach_goal_location(x, y, z)

    # Set the goal for the world
    x, y, z = red_block_goal
    world.set_goal(x, y, z)

    # Run Tabu search
    init_node = CostNode(world, None, None, 0)
    search = TabuSearch(init_node, 20)

    # Run the search
    nodes = search.run()

    exit(0)