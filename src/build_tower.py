from drone_world.drone_world import DroneWorld
from drone_world.figure.drone_world_figure import DroneWorldFigure
from drone_world.local_search.drone_world_goal import TowerPlannerTabu

if __name__ == "__main__":
    # Initialize the world
    world = DroneWorld()
    world.initialize("worlds/40_red_40_blue_40_green_40_yellow_blocks.csv")

    # Display start world
    DroneWorldFigure(world).show()

    # Set the goal position to be the first read block that is not covered
    tower = TowerPlannerTabu(0, 5, 0, world)
    tower.run()

    # Display stats
    print ("Number of moves: {}".format(tower.moves))
    print ("Runtime: {}".format(tower.runtime))

    # Display end world
    DroneWorldFigure(world).show()

    exit(0)