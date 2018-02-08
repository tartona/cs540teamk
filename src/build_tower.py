from drone_world.drone_world_figure import DroneWorldFigure
from drone_world.drone_world import DroneWorld
from drone_world.drone_world_goal import TowerPlannerSimulateAnnealing

if __name__ == "__main__":
    # Initialize the world
    world = DroneWorld()
    world.initialize("worlds/50_red.csv")

    # Display start world
    DroneWorldFigure(world).show()

    # Set the goal position to be the first read block that is not covered
    tower = TowerPlannerSimulateAnnealing(0, 5, 0, world)
    tower.run()

    # Display stats
    print "Number of moves: {}".format(tower.moves)
    print "Runtime: {}".format(tower.runtime)

    # Display end world
    DroneWorldFigure(world).show()

    exit(0)