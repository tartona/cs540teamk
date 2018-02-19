from drone_world.figure.drone_world_figure import DroneWorldFigure
from drone_world.drone_world import DroneWorld
from drone_world.population_search.drone_world_goal import TowerPlannerCrow

if __name__ == "__main__":
    # Initialize the world
    world = DroneWorld()
    #world.initialize("worlds/50_red.csv")
    world.initialize("C:/Users/pinkmaggot/Documents/GitHub/cs540teamk/src/worlds/40_red_40_blue_40_green_40_yellow_blocks.csv")

    # Display start world
    # TODO:  Re-enable graphics
    # DroneWorldFigure(world).show()

    # Set the goal position to be the first read block that is not covered
    print("Running Crow Search...")
    tower = TowerPlannerCrow(world)
    fitness, solution = tower.run()

    # Display stats
    print("Total Distance Traveled: {}".format(fitness))
    print("Runtime: {}".format(tower.runtime))
    #for s in solution:
    #    print(s)

    # Display end world
    # TODO:  Re-enable graphics
    # DroneWorldFigure(world).show()

    exit(0)
