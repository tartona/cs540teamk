from drone_world.figure.drone_world_figure import DroneWorldFigure
from drone_world.drone_world import DroneWorld
from drone_world.population_search.drone_world_goal import TowerPlannerCrow
from drone_world.population_search.drone_world_goal import TabuTowerPlannerRunner

if __name__ == "__main__":
    # Initialize the world
    world = DroneWorld()
    #world.initialize("worlds/50_red.csv")
    world.initialize("C:/Users/pinkmaggot/Documents/GitHub/cs540teamk/src/worlds/40_red_40_blue_40_green_40_yellow_blocks.csv")

    # Display start world
    # TODO:  Re-enable graphics
    # DroneWorldFigure(world).show()

    # Set the goal position to be the first read block that is not covered
    print("Running Crow Search for planning the tower..")
    goal_height = 40
    planner = TowerPlannerCrow(world, goal_height)
    fitness, solution = planner.run()
    print("Planning done")
    print("Crow Search Planner Runtime: {}".format(planner.runtime))
    print("Distance Estimate: {}".format(fitness))
    #for s in solution:
    #    print(s)

    #print("Running Tabu Search for moving blocks..")
    #runner = TabuTowerPlannerRunner(solution, world)
    #moves = runner.run()
    #print("Running done")
    #print("Tabu Runner Runtime: {}".format(runner.runtime))
    #print("Total Moves: {}".format(moves))

    # Display end world
    # TODO:  Re-enable graphics
    # DroneWorldFigure(world).show()

    exit(0)
