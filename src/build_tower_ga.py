from drone_world.drone_world import DroneWorld
from drone_world.population_search.genetic_algorithm import GeneticAlgorithmTowerPlanner
from drone_world.population_search.genetic_algorithm import TabuTowerPlannerRunner
from drone_world.figure.drone_world_figure import DroneWorldFigure

if __name__ == "__main__":
    # Initialize the world
    world = DroneWorld()
    world.initialize("worlds/40_red_40_blue_40_green_40_yellow_blocks.csv")

    # Display start world
    DroneWorldFigure(world).show()

    # Create and run the planner
    planner = GeneticAlgorithmTowerPlanner(world, pattern="rrggbbyy")
    planner.run()

    # Get the best plan
    plan = planner.get_best_plan()

    # Run the actual plan
    runner = TabuTowerPlannerRunner(plan, world)
    runner.run()

    # Display start world
    DroneWorldFigure(world).show()

    exit(0)