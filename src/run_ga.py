from drone_world.drone_world import DroneWorld
from drone_world.population_search.drone_world_goal import TowerPlannerGA
from drone_world.tower_runners.tabu_tower_runner import TabuTowerRunner

if __name__ == "__main__":
    # Initialize the world
    world = DroneWorld()
    world.initialize("worlds/40_red_40_blue_40_green_40_yellow_blocks.csv")

    # Set the goal position to be the first read block that is not covered
    print("Running GA for planning the tower..")
    goal_height = world.y_max - 1
    planner = TowerPlannerGA(world, goal_height)
    fitness, solution = planner.run()
    print("Planning done")
    print("GA Planner Runtime: {}".format(planner.runtime))
    print("Distance Estimate: {}".format(fitness))

    print("Running Tabu Search for moving blocks..")
    drone_x, drone_y, drone_z = world.get_drone_location()
    runner = TabuTowerRunner(world, solution, x=drone_x, z=drone_z, debug=True)
    runner.run()

    print("Running done")
    print("Tabu Runner Runtime: {}".format(runner.get_runtime()))
    print("Total Moves: {}".format(world._drone.get_moves()))

    exit(0)