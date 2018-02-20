from drone_world.figure.drone_world_figure import DroneWorldFigure
from drone_world.drone_world import DroneWorld
from drone_world.population_search.drone_world_goal import TowerPlanner
from drone_world.population_search.drone_world_goal import TabuTowerPlannerRunner
import numpy as np
import copy

def calculate_mean(list, axis):
    sum = 0
    for i in range(0, len(list)):
        sum += list[i][axis]
    return sum/len(list)

def find_best(list, axis):
    best = 99999
    for i in range(0, len(list)):
        if best > list[i][axis]:
            best = list[i][axis]
    return best

def find_worst(list, axis):
    worst = -1
    for i in range(0, len(list)):
        if worst < list[i][axis]:
            worst = list[i][axis]
    return worst

def calculate_std(list, axis):
    new_list = []
    for i in range(0, len(list)):
        new_list.append(list[i][axis])
    return np.std(new_list)

if __name__ == "__main__":
    
    ''' Variables '''
    run_tabu = False # True if you want to run tabu search to move blocks and find the number of moves
    number_of_runs = 10
    fixed_goal = True # True if you want to run the algorithms for the same goal multiple times
    goal_height = 40 # height of the tower
    
    # Initialize the world
    world = DroneWorld()
    world.initialize("C:/Users/pinkmaggot/Documents/GitHub/cs540teamk/src/worlds/75_red_40_blue_40_green_40_yellow_blocks.csv")

    # Display start world
    # TODO:  Re-enable graphics
    # DroneWorldFigure(world).show()

    goal = None
    crow_results = [] # Duck
    DCS_results = [] # Duck
    GA_results = [] # Ian
    cuckoo_results = [] # Mike

    crow_moves = []
    DCS_moves = []
    GA_moves = []
    cuckoo_moves = []

    print("Starting the test..")
    for i in range(0, number_of_runs):
        if fixed_goal and i>0:
            goal = crow_results[0][1]
        planner = TowerPlanner(world, goal_height)
        result = planner.run()
        crow_results.append(result[0])
        DCS_results.append(result[1])
        GA_results.append(result[2])
        cuckoo_results.append(result[3])
        if run_tabu:
            runner = TabuTowerPlannerRunner(crow_results[i][1], copy.deepcopy(world))
            moves = runner.run()
            crow_moves.append(moves)
            runner = TabuTowerPlannerRunner(DCS_results[i][1], copy.deepcopy(world))
            moves = runner.run()
            DCS_moves.append(moves)
            runner = TabuTowerPlannerRunner(GA_results[i][1], copy.deepcopy(world))
            moves = runner.run()
            GA_moves.append(moves)
            runner = TabuTowerPlannerRunner(cuckoo_results[i][1], copy.deepcopy(world))
            moves = runner.run()
            cuckoo_moves.append(moves)
    print("Test is done")
    print("Total Runtime: {}".format(planner.runtime))
    print("")
    print("Result - Fitness")
    print("Algo\tBEST\t\tMEAN\t\tSTD\t\tWORST")
    print("%s\t%0.1f\t\t%0.1f\t\t%0.1f\t\t%0.1f"%("1. CSA", find_best(crow_results, 0), calculate_mean(crow_results, 0), calculate_std(crow_results, 0), find_worst(crow_results, 0)))
    print("%s\t%0.1f\t\t%0.1f\t\t%0.1f\t\t%0.1f"%("2. DDCS", find_best(DCS_results, 0), calculate_mean(DCS_results, 0), calculate_std(DCS_results, 0), find_worst(DCS_results, 0)))
    print("%s\t%0.1f\t\t%0.1f\t\t%0.1f\t\t%0.1f"%("3. GA", find_best(GA_results, 0), calculate_mean(GA_results, 0), calculate_std(GA_results, 0), find_worst(GA_results, 0)))
    print("%s\t%0.1f\t\t%0.1f\t\t%0.1f\t\t%0.1f"%("4. MDCS", find_best(cuckoo_results, 0), calculate_mean(cuckoo_results, 0), calculate_std(cuckoo_results, 0), find_worst(cuckoo_results, 0)))
    print("")
    print("Result - Runtime")
    print("Algo\tBEST\t\tMEAN\t\tSTD\t\tWORST")
    print("%s\t%0.3f\t\t%0.3f\t\t%0.3f\t\t%0.3f"%("1. CSA", find_best(crow_results, 2), calculate_mean(crow_results, 2), calculate_std(crow_results, 2), find_worst(crow_results, 2)))
    print("%s\t%0.3f\t\t%0.3f\t\t%0.3f\t\t%0.3f"%("2. DCS", find_best(DCS_results, 2), calculate_mean(DCS_results, 2), calculate_std(DCS_results, 2), find_worst(DCS_results, 2)))
    print("%s\t%0.3f\t\t%0.3f\t\t%0.3f\t\t%0.3f"%("3. GA", find_best(GA_results, 2), calculate_mean(GA_results, 2), calculate_std(GA_results, 2), find_worst(GA_results, 2)))
    print("%s\t%0.3f\t\t%0.3f\t\t%0.3f\t\t%0.3f"%("4. MDCS", find_best(cuckoo_results, 2), calculate_mean(cuckoo_results, 2), calculate_std(cuckoo_results, 2), find_worst(cuckoo_results, 2)))
    if run_tabu:
        print("")
        print("Result - Moves")
        print("Algo\tBEST\t\tMEAN\t\tSTD\t\tWORST")
        print("%s\t%0.1f\t\t%0.1f\t\t%0.1f\t\t%0.1f"%("1. CSA", np.min(crow_moves), np.mean(crow_moves), np.std(crow_moves), np.max(crow_moves)))
        print("%s\t%0.1f\t\t%0.1f\t\t%0.1f\t\t%0.1f"%("2. DCS", np.min(DCS_moves), np.mean(DCS_moves), np.std(DCS_moves), np.max(DCS_moves)))
        print("%s\t%0.1f\t\t%0.1f\t\t%0.1f\t\t%0.1f"%("3. GA", np.min(GA_moves), np.mean(GA_moves), np.std(GA_moves), np.max(GA_moves)))
        print("%s\t%0.1f\t\t%0.1f\t\t%0.1f\t\t%0.1f"%("4. MDCS", np.min(cuckoo_moves), np.mean(cuckoo_moves), np.std(cuckoo_moves), np.max(cuckoo_moves)))
    # Display end world
    # TODO:  Re-enable graphics
    # DroneWorldFigure(world).show()

    exit(0)
