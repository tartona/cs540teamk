import argparse
from drone_world.drone_world import DroneWorld
from drone_world.planner.high_level.csa_planner import CrowSearchPlanner
from drone_world.figure.drone_world_figure import DroneWorldFigure

def parse_args():
    parser = argparse.ArgumentParser(description="Run a drone world planner")
    parser.add_argument("--init_file", type=str, help="Initial drone world file",
                        required=True)
    parser.add_argument("--goal_file", type=str, help="Drone world goal file",
                        required=True)
    parser.add_argument("--debug", action="store_true", help="Print debug messages", default=False)
    parser.add_argument("--graphics", action="store_true", help="Display init and end drone world", default=False)
    parser.add_argument("--swap_yz", action="store_true", help="Swap the YZ coordinates in init and goal file", default=False)
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    world = DroneWorld(swap_yz=args.swap_yz)
    world.initialize(args.init_file)

    if args.graphics:
        DroneWorldFigure(world).show()

    csa_planner = CrowSearchPlanner(world, debug=args.debug, swap_yz=args.swap_yz)
    csa_planner.initialize(args.goal_file)
    csa_planner.run()

    if args.graphics:
        DroneWorldFigure(world).show()

    drone_moves = world.get_drone_move_counter()
    runtime = csa_planner.get_runtime()
    replan_count = csa_planner.get_replan_count()
    print "Solution provided in " + str(drone_moves) + " moves."
    print "Solution needed " + str(replan_count) + " replans."
    print "Runtime in " + str(runtime) + " seconds."