from drone_world.drone_world import DroneWorld
from drone_world.planner.high_level.csa_planner import CrowSearchPlanner
from drone_world.figure.drone_world_figure import DroneWorldFigure

if __name__ == "__main__":
    world = DroneWorld()
    world.initialize("world_states/multiple_covered_blocks/initial.txt")
    DroneWorldFigure(world).show()

    csa_planner = CrowSearchPlanner(world)
    csa_planner.initialize("world_states/multiple_covered_blocks/goal.txt")
    csa_planner.run()

    DroneWorldFigure(world).show()

    drone_moves = world.get_drone_move_counter();

    print "Solution provided in " + str(int(drone_moves)) + " moves."