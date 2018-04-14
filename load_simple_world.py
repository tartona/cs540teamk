from drone_world.drone_world import DroneWorld
from drone_world.planner.high_level.csa_planner import CrowSearchPlanner
from drone_world.figure.drone_world_figure import DroneWorldFigure

if __name__ == "__main__":
    world = DroneWorld()
    world.initialize("world_states/simple/initial.txt")
    DroneWorldFigure(world).show()

    csa_planner = CrowSearchPlanner(world)
    csa_planner.initialize("world_states/simple/goal.txt")
    csa_planner.run()

    DroneWorldFigure(world).show()