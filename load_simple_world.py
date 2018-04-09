from drone_world.drone_world import DroneWorld
from drone_world.figure.drone_world_figure import DroneWorldFigure

if __name__ == "__main__":
    world = DroneWorld()
    world.initialize("world_states/simple/initial.txt")
    DroneWorldFigure(world).show()