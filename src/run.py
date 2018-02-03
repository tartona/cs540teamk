from drone_world import DroneWorld
from drone_world_object import DroneWorldObjectId
from block import Block
from drone import Drone

if __name__ == "__main__":

    # Create drone world
    world  = DroneWorld()

    # Add a drone to the world
    drone = Drone(world, 0, 0, 0, DroneWorldObjectId.DRONE)

    # Add a block to the world
    block = Block(world, -1, 1, 0, DroneWorldObjectId.BLUE)

    # Dump initial state
    print world.state()

    # Move drone and attach to block
    drone.move(0, 1, 0)
    drone.move(-1, 0, 0)
    drone.attach()

    # Move the drone with the attached block
    drone.move(0, 1, 0)
    drone.move(0, 1, 0)

    # Release the block
    drone.release()

    # Dump the state of the world
    print world.state()