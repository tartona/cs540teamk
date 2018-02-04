from drone_world.drone_world import DroneWorld

if __name__ == "__main__":

    # Create drone world
    world  = DroneWorld()
    world.initialize("Not implemented")
    print world.state()

    # Move the drone
    world.move(-1,1,0)
    print world.state()

    # Attach and move with block (as of now, block is hardcoded in world init function)
    world.attach()
    world.move(5,5,5)
    print world.state()

    # Release the block
    world.release()
    print world.state()