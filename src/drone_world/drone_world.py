from drone_world_object import DroneWorldObjectId
from drone import Drone
from block import Block

class DroneWorld(object):
    def __init__(self, x_min=-50, x_max=50, y_min=0, y_max=50, z_min=-50, z_max=50):
        # Verify drone world dimensions
        if x_min > x_max:
            raise ValueError("Drone world x-min cannot be less than x-max")
        if y_min > y_max:
            raise ValueError("Drone world y-min cannot be less than y-max")
        if z_min > z_max:
            raise ValueError("Drone world z-min cannot be less than z-max")
        self.x_min = x_min
        self.y_min = y_min
        self.z_min = z_min
        self.x_max = x_max
        self.y_max = y_max
        self.z_max = z_max

        # List of objects populated in the world
        self._drone = None
        self._blocks = []

    def add_drone(self, x, y, z):
        """Add a drone to the world.
        """
        if self._drone:
            raise RuntimeError("Drone is already allocated in the world")
        if not self.can_move_object(x, y, z):
            raise ValueError("Cannot allocate drone at occupied location ({}, {}, {})".format(x, y, z))
        self._drone = Drone(self, x, y, z, DroneWorldObjectId.DRONE)

    def add_block(self, x, y, z, color):
        """Add a block to the world.
         Color must be specified as a string. Note that a block cannot be added to either the
         reserved drone location (0, 0, 0) if the drone is not create, and a block cannot be added
         above a drone.
        """
        if not self.can_move_object(x, y, z):
            raise ValueError("Cannot allocate block at occupied location ({}, {}, {})".format(x, y, z))
        if not self._drone and x == 0 and z == 0:
            raise ValueError("Cannot allocate block at reserved drone location of (0, 0, 0)")
        elif x == self._drone.x and z == self._drone.z and y > self._drone.y:
            raise ValueError("Cannot allocate a block above the drone")
        self._blocks.append(Block(self, x, y, z, DroneWorldObjectId.str_to_id(color)))

    def verify_world_bounds(self, x, y, z):
        if x < self.x_min or x > self.x_max:
            return False
        if y < self.y_min or y > self.y_max:
            return False
        if z < self.z_min or z > self.z_max:
            return False
        return True

    def get_object(self, x, y, z):
        """Get object from world based on (x, y, z) location.
        """
        for block in self._blocks:
            if (x, y, z) == block.location():
                return block
        if (x, y, z) == self._drone.location():
            return self._drone
        return None

    def can_move_object(self, new_x, new_y, new_z):
        """Verify that an object can move to the specified location.
        As of now, if the desired location is not occupied, the object can be moved.
        """
        if not self.verify_world_bounds(new_x, new_y, new_z):
            return False

        for block in self._blocks:
            if (new_x, new_y, new_z) == block.location():
                return False
        if self._drone:
            if (new_x, new_y, new_z) == self._drone.location():
                return False
        return True

    def attach(self):
        """Attach a block to the drone.
        """
        self._drone.attach()

    def release(self):
        """Release a block from the drone.
        """
        self._drone.release()

    def move(self, dx, dy, dz):
        """Move the drone in the drone world.
        """
        self._drone.move(dx, dy, dz)

    def speak(self, msg):
        """Not implemented.
        """
        self._drone.speak(msg)

    def state(self):
        """Get the state of all the objects in the drone world.
        """
        state = []
        for block in self._blocks:
            state.append(block.state())
        state.append(self._drone.state())
        return state

    def initialize(self, filename):
        """Initialize the drone world from a file.
        Not yet implemented.
        """
        self.add_drone(0, 0, 0)
        self.add_block(-1, 1, 0, "red")

    def actions(self):
        """Get all the actions for the drone.
        """
        return self._drone.actions()
