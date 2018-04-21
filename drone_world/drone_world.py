import csv
import re
from object.block import Block
from object.drone import Drone

class DroneWorld(object):
    def __init__(self, x_min=-50, x_max=50, y_min=0, y_max=50, z_min=-50, z_max=50, swap_yz=False, trace=False):
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
        self.swap_yz = swap_yz
        self.trace = trace

        # List of objects populated in the world
        self._drone = None
        self._blocks = []

        # Traced drone locations
        self._trace = []

    def get_drone_move_counter(self):
        """Get the total number of moves for the drone.
        :return: int
        """
        return self._drone.moves

    def add_drone(self, x, y, z):
        """Add a drone to the world.
        """
        if self._drone:
            raise RuntimeError("Drone is already allocated in the world")
        if not self.can_move_object(x, y, z):
            raise ValueError("Cannot allocate drone at occupied location ({}, {}, {})".format(x, y, z))
        self._drone = Drone(self, x, y, z, "pink")

    def add_block(self, x, y, z, obj_id):
        """Add a block to the world.
        """
        if not self.can_move_object(x, y, z):
            raise ValueError("Cannot allocate block at occupied location ({}, {}, {})".format(x, y, z))
        self._blocks.append(Block(self, x, y, z, obj_id))

    def add_object(self, x, y, z, color):
        """Add object to the world of a specific color.
        """
        if re.search("drone", color, re.IGNORECASE):
            self.add_drone(x, y, z)
        else:
            self.add_block(x, y, z, color)

    def verify_world_bounds(self, x, y, z):
        """Verify that the (x, y, z) location is within the defined drone world.
        """
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

    def can_move_drone(self, new_x, new_y, new_z):
        """Verify that the drone can move to the specified location.
        """
        if self.is_drone_attached():
            return self.can_move_object(new_x, new_y, new_z) and self.can_move_object(new_x, new_y - 1, new_z)
        else:
            return self.can_move_object(new_x, new_y, new_z)

    def is_occupied_by_block(self, x, y, z):
        """Check to see if location is occuppied by a block.
        :return: bool
        """
        for block in self._blocks:
            if (x, y, z) == block.location():
                return True
        return False


    def get_drone_location(self):
        """Get the current drone (x, y, z) location.
        """
        return self._drone.x, self._drone.y, self._drone.z

    def is_drone_attached(self):
        return self._drone.is_attached()

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
        did_move = self._drone.move(dx, dy, dz)
        if did_move and self.trace:
            trace_element = ("gray", self._drone.x, self._drone.y, self._drone.z)
            if trace_element not in self._trace:
                self._trace.append(trace_element)
        return did_move

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

    def get_trace(self):
        return self._trace

    def initialize(self, filename):
        """Initialize the drone world from a file.
        """
        with open(filename, "rt") as csv_file:
            reader = csv.reader(csv_file, delimiter=" ")
            for row in reader:
                if self.swap_yz:
                    self.add_object(int(row[0]), int(row[2]), int(row[1]), str(row[3]))
                else:
                    self.add_object(int(row[0]), int(row[1]), int(row[2]), str(row[3]))

    def actions(self):
        """Get all the actions for the drone.
        """
        return self._drone.actions()

    def __eq__(self, other):
        return self._drone == other._drone and self._blocks == other._blocks