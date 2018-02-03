import numpy as np
import math

class DroneWorldBoundsError(Exception):
    def __init__(self, msg):
        super(DroneWorldBoundsError, self).__init__(msg)

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

        # Allocate 3D object array for the drone world
        x_range = x_max - x_min + 1
        y_range = y_max - y_min + 1
        z_range = z_max - z_min + 1
        self._world = np.empty((x_range, y_range, z_range), dtype=object)

        # List of objects populated in the world
        self._objects = []

    def _x_index(self, x):
        """Function to convert x world location into an array index.
        """
        if x < self.x_min or x > self.x_max:
            raise DroneWorldBoundsError("X world position outside defined drone world")
        return x - self.x_min

    def _y_index(self, y):
        """Function to convert y world location into an array index.
        """
        if y < self.y_min or y > self.y_max:
            raise DroneWorldBoundsError("Y world position outside defined drone world")
        return y - self.y_min

    def _z_index(self, z):
        """Function to convert z world location into an array index.
        """
        if z < self.z_min or z > self.z_max:
            raise DroneWorldBoundsError("Z world position outside defined drone world")
        return z - self.z_min

    def get_object(self, x, y, z):
        """Get object from world based on (x, y, z) location.
        """
        x_index = self._x_index(x)
        y_index = self._y_index(y)
        z_index = self._z_index(z)
        world_object = self._world[x_index][y_index][z_index]
        return world_object

    def is_occupied(self, x, y, z):
        """"Check to see if (x, y, z) location is occupied.
        """
        return self.get_object(x, y, z) is not None

    def add_object(self, drone_object, x, y, z):
        """Add an object to the drone world at (x, y, z) location.
        """
        if self.is_occupied(x, y, z):
            raise ValueError("Cannot added object to occupied location")
        x_index = self._x_index(x)
        y_index = self._y_index(y)
        z_index = self._z_index(z)
        self._world[x_index][y_index][z_index] = drone_object
        self._objects.append(drone_object)

    def can_move_object(self, cur_x, cur_y, cur_z, new_x, new_y, new_z):
        """Verify that an object can move to the specified location.
        As of now, if the desired location is not occupied, the object can be moved.
        """
        return not self.is_occupied(new_x, new_y, new_z)

    def move_object(self, cur_x, cur_y, cur_z, new_x, new_y, new_z):
        """Move an object to a new (x, y, z) location.
        """
        cur_x_index = self._x_index(cur_x)
        cur_y_index = self._y_index(cur_y)
        cur_z_index = self._z_index(cur_z)
        new_x_index = self._x_index(new_x)
        new_y_index = self._y_index(new_y)
        new_z_index = self._z_index(new_z)
        self._world[new_x_index][new_y_index][new_z_index] = self._world[cur_x_index][cur_y_index][cur_z_index]
        self._world[cur_x_index][cur_y_index][cur_z_index] = None

    def state(self):
        """Get the state of all the objects in the drone world.
        """
        state = []
        for world_object in self._objects:
            state.append(world_object.state())
        return state