class DroneWorldObjectId(object):
    DRONE = 0
    RED = 1
    GREEN = 2
    BLUE = 4

class DroneWorldObject(object):
    def __init__(self, world, x, y, z, object_id):
        self.x = x
        self.y = y
        self.z = z
        self.id = object_id
        self._world = world
        self._world.add_object(self, x, y, z)

    def move(self, dx, dy, dz):
        """Move this object to a new location in the world.
        If the location is occupied or invalid by another object, the result of this function is false.
        """
        new_x = self.x + dx
        new_y = self.y + dy
        new_z = self.z + dz
        if self._world.move_object(self.x, self.y, self.z, new_x, new_y, new_z):
            self.x = new_x
            self.y = new_y
            self.z = new_z
            return True
        return False

    def state(self):
        """Return a (id, x, y, z) tuple.
        """
        return self.id, self.x, self.y, self.z