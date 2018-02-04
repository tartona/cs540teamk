import re

class DroneWorldObjectId(object):
    DRONE = 0
    RED = 1
    GREEN = 2
    BLUE = 4

    @staticmethod
    def str_to_id(string):
        if re.search("red", string, re.IGNORECASE):
            return DroneWorldObjectId.RED
        elif re.search("green", string, re.IGNORECASE):
            return DroneWorldObjectId.GREEN
        elif re.search("blue", string, re.IGNORECASE):
            return DroneWorldObjectId.BLUE
        elif re.search("drone", string, re.IGNORECASE):
            return DroneWorldObjectId.DRONE
        else:
            raise ValueError("Unsupported drone world object type: {}".format(string))

class DroneWorldObject(object):
    def __init__(self, world, x, y, z, object_id):
        self.x = x
        self.y = y
        self.z = z
        self.id = object_id
        self._world = world

    def move(self, dx, dy, dz):
        """Move this object to a new location in the world.
        If the location is occupied or invalid by another object, the result of this function is false.
        """
        new_x = self.x + dx
        new_y = self.y + dy
        new_z = self.z + dz
        if self._world.can_move_object(new_x, new_y, new_z):
            self.x = new_x
            self.y = new_y
            self.z = new_z
            return True
        return False

    def location(self):
        """Return (x, y, z) location of object.
        """
        return self.x, self.y, self.z

    def state(self):
        """Return a (id, x, y, z) tuple.
        """
        return self.id, self.x, self.y, self.z

    def actions(self):
        """Return all the possible actions for the drone.
        An action is consider a variation to only a single x, y, or z value for the current
        position. All actions are verified with the world. All actions are represented as a
        (dx, dy, dz) for the object.
        """
        valid_actions = []
        if self._world.can_move_object(self.x + 1, self.y, self.z):
            valid_actions.append((1, 0, 0))
        if self._world.can_move_object(self.x - 1, self.y, self.z):
            valid_actions.append((-1, 0, 0))
        if self._world.can_move_object(self.x, self.y + 1, self.z):
            valid_actions.append((0, 1, 0))
        if self._world.can_move_object(self.x, self.y - 1, self.z):
            valid_actions.append((0, -1, 0))
        if self._world.can_move_object(self.x, self.y, self.z + 1):
            valid_actions.append((0, 0, 1))
        if self._world.can_move_object(self.x, self.y, self.z - 1):
            valid_actions.append((0, 0, -1))
        return valid_actions

    def __eq__(self, other):
        return self.id == other.id and self.x == other.x and self.y == other.y and self.z == other.z