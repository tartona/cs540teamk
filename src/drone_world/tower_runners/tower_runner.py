import math
from abc import ABCMeta, abstractmethod
from ..drone_world import DroneWorld
from ..object.drone_world_object import DroneWorldObjectId

class TowerRunner(object):
    __metaclass__ = ABCMeta

    VALID_BLOCK_IDS = [DroneWorldObjectId.RED, DroneWorldObjectId.BLUE, DroneWorldObjectId.YELLOW,
                       DroneWorldObjectId.GREEN]

    def __init__(self, drone_world, tower_blocks, x=0, z=0, debug=True):
        """Given a list tower blocks, move the blocks in the world to construct a tower.
        The tower MUST be buildable (drone and tower must not exceed world bounds).
        :param drone_world: Drone world object
        :param tower_blocks: Ordered list of blocks (the plan) to be put into a tower.
        :param x: X-location to start tower
        :param z: Z-location to start tower
        """

        # Verify arguments
        if not isinstance(drone_world, DroneWorld):
            raise TypeError("World must be a drone world type")
        if x < drone_world.x_min or x > drone_world.x_max:
            raise ValueError("X location outside the world")
        if z < drone_world.z_min or z > drone_world.z_max:
            raise ValueError("Z location outside the world")
        if not isinstance(tower_blocks, list):
            raise TypeError("Planner must a list object")
        if len(tower_blocks) > int(math.fabs(drone_world.y_max - drone_world.y_min)) - 1:
            raise ValueError("Number of blocks exceeds the world")

        self._drone_world = drone_world
        self._tower_blocks = tower_blocks
        self._x = x
        self._z = z
        self._cur_height = 0
        self._debug = debug
        self._start_time = None
        self._end_time = None

    def get_runtime(self):
        if self._start_time is None:
            raise ValueError("Start time not set")
        if self._end_time is None:
            raise ValueError("End time not set")
        return self._end_time - self._start_time

    def _generate_attach_goal(self, state):
        """Generate an attach goal.
        """
        obj_id, x, y, z = state
        if obj_id == DroneWorldObjectId.DRONE or x == self._x and z == self._z:
            raise RuntimeError("Invalid goal position")
        elif obj_id not in TowerRunner.VALID_BLOCK_IDS:
            raise ValueError("Invalid block object id: {}".format(obj_id))
        else:
            if not self._drone_world.verify_world_bounds(x, y + 1, z):
                raise RuntimeError("Goal position cannot be achieved")
            return x, y + 1, z

    def _generate_release_goal(self):
        """Generate a release goal.
        """
        if not self._drone_world.verify_world_bounds(self._x, self._cur_height + 1, self._z):
            raise RuntimeError(
                "Goal position cannot be achieved: {}".format((self._x, self._cur_height + 1, self._z)))
        return self._x, self._cur_height + 1, self._z

    @abstractmethod
    def run(self):
        pass
