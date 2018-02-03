from drone_world_object import DroneWorldObject
from block import Block

class DroneRuntimeError(Exception):
    def __init__(self, msg):
        super(DroneRuntimeError, self).__init__(msg)

class Drone(DroneWorldObject):
    def __init__(self, world, x, y, z, object_id):
        super(Drone, self).__init__(world, x, y, z, object_id)
        self._attached_block = None

    def attach(self):
        """Search for a Block object directly below drone location.
        """
        if self._attached_block:
            raise DroneRuntimeError("Drone already has an attached block")
        world_object = self._world.get_object(self.x, self.y - 1, self.z)
        if isinstance(world_object, Block):
            self._attached_block = world_object

    def release(self):
        """Release an attached Block.
        """
        if not self._attached_block:
            raise DroneRuntimeError("Drone does not have an attached block to be released")
        self._attached_block.drop()
        self._attached_block = None

    def move(self, dx, dy, dz):
        """Move the drone in the world.
         If a block is attached, need to make extra checks to verify space in the world for both
         the drone and attached block.
         If move is unsuccessful, false is returned
        """
        if not self._attached_block:
            return super(Drone, self).move(dx, dy, dz)
        else:
            drone_new_x = self.x + dx
            drone_nex_y = self.y + dy
            drone_new_z = self.z + dz
            block_new_x = self._attached_block.x + dx
            block_new_y = self._attached_block.y + dy
            block_new_z = self._attached_block.z + dz

            # Verify that the drone and attached block can move
            can_move_drone = False
            can_move_block = False
            if dy == 0:
                can_move_drone = self._world.can_move_object(self.x, self.y, self.z, drone_new_x, drone_nex_y,
                                                             drone_new_z)
                can_move_block = self._world.can_move_object(self._attached_block.x, self._attached_block.y,
                                                             self._attached_block.z, block_new_x, block_new_y,
                                                             block_new_z)
            elif dy > 0:
                can_move_drone = self._world.can_move_object(self.x, self.y, self.z, drone_new_x, drone_nex_y,
                                                             drone_new_z)
                can_move_block = can_move_drone
            elif dy < 0:
                can_move_block = self._world.can_move_object(self._attached_block.x, self._attached_block.y,
                                                             self._attached_block.z, block_new_x, block_new_y,
                                                             block_new_z)
                can_move_drone = can_move_block

            # Move the drone and attached block (need a rollback move feature?)
            if can_move_drone and can_move_block:
                if dy < 0:
                    self._attached_block.move(dx, dy, dz)
                    super(Drone, self).move(dx, dy, dz)
                else:
                    super(Drone, self).move(dx, dy, dz)
                    self._attached_block.move(dx, dy, dz)
            else:
                return False

        # Exit drone move function with success
        return True

    def speak(self, msg):
        """ Not implemented speak function.
        """
        return