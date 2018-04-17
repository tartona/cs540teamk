from drone_world.object.drone_world_object import DroneWorldObject
from drone_world.object.block import Block

class Drone(DroneWorldObject):
    def __init__(self, world, x, y, z, color):
        super(Drone, self).__init__(world, x, y, z, color)
        self._attached_block = None

    def is_attached(self):
        return self._attached_block is not None

    def attach(self):
        """Search for a Block object directly below drone location.
        """
        if self._attached_block:
            raise RuntimeError("Drone already has an attached block")
        world_object = self._world.get_object(self.x, self.y - 1, self.z)
        if isinstance(world_object, Block):
            self._attached_block = world_object
        else:
            raise RuntimeError("No block to attach to")

    def release(self):
        """Release an attached Block.
        """
        if not self._attached_block:
            raise RuntimeError("Drone does not have an attached block to be released")
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

            # Move the block and the drone
            block_moved = False
            drone_moved = False
            if dy < 0:
                block_moved = self._attached_block.move(dx, dy, dz)
                if block_moved:
                    drone_moved = super(Drone, self).move(dx, dy, dz)
            else:
                drone_moved = super(Drone, self).move(dx, dy, dz)
                if drone_moved:
                    block_moved = self._attached_block.move(dx, dy, dz)

            # If the moves were not successful, rollback any block or drone moves
            if block_moved and drone_moved:
                return True
            elif block_moved and not drone_moved:
                self._attached_block.move(-dx, -dy, -dz)
            elif not block_moved and drone_moved:
                super(Drone, self).move(-dx, -dy, -dz)
        return False

    def actions(self):
        if not self._attached_block:
            return super(Drone, self).actions()
        else:
            drone_actions = super(Drone, self).actions()
            block_actions = self._attached_block.actions()

            # Find the intersection of drone and block actions. Note that this does not account
            # for valid movements along the y-axis since super().actions() will report that the
            # drone cannot move done since the block is there.
            actions = list(set(drone_actions) & set(block_actions))

            # Add in y-axis actions
            if self._world.can_move_object(self.x, self.y + 1, self.z):
                actions.append((0, 1, 0))
            if self._world.can_move_object(self._attached_block.x, self._attached_block.y - 1, self._attached_block.z):
                actions.append((0, -1, 0))
            return actions

    def speak(self, msg):
        """ Not implemented speak function.
        """
        return