import math
from drone_world_object import DroneWorldObject
from block import Block

class Drone(DroneWorldObject):
    def __init__(self, world, x, y, z, object_id):
        super(Drone, self).__init__(world, x, y, z, object_id)
        self._attached_block = None
        self._moves = 0

    def get_moves(self):
        """Get the current number of moves by the drone
        """
        return self._moves

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

        # Record the number of moves
        if dx != 0:
            self._moves += int(math.fabs(dx))
        if dy != 0:
            self._moves += int(math.fabs(dy))
        if dz != 0:
            self._moves += int(math.fabs(dz))

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

            # Evaluate all moves in the positive y direction from the drone's perspective
            y = self.y + 1
            while self._world.can_move_object(self.x, y, self.z):
                actions.append((0, y - self.y, 0))
                y += 1

            # Evaluate all moves in the negative y direction from the block's perspective
            y = self._attached_block.y - 1
            while self._world.can_move_object(self._attached_block.x, y, self._attached_block.z):
                actions.append((0, y - self._attached_block.y, 0))
                y -= 1

            return actions

    def speak(self, msg):
        """ Not implemented speak function.
        """
        return