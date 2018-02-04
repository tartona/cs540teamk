from drone_world_object import DroneWorldObject

class Block(DroneWorldObject):
    def __init__(self, world, x, y, z, object_id):
        super(Block, self).__init__(world, x, y, z, object_id)
        self.drop()

    def drop(self):
        """Move the block to the lowest y position holding x and z.
        """
        while self.move(0, -1, 0):
            continue