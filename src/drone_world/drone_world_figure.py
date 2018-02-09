from drone_world import DroneWorld
from drone_world_object import DroneWorldObjectId
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

class DroneWorldFigure(object):
    def __init__(self, world):
        if not isinstance(world, DroneWorld):
            raise TypeError("World must be a DroneWorld object")
        self.world = world
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self._draw_blocks()

    def _draw_blocks(self):
        for state in self.world.state():
            obj_id, x, y, z = state
            color = DroneWorldObjectId.id_to_str(obj_id)

            # Need to swap y and z values
            tmp = y
            y = z
            z = tmp

            # vertices of a cube
            v = np.array(
                [[x, y, z],
                 [x, y, z + 1],
                 [x, y + 1, z],
                 [x, y + 1, z + 1],
                 [x + 1, y, z],
                 [x + 1, y, z + 1],
                 [x + 1, y + 1, z],
                 [x + 1, y + 1, z + 1]]
            )
            self.ax.scatter3D(v[:, 0], v[:, 1], v[:, 2])

            # generate list of sides' polygons of our pyramid
            verts = [
                [v[0], v[2], v[3], v[1]],
                [v[0], v[4], v[5], v[1]],
                [v[4], v[6], v[7], v[5]],
                [v[6], v[2], v[3], v[7]],
                [v[0], v[4], v[6], v[2]],
                [v[1], v[5], v[7], v[3]]
            ]

            # plot sides
            self.ax.add_collection3d(Poly3DCollection(verts, facecolors=color, linewidths=1, edgecolors='black', alpha=.25))
        return

    def show(self):
        plt.show()