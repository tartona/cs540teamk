import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from drone_world.drone_world import DroneWorld

class DroneWorldFigureLite(object):
    def __init__(self, world):
        if not isinstance(world, DroneWorld):
            raise TypeError("World must be a DroneWorld object")
        self.world = world
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Z")
        self.ax.set_zlabel("Y")
        #self.ax.set_ylim(bottom=world.z_min, top=world.z_max)
        #self.ax.set_zlim(bottom=world.y_min, top=world.y_max)
        #self.ax.set_xlim(left=world.x_min, right=world.x_max)
        self._draw_blocks()

    def _draw_blocks(self):
        for state in self.world.state():
            color, x, y, z = state

            # Need to swap y and z values
            tmp = y
            y = z
            z = tmp

            # vertices of a cube
            v = np.array(
                [[x, y, z]]
            )
            self.ax.scatter3D(v[:, 0], v[:, 1], v[:, 2], c=color)
        return

    def show(self):
        plt.show()

class DroneWorldFigure(object):
    def __init__(self, world):
        if not isinstance(world, DroneWorld):
            raise TypeError("World must be a DroneWorld object")
        self.world = world
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Z")
        self.ax.set_zlabel("Y")
        #self.ax.set_ylim(bottom=world.z_min, top=world.z_max)
        #self.ax.set_zlim(bottom=world.y_min, top=world.y_max)
        #self.ax.set_xlim(left=world.x_min, right=world.x_max)
        self._draw_blocks()

    def _draw_blocks(self):
        for state in self.world.state():
            color, x, y, z = state

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
            self.ax.scatter3D(v[:, 0], v[:, 1], v[:, 2], c=color)

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