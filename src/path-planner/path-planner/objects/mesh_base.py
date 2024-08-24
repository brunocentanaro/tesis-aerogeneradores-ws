import time
from abc import abstractmethod, ABC
from pathlib import Path

import pygame
import numpy as np

# from tranformation.camera import CameraInfo
from stl import mesh

from transformation.transform import Transform

class MeshBase(ABC):
    def __init__(self, vertices, color):
        self.vertices = vertices
        self.color = color

#    @abstractmethod
#    def draw(self, screen, camera, color=None):
#        pass


def random_color():
    return (int(np.random.uniform(0, 255)), int(np.random.uniform(0, 255)), int(np.random.uniform(0, 255)))


def unit_vector(v):
    return v / np.linalg.norm(v)


class Mesh(MeshBase):
    def __init__(self, vertices, color):
        super().__init__(vertices, color)
        self.colors = [random_color() for _ in range(len(self.vertices))]
        self.colors = np.array(self.colors)

#    def draw(self, screen, camera: CameraInfo, color=None):
#        c = color if color is not None else self.color
#        for points in self.vertices:
#            for i in range(len(points)):
#                # transform into camera
#                p1_im = camera.transform_world_to_image(points[i])
#                p2_im = camera.transform_world_to_image(points[(i + 1) % (len(points) - 1)])
#                pygame.draw.line(screen, c, p1_im, p2_im)

#    def draw_vertices(self, screen, camera: CameraInfo):
#        depth = []
#        dist = []
#        for points in self.vertices:
#            v = []
#            center = 0
#            for point in points:
#                # p_c = np.squeeze(camera.transform_to_camera(point))[:-1]
#                p_c = np.squeeze(camera.transform_to_camera(point))
#                v.append(p_c)
#                center += p_c[0]
#            center /= 3
#            depth.append(v)
#            dist.append(center)
#        depth = np.array(depth)
#        ind = np.argsort(dist)[::-1]
#        depth = depth[ind]
#        c = self.colors[ind]
#        for i, points in enumerate(depth):
#            p = []
#            for point in points:
#                u, v = camera.transform_cam_to_image(point)
#                p.append((u, v))
#            pygame.draw.polygon(screen, c[i], p)
#        debug = 0


class Points:
    def __init__(self, points, color):
        self.points = points
        self.color = color

    #def draw(self, screen, camera: CameraInfo, color=None):
    #    c = color if color is not None else self.color
    #    for p in self.points:
    #        u, v = camera.transform_world_to_image(p)
    #        pygame.draw.circle(screen, c, (u, v), radius=3)


class Wireframe:
    """ An array of vectors in R3 and list of edges connecting them. """

    def __init__(self, vertices=None, normals=None, vertice_colors=None):
        self.vertices = vertices
        self.vertice_colors = [random_color() for _ in vertices] if vertice_colors is None else vertice_colors
        self.normals = self.compute_normals() if normals is None else normals
        self.center = self.compute_center()
        self.seen = np.zeros(len(self.vertices), int)

#    def add_vertices(self, vertice_array):
#        """ Append 1s to a list of 3-tuples and add to self.nodes. """
#        ones_added = np.hstack((vertice_array, np.ones((len(vertice_array), 1))))
#        self.vertices = np.vstack((self.vertices, ones_added))
#    
#    def add_faces(self, face_list, face_colour=(255, 255, 255)):
#        for node_list in face_list:
#            num_nodes = len(node_list)
#            if all((node < len(self.nodes) for node in node_list)):
#                # self.faces.append([self.nodes[node] for node in node_list])
#                self.faces.append((node_list, np.array(face_colour, np.uint8)))
#                self.add_edges([(node_list[n - 1], node_list[n]) for n in range(num_nodes)])

    def output(self):
        if len(self.vertices) > 1:
            self.output_normals()

    def output_vertices(self):
        print("\n --- Vertices --- ")
        for i, vertice in enumerate(self.vertices):
            print(f"  {i}:")
            for (x, y, z) in vertice:
                print(f"  \t{(x, y, z)}")

    def output_normals(self):
        print("\n --- Normals --- ")
        for i, (x, y, z) in enumerate(self.normals):
            print(f"  {i}:\t{(x, y, z)}")

    def output_coverage(self):
        print("\n --- Coverage --- ")
        print(f"  total amount covered: {(len(np.where(self.seen >= 1)[0]) / len(self.seen)) * 100}%")
        print(f"  total triangles: {len(self.seen)} and covered: {len(np.where(self.seen >= 1)[0])}")

#    def output_avg_vertice_per_img(self, num_img):
#        s = np.sum(self.seen)
#        print("\n --- avg vertice --- ")
#        print(f"  avg vertice seen per image: sum_of_triangle_seen:{s} / timesteps:{num_img} = {s / num_img}")
#        return s / num_img
#
#    def output_seen_count_per_vertice(self):
#        print("\n --- Seen count per vertice --- ")
#        for i, count in enumerate(self.seen):
#            print(f"  i={i}: {count}")
#
#    def get_coverage(self):
#        return (len(np.where(self.seen >= 1)[0]) / len(self.seen)) * 100
#
#    def get_coverage_text(self):
#        return f"{len(np.where(self.seen >= 1)[0])}/{len(self.seen)}"
#
    def transform(self, transform:Transform):
        """ Apply a transformation defined by a transformation matrix. """
        transformed_vertices = []
        for vertice in self.vertices:
            v = []
            for point in vertice:
                p = transform(point)
                v.append(p)
            transformed_vertices.append(v)
        return Wireframe(np.array(transformed_vertices).squeeze(), vertice_colors=self.vertice_colors)
#
#    def find_centre(self):
#        """ Find the spatial centre by finding the range of the x, y and z coordinates. """
#
#        min_values = self.nodes[:, :-1].min(axis=0)
#        max_values = self.nodes[:, :-1].max(axis=0)
#        return 0.5 * (min_values + max_values)
#
#    def get_boundary_points(self):
#        min_x = self.vertices[:, :, 0].argmin(axis=0)
#        min_x = self.vertices[:, :, 0].argmin(axis=0)
#        min_x = self.vertices[:, :, 0].argmin(axis=0)
#        min_x = self.vertices[:, :, 0].argmin(axis=0)
#
#    def sorted_vertices_ind(self):
#        l = np.abs(self.vertices[:, :, 0].min(axis=1))
#        l2 = np.abs(self.center[:, 0])
#        ind = np.lexsort((l2, l))
#        # ind = np.argsort(self.vertices[:, :, 0].min(axis=1))
#        ind_inv = ind[::-1]
#        return ind_inv
#
    def compute_normals(self):
        v1 = self.vertices[:, 1] - self.vertices[:, 0]
        v2 = self.vertices[:, 2] - self.vertices[:, 0]
        cross1 = np.cross(v1, v2)
        norm = np.linalg.norm(cross1, axis=1)
        self.normals = (cross1.transpose() / norm).transpose()

        # Commented out because it is too slow, but here is the more readable version
        # normals = []
        # for vertice in self.vertices:
        #     v1 = vertice[1] - vertice[0]
        #     v2 = vertice[2] - vertice[0]
        #     normal = unit_vector(np.cross(v1, v2))
        #     normals.append(normal)
        # self.normals = np.array(normals)

        return self.normals

    def compute_center(self):
        centers = np.mean(self.vertices, axis=1)

        # commented out because it is too slow
        # centers = []
        # for vertice in self.vertices:
        #     c = np.array([0, 0, 0], float)
        #     for point in vertice:
        #         c += point
        #     centers.append(c)
        # centers = np.array(centers)
        return centers

    def update(self):
        """ Override this function to control wireframe behaviour. """
        pass


    @staticmethod
    def from_stl(stl):
        vertices = stl.vectors
        normals = stl.units
        return Wireframe(vertices, normals)

    @staticmethod
    def from_stl_path(path):
        stl = mesh.Mesh.from_file(str(path))
        return Wireframe.from_stl(stl)

if __name__ == '__main__':
    def get_project_root() -> Path:
        return Path(__file__).parent.parent
    wt = Wireframe.from_stl_path(Path(get_project_root(), "stl_gen/turbine.stl"))
    wt.output()
