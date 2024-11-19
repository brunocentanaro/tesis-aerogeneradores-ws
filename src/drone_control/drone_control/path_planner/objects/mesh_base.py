from abc import ABC
import numpy as np
from stl import mesh

class MeshBase(ABC):
    def __init__(self, vertices, color):
        self.vertices = vertices
        self.color = color

class Mesh(MeshBase):
    def __init__(self, vertices, color):
        super().__init__(vertices, color)
        self.colors = [random_color() for _ in range(len(self.vertices))]
        self.colors = np.array(self.colors)

class Wireframe:
    def __init__(self, vertices=None, normals=None, vertice_colors=None):
        self.vertices = vertices
        self.vertice_colors = [random_color() for _ in vertices] if vertice_colors is None else vertice_colors
        self.normals = self.compute_normals() if normals is None else normals
        self.center = self.compute_center()
        self.seen = np.zeros(len(self.vertices), int)

    # Outputs the normals if the wireframe has more than one vertex
    def output(self):
        if len(self.vertices) > 1:
            self.output_normals()

    # Prints the vertices of the wireframe
    def output_vertices(self):
        print("\n --- Vertices --- ")
        for i, vertice in enumerate(self.vertices):
            print(f"  {i}:")
            for (x, y, z) in vertice:
                print(f"  \t{(x, y, z)}")

    # Prints the normals of the wireframe
    def output_normals(self):
        print("\n --- Normals --- ")
        for i, (x, y, z) in enumerate(self.normals):
            print(f"  {i}:\t{(x, y, z)}")

    # Prints the coverage percentage of the wireframe and the number of covered triangles
    def output_coverage(self):
        print("\n --- Coverage --- ")
        print(f"  total amount covered: {(len(np.where(self.seen >= 1)[0]) / len(self.seen)) * 100}%")
        print(f"  total triangles: {len(self.seen)} and covered: {len(np.where(self.seen >= 1)[0])}")

    # Computes the normals for each triangle in the wireframe 
    # by calculating the cross product of the triangle's edges
    def compute_normals(self):
        v1 = self.vertices[:, 1] - self.vertices[:, 0]
        v2 = self.vertices[:, 2] - self.vertices[:, 0]
        cross1 = np.cross(v1, v2)
        # Normalize the normals
        norm = np.linalg.norm(cross1, axis=1)
        self.normals = (cross1.transpose() / norm).transpose()
        return self.normals

    # Computes the center (centroid) of each triangle in the wireframe
    def compute_center(self):
        centers = np.mean(self.vertices, axis=1)
        return centers

    # Creates a Wireframe from an STL object
    @staticmethod
    def from_stl(stl):
        vertices = stl.vectors
        normals = stl.units
        return Wireframe(vertices, normals)

    # Loads an STL file and creates a Wireframe from it
    @staticmethod
    def from_stl_path(path):
        stl = mesh.Mesh.from_file(path + ".stl")
        return Wireframe.from_stl(stl)
    
def random_color():
    return (int(np.random.uniform(0, 255)), int(np.random.uniform(0, 255)), int(np.random.uniform(0, 255)))

# Returns the unit vector (normalized vector) of a given vector
def unit_vector(v):
    return v / np.linalg.norm(v)
