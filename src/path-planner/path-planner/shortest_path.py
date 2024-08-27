import json
import random
import time
from pathlib import Path

import numpy as np
from ortools.constraint_solver import routing_enums_pb2, pywrapcp
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
from typing import List

from Visualizer.utils.colors import get_distinct_color
from objects.mesh_base import Wireframe, unit_vector
from polynomial_fitting import plot_data
from transformation.transform import TRotation, Transform
from utils.folder import get_project_root
from k_means_constrained import KMeansConstrained

cache = {}
data = {}
use_cache = True

class Section:
    def __init__(self, points, normal, color):
        self.points = np.array(points)
        self.normal = normal
        if len(self.points) > 1:
            # Ordena los puntos segun su coordenada Z
            indices = np.argsort(self.points[:, 2])
            self.points = self.points[indices]
            self.p1 = midpoint(self.points[0], self.points[1])  # lowest point (no need to sort)
            self.p2 = midpoint(self.points[-1], self.points[-2])  # highest point (need to reorder list)
        else:
            self.p1 = self.points[0]
            self.p2 = self.points[0]
        self.length = dist(self.p1, self.p2)
        self.p1_str = str(self.p1)
        self.p2_str = str(self.p2)
        self.is_one_point_section = self.p1_str == self.p2_str
        self.color = color

    def get_other_end(self, p):
        if np.all(np.isclose(p, self.p1)):
            return self.p2
        return self.p1

    def get_other_end_str(self, p_str: str) -> str:
        if p_str == self.p1_str:
            return self.p2_str
        return self.p1_str

    def scale_using_normal(self, multiplier):
        n = self.normal * np.array([1, 1, 0])
        n = unit_vector(n)
        p = self.points + n * multiplier
        return Section(p, self.normal, self.color)

    def get_ordered_points(self, p_str):
        if p_str == self.p1_str:
            return [self.p1, self.p2]
        return [self.p2, self.p1]
    
    def get_point_from_str(self, p_str):
        if p_str == self.p1_str:
            return self.p1
        return self.p2
    
    def get_ordered_points_and_index(self, p_str):
        ps = self.get_ordered_points(p_str)
        return ps, 1 if self.is_one_point_section else 2

def get_random_color():
    return (np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9))

def writefile(filename, data):
    with open(filename, 'w') as file:
        for data_point in data:
            L = [f"{str(data_point)}\n"]
            file.writelines(L)

def dist(p_to, p_from=np.array([0, 0, 0])):
    v = p_to - p_from
    return np.linalg.norm(v)

def shortest_path(start_node, sections: List[Section], multiplier=10, turn_cost=0):
    points_to_sections = {}
    points = []
    normals = []
    points_str = []
    for s in sections:
        n = s.normal
        points_to_sections[s.p1_str] = s
        points_to_sections[s.p2_str] = s
        points.append(s.p1)
        points_str.append(s.p1_str)
        normals.append(n)
        if np.any(s.p1 != s.p2):
            points.append(s.p2)
            points_str.append(s.p2_str)
            normals.append(n)

    # Compute cost between nodes (turn cost, dist cost, invalid turns=-1)
    # Hay 22 puntos en total 2 por lado + los dos individuales de cada lado.
    # Cada punto representa el punto medio entre los dos de mayor/menor Z de ese lado
    edges = np.zeros((len(points), len(points)))
    for i, p in enumerate(points):
        n1 = normals[i]
        for j in range(i + 1, len(points)):
            p2 = points[j]
            n2 = normals[j]
            # considera como invalido ir de un punto a otro con normales opuestas
            # basicamente prohibido saltarse caras
            if np.all(n1 + n2 == np.zeros(3)):
                d = -1
            elif points_to_sections[str(p)] == points_to_sections[str(p2)]:
                d = 0
            else:
                # le agrega un costo a las distancias si tiene que cambiar normal (lado de la/s pala/s)
                d = dist(p, p2) + (turn_cost if np.all(n1 != n2) else 0)
            edges[i, j] = d
            edges[j, i] = d

    # Add costs from the initial node to each of the 22 nodes
    start_distances = np.array([dist(start_node, points[i]) for i in range(len(points))])
    # Agregar la fila
    edges = np.vstack([start_distances, edges])
    # Agregar la columna (asegúrate de que extra_col sea una columna)
    edges = np.hstack([np.hstack((0, start_distances)).reshape(-1, 1), edges])
    best_order, best_dist = solve_tsp(start_node, points, edges)
    print("Done")
    print(f"Best: order={str(best_order)}, dist={best_dist}")
    get_full_order(0, best_order, best_dist, points_str, points_to_sections)
    return np.array(points)[best_order], np.array(normals)[best_order]

def solve_tsp(start_node, points, edges):
    modifiedPoints = points.copy()
    modifiedPoints.insert(0, start_node)
    # Escalo edges y lo paso a ints
    edges = np.round(edges * 100).astype(int)    

    manager = pywrapcp.RoutingIndexManager(len(edges), 1, 0)
    routing = pywrapcp.RoutingModel(manager)
    
    
    # Create and register a transit callback

    # Create and register a transit callback
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return edges[from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    #search_parameters.first_solution_strategy = (
    #    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 5
    solution = routing.SolveWithParameters(search_parameters)
    
    
    # Print solution on console

    # Print solution on console
    if solution:
        index = routing.Start(0)
        route = []
        route_distance = 0
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        route.append(manager.IndexToNode(index))
        
        # Elimino el start_node de la solucion
        route = np.array(route)
        route_non_zero = route[route != 0]
        route_adjusted = route_non_zero - 1

        return route_adjusted, route_distance / 100
    else:
        return None, None

def grouping(obj):
    centers = obj.center
    number_of_groups = 3
    lowest_amount_of_triangles = centers.shape[0] / number_of_groups

    clf = KMeansConstrained(
        n_clusters=int(number_of_groups),
        size_min=lowest_amount_of_triangles,
        size_max=lowest_amount_of_triangles,
        random_state=0
    )
    clf.fit_predict(centers)

    result = []
    for i in range(int(number_of_groups)):
        sub_grp = []
        for node_index, label in enumerate(clf.labels_):
            if label == i:
                sub_grp.append(g[node_index])
        result.append(sub_grp)
    debug = 0
    return result

def get_full_order(m, test_order, dist, points_str, points_to_sections, start_node=None):
    resulting_points = []
    if start_node is not None:
        resulting_points.append(([start_node], get_random_color()))
        print(test_order)
    for index in test_order:
        p_str = points_str[index]
        section = points_to_sections[p_str]
        resulting_points.append(([section.get_point_from_str(p_str)], get_color_test(index)))
    plot_data_color_connected(resulting_points, f"m_{m}_d_{dist}_o_{str(test_order)}", dpi=300)
    debug = 0

def get_color_test(index, max_index=22):    
    normalized_index = min(max(index / max_index, 0), 1)
    red = int(255 * (1 - normalized_index))
    green = 0
    blue = 0
    return (red / 255, green / 255, blue / 255)

def plot_data_color(data, title, save=False, show=True, dpi=600):
    # now lets plot it!
    fig = plt.figure(dpi=dpi)
    ax = Axes3D(fig)
    ax.grid(False)
    ax.set_facecolor('white')
    for d_grp in data:
        d = np.array(d_grp).transpose()
        ax.plot(d[0], d[1], d[2], lw=2,
                c=(np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9)))
    ax.legend()
    # plt.xlim(-50, -110)
    plt.savefig(f'data/fig/{title}_plot.png')
    if show:
        plt.show()
        plt.clf()
    return ax


def plot_data_color_connected(data, title, save=False, show=True, dpi=600):
    # now lets plot it!
    plt.clf()
    fig = plt.figure(dpi=dpi)
    try:
        ax = Axes3D(fig, auto_add_to_figure=False)
        fig.add_axes(ax)
    except:
        ax = Axes3D(fig)
    ax.grid(False)
    ax.set_facecolor('white')
    last_point = None
    for d_grp, c in data:
        d = np.array(d_grp).transpose()
        if last_point is not None:
            last_point[0].append(d[0, 0])
            last_point[1].append(d[1, 0])
            last_point[2].append(d[2, 0])
            ax.plot(last_point[0], last_point[1], last_point[2], lw=2, c=color)
        color = c
        ax.plot(d[0], d[1], d[2], lw=2, c=color)
        last_point = [[d[0, -1]], [d[1, -1]], [d[2, -1]]]
    ax.legend()
    # plt.xlim(-50, -110)
    plt.savefig(f'data/fig/{title}_plot.png')
    if show:
        plt.show()
        plt.clf()
    return ax


def plot_data_color_sections(sections, title, save=False, show=True, dpi=600):
    # now lets plot it!
    fig = plt.figure(dpi=dpi)
    ax = Axes3D(fig)
    ax.grid(False)
    ax.set_facecolor('white')
    for d_grp in sections:
        d = d_grp.points.transpose()
        ax.plot(d[0], d[1], d[2], lw=1,
                c=(np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9)))
    ax.legend()
    # plt.xlim(-50, -110)
    plt.savefig(f'data/fig/{title}_plot.png')
    if show:
        plt.show()
        plt.clf()
    return ax

def write_shortest_path(info):
    # Directly from dictionary
    with open('data/out/shortest_path.json', 'w') as outfile:
        json.dump(info, outfile)

def midpoint(p1, p2):
    return (p1 + p2) / 2

def save_traj(filename, traj):
    if traj.shape[0] != 3:
        data = traj.transpose()
    else:
        data = traj
    writefile(f"data/out/{filename}_x.txt", data[0])
    writefile(f"data/out/{filename}_y.txt", data[1])
    writefile(f"data/out/{filename}_z.txt", data[2])

if __name__ == '__main__':
    seed = 21
    np.random.seed(seed)
    random.seed(seed)
    wt = Wireframe.from_stl_path('stl_gen/turbine.stl')
    # matriz de rotacion sobre su propio eje (intrinseca) en el eje Z
#    r = Rotation.from_euler("XYZ", [0, 0, 90], degrees=True).as_matrix() 

#    r = TRotation().set_matrix(r, "XYZ")
#    t = Transform(np.expand_dims(np.array([-80, 0, 20]), axis=1), r,
#                  translate_before_rotate=False)
#    wt = wt.transform(t)
    gps = grouping(wt)
    sections = []
    for i, (n, g) in enumerate(gps):
        sections.append(Section(g, n, get_distinct_color(i)))
#    plot_data_color_sections(sections, "te", True, False)

    start_node = np.array([0, -30, 30])
    traj, normals = shortest_path(start_node, sections, multiplier=10, turn_cost=0)
    save_traj(f"p162", traj)
    save_traj(f"n162", normals)

    debug = 0
