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
    points_str = []  # We dont want to convert np arrays on the fly so do it once here
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
    edges = np.zeros((len(points) + 1, len(points) + 1))
    for i, p in enumerate(points):
        n1 = normals[i]
        for j in range(i + 1, len(points)):
            p2 = points[j]
            n2 = normals[j]
            # considera como invalido ir de un punto a otro con normales opuestas
            # basicamente prohibido saltarse caras
            if np.all(n1 + n2 == np.zeros(3)):
                d = -1
            else:
                # le agrega un costo a las distancias si tiene que cambiar normal (lado de la/s pala/s)
                d = dist(p, p2) + (turn_cost if np.all(n1 != n2) else 0)
            edges[i, j] = d
            edges[j, i] = d
    best_order, best_dist = tsp_brute_force_with_start_node(edges, points, start_node)
    print("Done")
    print(f"Best: order={str(best_order)}, dist={best_dist}")
    return points[list(best_order)], normals[list(best_order)]

#def calculate_total_cost(permutation, costs, start_distances):
#    total_cost = 0
#    # Agregar el costo de ir del start_node al primer nodo en la permutación
#    total_cost += start_distances[permutation[0]]
#    # Calcular el costo del recorrido entre los nodos
#    for i in range(len(permutation) - 1):
#        total_cost += costs[permutation[i]][permutation[i + 1]]
#    return total_cost
#
#def tsp_brute_force_with_start_node(costs, points, start_node):
#    # Calcular las distancias desde el start_node a todos los otros nodos
#    start_distances = np.array([dist(start_node, points[i]) for i in range(1, len(points))])
#    nodes = list(range(len(points)))  # Nodos a permutar
#    # Generar todas las permutaciones posibles de los nodos
#    all_permutations = itertools.permutations(nodes)
#    # Inicializar variables para guardar la mejor ruta y su costo
#    min_cost = float('inf')
#    best_path = None
#    for permutation in all_permutations:
#        current_cost = calculate_total_cost(permutation, costs, start_distances)
#        if current_cost < min_cost:
#            min_cost = current_cost
#            best_path = permutation
#    return best_path, min_cost


def solve_tsp(edges, points, start_node, data):
    num_nodes = 22
    cost_matrix = np.zeros((num_nodes + 1, num_nodes + 1))

    # Fill in the distance matrix (you should replace these with your actual distances)
    for i in range(num_nodes):
        for j in range(num_nodes):
            cost_matrix[i + 1, j + 1] = distance_between_nodes[i][j]  # Replace with your actual distances

    # Add costs from the initial node to each of the 22 nodes
    initial_node_distances = np.array([distance_from_initial_to_node[i] for i in range(num_nodes)])
    cost_matrix[0, 1:] = initial_node_distances
    cost_matrix[1:, 0] = initial_node_distances

    """Solve the TSP problem."""
    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), 1, 0)
    
    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)
    
    # Create and register a transit callback
    def distance_callback(from_index, to_index):
        # Returns the distance between the two nodes.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    
    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)
    
    # Print solution on console
    if solution:
        index = routing.Start(0)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        
        # Compute the total distance
        total_distance = 0
        for i in range(len(route) - 1):
            total_distance += data['distance_matrix'][route[i]][route[i + 1]]
        total_distance += data['distance_matrix'][route[-1]][route[0]]  # Return to the start node
        
        return route, total_distance
    else:
        return None, None

def grouping(obj):
    centers = obj.center
    normals = obj.normals

    # Diccionario de normales como key y como valor array que contiene a todos los centros del triangulo con esa normal
    grp = {}
    string_to_normal = {}
    for i, n in enumerate(normals):
        n += np.zeros(3)
        n = np.round(n, 2)
        if str(n) not in grp:
            grp[str(n)] = []
            string_to_normal[str(n)] = n
        grp[str(n)].append(centers[i])

    # Array de duplas (normal, [centros con esa normal])
    grps = []
    # Minima cantidad de centros contenidos en un array que sea valor del diccionario grp
    lowest_amount_of_triangles = float('inf')
    for k, g in grp.items():
        #print("NUMERO: " + k + str(len(g)))
        if len(g) < lowest_amount_of_triangles:
            lowest_amount_of_triangles = len(g)
    for k, g in grp.items():
        if len(g) == lowest_amount_of_triangles:
            # The groups pointing up
            # En realidad tienen la misma cantidad de los que apuntan para el costado, pero los que apuntan para el costado le agregan uno mas en la parte de abajo, entonces los que apuntan para arriba son los mas chicos 
            grps.append((string_to_normal[k], g))
        elif len(g) == lowest_amount_of_triangles + 1:
            # the end of the turbines
            # Find the longest dist to between nodes:
            distance = np.full((len(g), len(g)), float("inf"))
            for i in range(len(g)):
                for j in range(i + 1, len(g)):
                    d = dist(g[i], g[j])
                    distance[i, j] = d
                    distance[j, i] = d
            # este es el que agregan los que apuntan para los costados, lo agregan al grouping pero separados del resto
            outlier_index = np.min(distance, axis=0).argmax()
            grps.append((string_to_normal[k], [g[outlier_index]]))
            del g[outlier_index]
            grps.append((string_to_normal[k], g))
        else:
            # estos serian los que faltan, que son los que apuntan para adelante y para atras
            number_of_groups = len(g) / lowest_amount_of_triangles
            # number_of_groups va a ser igual a 3, pq lowest_amount_of_triangles va a ser los que se necesitan para el lado de arriba de una pala, y los que van para adelante (y para atras) son 3 lados de pala
            np_g = np.array(g)
            clf = KMeansConstrained(
                n_clusters=int(number_of_groups),
                size_min=lowest_amount_of_triangles,
                size_max=lowest_amount_of_triangles,
                random_state=0
            )
            clf.fit_predict(np_g)

            for i in range(int(number_of_groups)):
                sub_grp = []
                for node_index, label in enumerate(clf.labels_):
                    if label == i:
                        sub_grp.append(g[node_index])
                grps.append((string_to_normal[k], sub_grp))

            debug = 0
    return grps


def plot_data_color(data, title, save=False, show=True, dpi=600):
    # now lets plot it!
    fig = plt.figure(dpi=dpi)
    ax = Axes3D(fig)
    ax.grid(False)
    ax.set_facecolor('white')
    for d_grp in data:
        d = np.array(d_grp).transpose()
        ax.plot(d[0], d[1], d[2], label='Original Global Path', lw=2,
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

    start_node = np.array([0, 0, 30])
    traj, normals = shortest_path(start_node, sections, multiplier=10, turn_cost=0)
#    plot_data_color_connected(traj, "title", save=False, show=True, dpi=600)
    save_traj(f"p162", np.array(traj))
    save_traj(f"n162", np.array(normals))

    debug = 0
