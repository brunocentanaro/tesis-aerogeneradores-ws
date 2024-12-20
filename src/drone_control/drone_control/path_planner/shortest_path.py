import numpy as np
from collections import Counter
from ortools.constraint_solver import routing_enums_pb2, pywrapcp
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List

from drone_control.path_planner.objects.mesh_base import Wireframe
from k_means_constrained import KMeansConstrained

cache = {}
data = {}
use_cache = True

# Section class to represent a segment of points in the path
class Section:
    def __init__(self, points, safe_distance=0, angle_y=0, angle_z=0):
        self.points = np.array(points) + np.array([0,1,0]) * safe_distance
        
        # Sort points
        conteo_x = Counter(self.points[:, 0])
        print(conteo_x)
        if any(conteo >= 2 for conteo in conteo_x.values()):
            indices = np.argsort(self.points[:, 2]) # Sort by z if there's repetition in x
        else:
            indices = np.argsort(self.points[:, 0]) # Otherwise, sort by x
        self.points = self.points[indices]
        
        # Define points p1 and p2 (lowest and highest points)
        self.p1 = self.rotate_point(midpoint(self.points[0], self.points[1]), angle_y, angle_z)  # lowest point (no need to sort)
        self.p2 = self.rotate_point(midpoint(self.points[-1], self.points[-2]), angle_y, angle_z)  # highest point (need to reorder list)
        self.p1_str = str(self.p1)
        self.p2_str = str(self.p2)

    # Function to rotate a point by given angles
    def rotate_point(self, point, angle_y, angle_z):
        angle_y_radians = np.radians(angle_y)
        angle_z_radians = np.radians(angle_z)

        rotation_matrix_y = np.array([
            [np.cos(angle_y_radians), 0, np.sin(angle_y_radians)],
            [0, 1, 0],
            [-np.sin(angle_y_radians), 0, np.cos(angle_y_radians)]
        ])

        rotation_matrix_z = np.array([
            [np.cos(angle_z_radians), -np.sin(angle_z_radians), 0],
            [np.sin(angle_z_radians), np.cos(angle_z_radians), 0],
            [0, 0, 1]
        ])
        rotation_matrix = np.dot(rotation_matrix_z, rotation_matrix_y)
        return np.dot(rotation_matrix, point)

# Calculates the midpoint of two points
def midpoint(p1, p2):
    return (p1 + p2) / 2

# Euclidean distance between two points
def dist(p_to, p_from=np.array([0, 0, 0])):
    v = p_to - p_from
    return np.linalg.norm(v)

# Finds the shortest path from STL data
def shortest_path_from_stl(start_node, end_node, safe_distance, angle_y, angle_z, stl_name):
    wt = Wireframe.from_stl_path(stl_name)
    gps = grouping(wt)
    sections = []
    for g in gps:
        sections.append(Section(g, safe_distance, angle_y, angle_z))
    return shortest_path(start_node, sections, end_node)

# Computes the shortest path through all sections
def shortest_path(start_node, sections: List[Section], end_node):
    points = [np.array(start_node)]
    points_to_sections = {}
    points_to_sections[0] = None
    for s in sections:
        points.append(s.p1)
        points_to_sections[len(points) - 1] = s
        points.append(s.p2)
        points_to_sections[len(points) - 1] = s
    if end_node is not None:
        points.append(np.array(end_node)) 
        points_to_sections[len(points) - 1] = None
    
    # Initialize the distance matrix (edges)
    edges = np.zeros((len(points), len(points)))
    for i in range(len(points)):
        for j in range(len(points)):
            if j == 0 or (points_to_sections[i] == points_to_sections[j]): # (j is starting point) or (i and j belong to the same section)
                edges[i, j] = 0
            elif i == len(points) - 1 and end_node is not None: # i is the last point
                edges[i, j] = 1000000 # infinite
            else:
                edges[i, j] = dist(points[i], points[j])
    print(edges)

    # Solve the traveling salesman problem to find the best order
    best_order, best_dist = solve_tsp(edges)
    print(f"Best: order={str(best_order)}, dist={best_dist}")
    return np.array([(id(points_to_sections[index]), value) for index, value in enumerate(points)], dtype=object)[best_order]

# Function to solve the TSP using Google OR-Tools
def solve_tsp(edges):
    edges = np.round(edges * 100).astype(int)    

    manager = pywrapcp.RoutingIndexManager(len(edges), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return edges[from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    solution = routing.SolveWithParameters(search_parameters)
    
    if solution:
        index = routing.Start(0)
        route = []
        route_distance = 0
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        
        route = np.array(route)
        return route, route_distance / 100
    else:
        return None, None

# Groups vertices into clusters
def grouping(obj):
    vertices = obj.center
    number_of_groups = 3
    lowest_amount_of_triangles = vertices.shape[0] / number_of_groups

    # Apply KMeans clustering with constraints on cluster size
    clf = KMeansConstrained(
        n_clusters=int(number_of_groups),
        size_min=lowest_amount_of_triangles,
        size_max=lowest_amount_of_triangles,
        random_state=0
    )
    clf.fit_predict(vertices)

    result = []
    for i in range(int(number_of_groups)):
        sub_grp = []
        for node_index, label in enumerate(clf.labels_):
            if label == i:
                sub_grp.append(vertices[node_index])
        result.append(sub_grp)
    return result

# Function to plot the best order of points in 3D space
def plot_best_order(points):
    resulting_points = []
    for i, point in enumerate(points):
        resulting_points.append(([point], get_color_test(i)))
    plot_data_color_connected(resulting_points, "order", dpi=300)

# Helper function to generate color for each index in a range
def get_color_test(index, max_index=5):    
    normalized_index = min(max(index / max_index, 0), 1)
    red = int(255 * (1 - normalized_index))
    green = 0
    blue = 0
    return (red / 255, green / 255, blue / 255)

# Function to plot data points in 3D space with color and connections
def plot_data_color_connected(data, title, show=True, dpi=600):
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
    ax.set_xlabel('Eje X')
    ax.set_ylabel('Eje Y')
    ax.set_zlabel('Eje Z')
    plt.savefig(f'data/{title}_plot.png')
    if show:
        plt.show()
        plt.clf()
    return ax

# Plots the points in a 3D space
def plot_points(points):
    points_array = np.array(points)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points_array[:, 0], points_array[:, 1], points_array[:, 2], c='r', marker='o')
    ax.set_xlabel('Eje X')
    ax.set_ylabel('Eje Y')
    ax.set_zlabel('Eje Z')
    plt.show()

# Writes the data to a text file, one data point per line
def writefile(filename, data):
    with open(filename, 'w') as file:
        for data_point in data:
            L = [f"{str(data_point)}\n"]
            file.writelines(L)

# Saves the trajectory data to three separate text files (for x, y, and z coordinates)
def save_traj(filename, traj):
    if traj.shape[0] != 3:
        data = traj.transpose()
    else:
        data = traj
    writefile(f"data/out/{filename}_x.txt", data[0])
    writefile(f"data/out/{filename}_y.txt", data[1])
    writefile(f"data/out/{filename}_z.txt", data[2])

if __name__ == '__main__':
    wt = Wireframe.from_stl_path('stl_gen/turbine.stl')
    gps = grouping(wt)
    sections = []
    for i, g in enumerate(gps):
        sections.append(Section(g))
    start_node = np.array([0, -30, 30])
    traj = shortest_path(start_node, sections)
    plot_best_order(start_node, traj)
    plot_points(traj)
    save_traj(f"p162", traj)