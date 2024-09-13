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

class Section:
    def __init__(self, points, safe_distance=0):
        self.points = np.array(points) + np.array([0,1,0]) * safe_distance
        
        conteo_x = Counter(self.points[:, 0])
        print(conteo_x)
        if any(conteo >= 2 for conteo in conteo_x.values()):
            indices = np.argsort(self.points[:, 2])
        else:
            indices = np.argsort(self.points[:, 0])
        self.points = self.points[indices]
        self.p1 = midpoint(self.points[0], self.points[1])  # lowest point (no need to sort)
        self.p2 = midpoint(self.points[-1], self.points[-2])  # highest point (need to reorder list)
        self.p1_str = str(self.p1)
        self.p2_str = str(self.p2)

    def get_ordered_points(self, p_str):
        if p_str == self.p1_str:
            return [self.p1, self.p2]
        return [self.p2, self.p1]
    
    def get_point_from_str(self, p_str):
        if p_str == self.p1_str:
            return self.p1
        return self.p2
    
def midpoint(p1, p2):
    return (p1 + p2) / 2

def dist(p_to, p_from=np.array([0, 0, 0])):
    v = p_to - p_from
    return np.linalg.norm(v)

def shortest_path_from_stl(start_node, safe_distance, stl_name):
    wt = Wireframe.from_stl_path(stl_name + '.stl')
    gps = grouping(wt)
    sections = []
    for g in gps:
        sections.append(Section(g, safe_distance))
    return shortest_path(np.array(start_node), sections)

def shortest_path(start_node, sections: List[Section]):
    points = []
    points_to_sections = {}
    for s in sections:
        points.append(s.p1)
        points.append(s.p2)
        points_to_sections[s.p1_str] = s
        points_to_sections[s.p2_str] = s
    edges = np.zeros((len(points), len(points)))
    for i, p in enumerate(points):
        for j in range(i + 1, len(points)):
            p2 = points[j]
            if points_to_sections[str(p)] == points_to_sections[str(p2)]:
                d = 0
            else:
                d = dist(p, p2)
            edges[i, j] = d
            edges[j, i] = d
    start_distances = np.array([dist(start_node, points[i]) for i in range(len(points))])
    edges = np.vstack([start_distances, edges])
    edges = np.hstack([np.hstack((0, start_distances)).reshape(-1, 1), edges])
    best_order, best_dist = solve_tsp(start_node, points, edges)
    print(f"Best: order={str(best_order)}, dist={best_dist}")
    return np.array(points)[best_order]

def solve_tsp(start_node, points, edges):
    modifiedPoints = points.copy()
    modifiedPoints.insert(0, start_node)
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
        route.append(manager.IndexToNode(index))
        
        route = np.array(route)
        route_non_zero = route[route != 0]
        route_adjusted = route_non_zero - 1
        return route_adjusted, route_distance / 100
    else:
        return None, None

def grouping(obj):
    vertices = obj.center
    number_of_groups = 3
    lowest_amount_of_triangles = vertices.shape[0] / number_of_groups

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
    debug = 0
    return result

def plot_best_order(start_node, points):
    resulting_points = []
    if start_node is not None:
        resulting_points.append(([start_node], (173/255, 216/255, 230/255)))
    for i, point in enumerate(points):
        resulting_points.append(([point], get_color_test(i)))
    plot_data_color_connected(resulting_points, "order", dpi=300)
    debug = 0

def get_color_test(index, max_index=5):    
    normalized_index = min(max(index / max_index, 0), 1)
    red = int(255 * (1 - normalized_index))
    green = 0
    blue = 0
    return (red / 255, green / 255, blue / 255)

def plot_data_color_connected(data, title, save=False, show=True, dpi=600):
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

def plot_points(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='r', marker='o')
    ax.set_xlabel('Eje X')
    ax.set_ylabel('Eje Y')
    ax.set_zlabel('Eje Z')
    plt.show()

def writefile(filename, data):
    with open(filename, 'w') as file:
        for data_point in data:
            L = [f"{str(data_point)}\n"]
            file.writelines(L)

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
    # matriz de rotacion sobre su propio eje (intrinseca) en el eje Z
#    r = Rotation.from_euler("XYZ", [0, 0, 90], degrees=True).as_matrix() 

#    r = TRotation().set_matrix(r, "XYZ")
#    t = Transform(np.expand_dims(np.array([-80, 0, 20]), axis=1), r,
#                  translate_before_rotate=False)
#    wt = wt.transform(t)
    gps = grouping(wt)
    sections = []
    for i, g in enumerate(gps):
        sections.append(Section(g))

    start_node = np.array([0, -30, 30])
    traj = shortest_path(start_node, sections)
    plot_best_order(start_node, traj)
    plot_points(traj)
    save_traj(f"p162", traj)
    debug = 0