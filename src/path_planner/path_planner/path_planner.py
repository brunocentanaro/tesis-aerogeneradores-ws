from stl_gen.create_stl import WindTurbine, create_stl
from shortest_path import shortest_path_from_stl, plot_best_order, plot_points

def path_planner(windTurbine: WindTurbine=WindTurbine(3, 40, "stl_gen/turbine"), start_position=[0, -30, 30], safe_distance=10):
    create_stl(windTurbine)
    start_position = point_from_machine(start_position)
    traj = shortest_path_from_stl(start_position, windTurbine.stl_path, safe_distance)
    traj = points_to_machine(traj)
    plot_best_order(start_position, traj)
    plot_points(traj)
    return traj

def point_from_machine(point):
    y, x, z = point
    return [x, y, -z]

def points_to_machine(points):
    return [(y, x, -z) for x, y, z in points]

if __name__ == '__main__':
    path_planner()