from drone_control.path_planner.stl_gen.create_stl import WindTurbine, create_stl
from drone_control.path_planner.shortest_path import shortest_path_from_stl, plot_best_order, plot_points

def path_planner(windTurbine:WindTurbine=WindTurbine(6, 39, "stl_gen/turbine"), start_position=[-10, 0, 0], end_position=None, safe_distance=0):
    create_stl(windTurbine)
    traj = shortest_path_from_stl(point_from_machine(start_position), point_from_machine(end_position), safe_distance, windTurbine.stl_path)
    points_only = [item[1] for item in traj]
    plot_best_order(points_only)
    plot_points(points_only)
    return result_to_machine(traj)

def point_from_machine(point):
    if point is None:
        return None
    x, y, z = point
    return [y, x, -z]

def result_to_machine(points):
    return [(i, (y, x, -z)) for i, (x, y, z) in points]

if __name__ == '__main__':
    path_planner()