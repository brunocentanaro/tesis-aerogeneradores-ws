from drone_control.path_planner.stl_gen.create_stl import WindTurbine, create_stl
from drone_control.path_planner.shortest_path import shortest_path_from_stl, plot_best_order, plot_points

def path_planner(windTurbine: WindTurbine=WindTurbine(6, 39, "stl_gen/turbine"), start_position=[-10, 0, 0], safe_distance=0):
    create_stl(windTurbine)
    traj = shortest_path_from_stl(point_from_machine(start_position), safe_distance, windTurbine.stl_path)
    traj = points_to_machine(traj)
    plot_best_order(start_position, traj)
    #plot_points(traj)
    print(traj)
    return traj

def point_from_machine(point):
    x, y, z = point
    return [y, x, -z]

def points_to_machine(points):
    return [(y, x, -z) for x, y, z in points]

if __name__ == '__main__':
    path_planner()