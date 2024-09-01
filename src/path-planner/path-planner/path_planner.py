from stl_gen.create_stl import WindTurbine, create_stl
from shortest_path import shortest_path_from_stl, plot_best_order, plot_points

def path_planner(windTurbine: WindTurbine=WindTurbine(80, 3, 10, "turbine"), start_position=[0, -30, 30]):
    create_stl(windTurbine)
    traj = shortest_path_from_stl(start_position, windTurbine.stl_name)
    plot_best_order(start_position, traj)
    plot_points(traj)
    return traj

if __name__ == '__main__':
    path_planner()