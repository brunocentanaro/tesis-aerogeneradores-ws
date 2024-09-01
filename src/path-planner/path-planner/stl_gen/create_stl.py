import numpy as np

class WindTurbine:
    def __init__(self, tower_height, rotor_diameter, blade_length, stl_name):
        self.tower_height = tower_height
        self.rotor_diameter = rotor_diameter
        self.blade_length = blade_length
        self.stl_name = stl_name

def create_stl(windTurbine: WindTurbine=WindTurbine(80, 3, 10, "turbine2")):
    w = 3      # center(hub) width, affects vertical blade width
    b_w = 3                             # blade width
    b_l = 40      # blade length
    b_t = 0                             # blade thickness
    t_c = 80 + 3  # height from vertical blade base to the ground

    m_s = 1.5
    facets_range = int(np.ceil(b_l / m_s))
    num_points = int(np.ceil((b_l + 10) / m_s))

    x = np.zeros((2000, 3))
    print(windTurbine.stl_name + ".stl")
    with open(windTurbine.stl_name + ".stl", "w") as myfile:
        myfile.write("solid AssimpScene\n")

        cc = 0
        x[cc] = [w / 2, b_t / 2, t_c]
        x[cc + 1] = [w / 2, b_t / 2, t_c - b_w]

        for k in range(1, num_points):
            x[cc + 2 * k] = [x[cc + 2 * k - 2][0] + m_s * np.cos(np.pi / 5),
                            x[cc + 2 * k - 2][1],
                            x[cc + 2 * k - 2][2] - m_s * np.sin(np.pi / 5)]
            x[cc + 2 * k + 1] = [x[cc + 2 * k - 1][0] + m_s * np.cos(np.pi / 5),
                                x[cc + 2 * k - 1][1],
                                x[cc + 2 * k - 1][2] - m_s * np.sin(np.pi / 5)]

        # Genera las facetas para un lado
        for k in range(facets_range):
            myfile.write("facet normal 0 1 0\n")
            myfile.write("outer loop\n")
            myfile.write(f"vertex {x[cc + 2 * k][0]} {x[cc + 2 * k][1]} {x[cc + 2 * k][2]}\n")
            myfile.write(f"vertex {x[cc + 2 * k + 2][0]} {x[cc + 2 * k + 2][1]} {x[cc + 2 * k + 2][2]}\n")
            myfile.write(f"vertex {x[cc + 2 * k + 1][0]} {x[cc + 2 * k + 1][1]} {x[cc + 2 * k + 1][2]}\n")
            myfile.write("endloop\n")
            myfile.write("endfacet\n")

        for k in range(facets_range):
            myfile.write("facet normal 0 1 0\n")
            myfile.write("outer loop\n")
            myfile.write(f"vertex {x[cc + 2 * k + 1][0]} {x[cc + 2 * k + 1][1]} {x[cc + 2 * k + 1][2]}\n")
            myfile.write(f"vertex {x[cc + 2 * k + 2][0]} {x[cc + 2 * k + 2][1]} {x[cc + 2 * k + 2][2]}\n")
            myfile.write(f"vertex {x[cc + 2 * k + 3][0]} {x[cc + 2 * k + 3][1]} {x[cc + 2 * k + 3][2]}\n")
            myfile.write("endloop\n")
            myfile.write("endfacet\n")

        # Genera puntos para el otro lado de la turbina
        cc = 500
        x[cc] = [-w / 2, b_t / 2, t_c]
        x[cc + 1] = [-w / 2, b_t / 2, t_c - b_w]

        for k in range(1, num_points):
            x[cc + 2 * k] = [x[cc + 2 * k - 2][0] - m_s * np.cos(np.pi / 5),
                            x[cc + 2 * k - 2][1],
                            x[cc + 2 * k - 2][2] - m_s * np.sin(np.pi / 5)]
            x[cc + 2 * k + 1] = [x[cc + 2 * k - 1][0] - m_s * np.cos(np.pi / 5),
                                x[cc + 2 * k - 1][1],
                                x[cc + 2 * k - 1][2] - m_s * np.sin(np.pi / 5)]

        for k in range(facets_range):
            myfile.write("facet normal 0 1 0\n")
            myfile.write("outer loop\n")
            myfile.write(f"vertex {x[cc + 2 * k][0]} {x[cc + 2 * k][1]} {x[cc + 2 * k][2]}\n")
            myfile.write(f"vertex {x[cc + 2 * k + 1][0]} {x[cc + 2 * k + 1][1]} {x[cc + 2 * k + 1][2]}\n")
            myfile.write(f"vertex {x[cc + 2 * k + 2][0]} {x[cc + 2 * k + 2][1]} {x[cc + 2 * k + 2][2]}\n")
            myfile.write("endloop\n")
            myfile.write("endfacet\n")

        for k in range(facets_range):
            myfile.write("facet normal 0 1 0\n")
            myfile.write("outer loop\n")
            myfile.write(f"vertex {x[cc + 2 * k + 1][0]} {x[cc + 2 * k + 1][1]} {x[cc + 2 * k + 1][2]}\n")
            myfile.write(f"vertex {x[cc + 2 * k + 3][0]} {x[cc + 2 * k + 3][1]} {x[cc + 2 * k + 3][2]}\n")
            myfile.write(f"vertex {x[cc + 2 * k + 2][0]} {x[cc + 2 * k + 2][1]} {x[cc + 2 * k + 2][2]}\n")
            myfile.write("endloop\n")
            myfile.write("endfacet\n")

        # Genera puntos para la parte vertical de la turbina
        cc = 0
        x[cc] = [-w / 2, b_t / 2, t_c]
        x[cc + 1] = [w / 2, b_t / 2, t_c]

        for k in range(1, num_points):
            x[cc + 2 * k] = [x[cc + 2 * k - 2][0],
                            x[cc + 2 * k - 2][1],
                            x[cc + 2 * k - 2][2] + m_s * np.sin(np.pi / 2)]
            x[cc + 2 * k + 1] = [x[cc + 2 * k - 1][0],
                                x[cc + 2 * k - 1][1],
                                x[cc + 2 * k - 1][2] + m_s * np.sin(np.pi / 2)]

        for k in range(facets_range):
            myfile.write("facet normal 0 1 0\n")
            myfile.write("outer loop\n")
            myfile.write(f"vertex {x[cc + 2 * k][0]} {x[cc + 2 * k][1]} {x[cc + 2 * k][2]}\n")
            myfile.write(f"vertex {x[cc + 2 * k + 2][0]} {x[cc + 2 * k + 2][1]} {x[cc + 2 * k + 2][2]}\n")
            myfile.write(f"vertex {x[cc + 2 * k + 1][0]} {x[cc + 2 * k + 1][1]} {x[cc + 2 * k + 1][2]}\n")
            myfile.write("endloop\n")
            myfile.write("endfacet\n")

        for k in range(facets_range):
            myfile.write("facet normal 0 1 0\n")
            myfile.write("outer loop\n")
            myfile.write(f"vertex {x[cc + 2 * k + 1][0]} {x[cc + 2 * k + 1][1]} {x[cc + 2 * k + 1][2]}\n")
            myfile.write(f"vertex {x[cc + 2 * k + 2][0]} {x[cc + 2 * k + 2][1]} {x[cc + 2 * k + 2][2]}\n")
            myfile.write(f"vertex {x[cc + 2 * k + 3][0]} {x[cc + 2 * k + 3][1]} {x[cc + 2 * k + 3][2]}\n")
            myfile.write("endloop\n")
            myfile.write("endfacet\n")

        myfile.write("endsolid AssimpScene\n")

if __name__ == '__main__':
    print("hola")
    create_stl()