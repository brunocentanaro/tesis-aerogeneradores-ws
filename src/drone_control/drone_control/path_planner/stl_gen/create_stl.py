import numpy as np
import os

# Class representing a wind turbine with rotor diameter, blade length, and path to the STL file
class WindTurbine:
    def __init__(self, rotor_diameter, blade_length, stl_path):
        self.rotor_diameter = rotor_diameter  # Diameter of the rotor
        self.blade_length = blade_length      # Length of the blade
        self.stl_path = stl_path              # Path to save the STL file

# Function to create an STL file for a wind turbine
def create_stl(windTurbine: WindTurbine=WindTurbine(3, 10, "turbine")):
    w = windTurbine.rotor_diameter      # Center(hub) width, affects vertical blade width
    b_w = w                             # Blade width
    b_l = windTurbine.blade_length      # Blade length
    b_t = 0                             # Blade thickness
    t_c = w / 2                         

    m_s = 1.5
    facets_range = int(np.ceil(b_l / m_s))
    num_points = int(np.ceil((b_l + 10) / m_s))

    x = np.zeros((2000, 3))

    stl_file_path = windTurbine.stl_path + ".stl"
    stl_file_path = os.path.abspath(stl_file_path)
    stl_directory = os.path.dirname(stl_file_path)
    
    # Create the directory if it doesn't exist
    os.makedirs(stl_directory, exist_ok=True)
    print(f"Creating STL file at: {stl_file_path}")

    with open(stl_file_path, "w") as myfile:
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

        # Generate facets for one side
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

        # Generate points for the other side of the turbine
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

        # Generate points for the vertical part of the turbine
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

    return stl_file_path

if __name__ == '__main__':
    create_stl()