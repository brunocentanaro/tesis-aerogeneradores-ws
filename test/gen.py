# Script para leer archivos t.txt y z.txt y generar wayPointsStack
# Asegúrate de que los archivos t.txt y z.txt están en el mismo directorio que este script

# Inicializa la lista de wayPointsStack
wayPointsStack = []

# Lee las posiciones en y del archivo t.txt
with open('y.txt', 'r') as t_file:
    y_positions = t_file.readlines()

# Lee las posiciones en z del archivo z.txt
with open('z.txt', 'r') as z_file:
    z_positions = z_file.readlines()

# Asegúrate de que ambos archivos tienen la misma cantidad de líneas
if len(y_positions) != len(z_positions):
    raise ValueError("Los archivos t.txt y z.txt no tienen la misma cantidad de líneas")

# Recorre ambas listas y añade las tuplas a wayPointsStack
for y, z in zip(y_positions, z_positions):
    y = float(y.strip())  # Convierte a float y elimina espacios en blanco
    z = float(z.strip())  # Convierte a float y elimina espacios en blanco
    wayPointsStack.append((0.0, y, z, 0.0))

# Exporta la lista de wayPointsStack a un archivo de texto
with open('waypoints.txt', 'w') as out_file:
    previousWaypoint = None
    for waypoint in wayPointsStack:
        if previousWaypoint is None:
            previousWaypoint = waypoint
            continue
        diffWaypoint = tuple(map(lambda x, y: x - y, waypoint, previousWaypoint))
        out_file.write(f"           self.wayPointsStack.append({diffWaypoint})\n")
        previousWaypoint = waypoint
