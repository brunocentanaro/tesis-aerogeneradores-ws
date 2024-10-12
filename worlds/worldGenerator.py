# generate_worlds.py

def generate_worlds(params_list, base_world_file):
    # Leer el contenido de baseWorld.sdf
    try:
        with open(base_world_file, 'r') as f:
            base_world_content = f.read()
    except FileNotFoundError:
        print(f"Error: No se pudo encontrar el archivo '{base_world_file}'.")
        return

    # Iterar sobre la lista de configuraciones
    for wind_config in params_list:
        wind = wind_config['wind']
        drone_configs = wind_config['droneConfigurations']

        # Verificar que hay al menos una configuración de dron
        if not drone_configs:
            print("Advertencia: No hay configuraciones de dron para este viento.")
            continue

        # Para cada configuración de dron, generar un mundo
        for drone_config in drone_configs:
            drone_position = drone_config['dronePosition']
            world_name = drone_config['nameToUse']
            output_file = f"{world_name}.sdf"

            # Generar el contenido del SDF
            sdf_content = f"""<sdf version='1.10'>
  <world name='{world_name}'>
    <wind>
      <linear_velocity>{wind}</linear_velocity>
    </wind>
    <include>
      <uri>model://x500_gimbal</uri>
      <name>x500_gimbal_0</name>
      <pose>{drone_position}</pose>
    </include>
{base_world_content}
  </world>
</sdf>
"""

            # Escribir el archivo SDF resultante
            with open(output_file, 'w') as f:
                f.write(sdf_content)

            print(f"Archivo SDF generado: {output_file}")


if __name__ == "__main__":
    params_list = [
        {
            'wind': "0 0 0",
            'droneConfigurations': [
                {
                    'dronePosition': "-68.86006 -7.48692 54.22478 0 0 -1.5557",
                    'nameToUse': "default"
                },
                {
                    'dronePosition': "45 -7.48692 48.72478 0 0 -1.5557",
                    'nameToUse': "default2"
                }
            ]
        },
        {
            'wind': "10 0 0",
            'droneConfigurations': [
                {
                    'dronePosition': "-68.86006 -7.48692 54.22478 0 0 -1.5557",
                    'nameToUse': "world_wind2_drone1"
                },
                {
                    'dronePosition': "10 10 20 0 0 0",
                    'nameToUse': "world_wind2_drone2"
                }
            ]
        }
    ]

    base_world_file = "baseWorld.sdf"

    generate_worlds(params_list, base_world_file)
