# generate_worlds.py

def generate_worlds(base_world_file, params_list, wind_turbine_list):
    # Read the contents of baseWorld.sdf
    generatedWorlds = []
    try:
        with open(base_world_file, 'r') as f:
            base_world_content = f.read()
    except FileNotFoundError:
        print(f"Error: Could not find file '{base_world_file}'.")
        return

    for i in range(len(wind_turbine_list)):
        windTurbinePosition = wind_turbine_list[i]
        isFrontInspection = i == 0
        for wind_config in params_list:
            wind = wind_config['wind']
            drone_configs = wind_config['droneConfigurations']

            # Verify that there is at least one drone configuration
            if not drone_configs:
                print("Warning: There are no drone settings for this wind.")
                continue

            # For each drone configuration, generate a world
            for drone_config in drone_configs:
                drone_position = drone_config['dronePosition']
                world_name = drone_config['nameToUse']
                suffix = "" if isFrontInspection else "_back"
                output_file = f"{world_name}{suffix}.sdf"
                generatedWorlds.append(output_file)

                # Generate the content of the SDF
                sdf_content = f"""<sdf version='1.10'>
<world name='{world_name}'>
    <wind>
    <linear_velocity>{wind}</linear_velocity>
    </wind>
    {base_world_content}
    <include>
    <uri>model://x500_gimbal</uri>
    <name>x500_gimbal_0</name>
    <pose>{drone_position}</pose>
    </include>
    <include>
    <uri>model://NuevoMolino</uri>
    <name>Molino_0</name>
    <pose>{windTurbinePosition}</pose>
    </include>
</world>
</sdf>
"""

                # Write the resulting SDF file
                with open(output_file, 'w') as f:
                    f.write(sdf_content)
                outputWithoutExtension = output_file.split(".")[0]
                print('  		' + outputWithoutExtension)


if __name__ == "__main__":
    wind_turbine_list = [
        '15.292486190795898 144.91380310058594 126.94837188720703 3.1186500007152751 -0.022236698983506227 -1.9153999947567857',
        '20.898960113525391 159.20082092285156 126.59645843505859 -3.1193099998709783 0.022901101362971601 1.1968399432586223'
    ]
    params_list = [
        {
            'wind': "0 0 0",
            'droneConfigurations': [
                {
                    'dronePosition': "-50.893397589860875 -7.2157147273725952 53.75795013669358 -2.347711240433879e-13 -5.8938645753208714e-08 -1.5556999413379522",
                    'nameToUse': "frontFarNoW"
                },
                {
                    'dronePosition': "43.375366204300967 94.119872188935915 47.612697250020688 -1.235685136484385e-13 -8.6219938620415493e-09 -1.5785300365654604",
                    'nameToUse': "rightNearNoW"
                },
                {
                    'dronePosition': "12.074535451124042 132.10949433237792 50 -0.10691173836875183 -0.02434792705363107 1.2608555326931348",
                    'nameToUse': "frontApproachInspectionReady"
                }
            ]
        },
        {
            'wind': "0 10 0",
            'droneConfigurations': [
                {
                    'dronePosition': "43.375366204300967 94.119872188935915 47.612697250020688 -1.235685136484385e-13 -8.6219938620415493e-09 -1.5785300365654604",
                    'nameToUse': "rightNearEW"
                }
            ]
        },
        {
            'wind': "10 0 0",
            'droneConfigurations': [
                {
                    'dronePosition': "-50.893397589860875 -7.2157147273725952 53.75795013669358 -2.347711240433879e-13 -5.8938645753208714e-08 -1.5556999413379522",
                    'nameToUse': "frontFarNW"
                },
                {
                    'dronePosition': "12.074535451124042 132.10949433237792 50 -0.10691173836875183 -0.02434792705363107 1.2608555326931348",
                    'nameToUse': "windyFrontApproachInspectionReady"
                }
            ]
        }
    ]

    base_world_file = "baseWorld.sdf"

    generate_worlds(base_world_file, params_list, wind_turbine_list)
