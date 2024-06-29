# tesis-aerogeneradores-ws

## Instrucciones

### Si estamos corriendo por primera vez:

1. colcon build

### Si ya compilamos los msj y ros com

1. colcon build --packages-select wind_turbine_detection wind_turbine_inspection (solo los paquetes que hayamos cambiado)

2. source install/setup.bash
3. ros2 launch wind_turbine_inspection wind_turbine_inspection.launch.py
