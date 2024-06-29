# tesis-aerogeneradores-ws

## Tecnologías usadas

- Ubuntu 22.04
- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [Gazebo Garden](https://gazebosim.org/docs/garden/install)
- [PX4](https://gazebosim.org/docs/garden/install)

## Instrucciones

### Compilar

#### Si estamos corriendo por primera vez:

```
colcon build
```

#### Si ya compilamos los msj y ros com

```
colcon build --packages-select wind_turbine_detection wind_turbine_inspection
```

(solo los paquetes que hayamos cambiado)

### Correr

```
source install/setup.bash
ros2 launch wind_turbine_inspection wind_turbine_inspection.launch.py
```
