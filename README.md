# tesis-aerogeneradores-ws

## Tecnologías usadas

- Ubuntu 22.04
- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [Gazebo Garden](https://gazebosim.org/docs/garden/install)
- [PX4](https://gazebosim.org/docs/garden/install)

## Instrucciones

### Instalar QGroundControl

https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu

y ponerlo en la carpeta del home

### Dependencias python

Correr en la terminal

```
pip install mavsdk
pip install aioconsole
pip install pygame
sudo apt install ros-humble-ros-gzgarden
pip install numpy
pip install opencv-python
pip install ultralytics
```

### Mundos y modelos

Copiar los modelos dentro de la instalacion de PX4 a
Tools/simulation/gz/models

Y los mundos
Tools/simulation/gz/worlds

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
