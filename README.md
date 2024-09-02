# tesis-aerogeneradores-ws

## Tecnologías usadas

- Ubuntu 22.04
- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [Gazebo Garden](https://gazebosim.org/docs/garden/install)
- [PX4](https://docs.px4.io/main/en/ros2/user_guide#installation-setup)

Despues de clonar el repositorio de PX4:
```
git checkout f8a42bcd58
git submodule update --init --recursive
```

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

En Tools/simulation/gz/models/OakD-Lite/model.sdf
Modificar donde dice

```
<clip>
    <near>0.1</near>
    <far>100</far>
</clip>
```

por

```
<clip>
    <near>0.1</near>
    <far>150</far>
</clip>
```

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
