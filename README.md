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
pip install geopy
```

### Mundos y modelos

Copiar los modelos dentro de la instalacion de PX4 a
`Tools/simulation/gz/models`

Y los mundos
`Tools/simulation/gz/worlds`

En `Tools/simulation/gz/models/OakD-Lite/model.sdf` realizar las siguientes modificaciones:

```diff
<sensor name="IMX214" type="camera">
-	<pose>0.01233 -0.03 .01878 0 0 0</pose>
+	<pose>0.01233 -0.03 -0.02 0 0 0</pose>
    <camera>
        <horizontal_fov>1.204</horizontal_fov>
        <image>
        <width>1920</width>
        <height>1080</height>
        </image>
        <clip>
        <near>0.1</near>
-       <far>100</far>
+       <far>150</far>
        </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    <topic>camera</topic>
</sensor>
```

<sensor name="IMX214" type="camera">
        <pose>0.01233 -0.03 -0.02 0 0 0</pose>
        <camera>
          <horizontal_fov>1.204</horizontal_fov>
          <image>
          <format>R8G8B8</format>
             <format>R8G8B8</format>
           <width>1280</width>
            <height>960</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>200</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <topic>camera</topic>
      </sensor>

```diff
<sensor name="StereoOV7251" type="depth_camera">
-	<pose>0.01233 -0.03 .01878 0 0 0</pose>
+	<pose>0.01233 -0.03 -0.02 0 0 0</pose>
        <camera>
          <horizontal_fov>1.274</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R_FLOAT32</format>
          </image>
          <clip>
            <near>0.2</near>
            <far>19.1</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <topic>depth_camera</topic>
      </sensor>
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
