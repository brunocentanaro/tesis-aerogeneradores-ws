# Inspección Automática de Aerogeneradores con Drones

Este repositorio contiene el código desarrollado durante la realización del proyecto de grado para la carrera de Ingeniería en Computación de la Facultad de Ingeniería (Fing) de la Universidad de la República (UdelaR).

El trabajo se centra en implementar un sistema para la inspección de las palas de un aerogenerador utilizando drones autónomos. 

El control del dron se gestiona a través del firmware PX4. 
El posicionamiento del dron frente al rotor, asegurando que esté centrado y ortogonal, se logra mediante visión por computadora (computer vision) utilizando la transformada probabilística de Hough proporcionada por la librería OpenCV para detectar los componentes del aerogenerador. Además, para la planificación del trayecto del dron durante la inspección, se resolvió un Problema del Viajante Asimétrico (Asymmetric TSP) utilizando Google OR-Tools, lo que permite optimizar la ruta más corta para inspeccionar las palas de manera eficiente.

### Autores
- Carolina Acosta
- Bruno Cantanaro
- Martín Tapia

## Contenido
- [Requerimientos](#requerimientos)
- [Set up del proyecto](#set-up-del-proyecto)
  - [Mundos y modelos](#mundos-y-modelos)
  - [Modificaciones Necesarias en el Modelo OakD-Lite](#modificaciones-necesarias-en-el-modelo-oakd-lite)
- [Compilación](#compilación)
  - [Compilación de Paquetes Específicos](#compilación-de-paquetes-específicos)
- [Ejecución](#ejecución)
- [Pruebas](#pruebas)
- [Nodos relevantes](#nodosmodulos-relevantes)
- [Informe](#informe)

## Requerimientos
- Ubuntu 22.04
- ROS2 Humble: [Guía de instalación](https://docs.ros.org/en/humble/Installation.html)
- Gazebo Garden: [Guía de instalación](https://gazebosim.org/docs/garden/install)
- PX4: [Guía de instalación](https://docs.px4.io/main/en/ros2/user_guide#installation-setup)

Después de clonar el repositorio de PX4, es necesario posicionarse en un commit específico para evitar breaking changes introducidos en versiones posteriores. Para ello, ejecuta el siguiente comando:
```bash
git checkout f8a42bcd58
```

Para agregar los cambios introducidos por el equipo que permiten hacer funcional el gimbal del dron, sigue estos pasos:
1. Copia el archivo `add_gimbal_changes.patch` dentro de la carpeta `/PX4-Autopilot`.

2. Desde el directorio `/PX4-Autopilot`, ejecuta el siguiente comando para aplicar los cambios:

```bash
git apply add_gimbal_changes.patch
```

3. Finalmente, ejecuta el siguiente comando para actualizar los submódulos necesarios:

```bash
git submodule update --init --recursive
```

- QGroundControl: [Guía de instalación](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu)

Una vez descargado el archivo `QGroundControl.AppImage`, colócalo en tu carpeta home (`~/`).

## Set up del proyecto

Instalar el paquete `ros-humble-ros-gzgarden` para agregar herramientas que permiten la integración de ROS 2 Humble con el simulador Gazebo Garden, ejecutando el siguiente comando:

```bash
sudo apt install ros-humble-ros-gzgarden
```

Instalar paquetes de python:

``` bash
pip install mavsdk aioconsole pygame numpy opencv-python ultralytics geopy ortools numpy-stl k-means-constrained
```

### Mundos y modelos

Para configurar los modelos y mundos en PX4, sigue estos pasos:

1. Coloca los modelos en el directorio correspondiente dentro de la instalación de PX4:
```bash
/PX4-Autopilot/Tools/simulation/gz/models
```
esto incluye el modelo del dron y del aerogenerador.

2. Para generar los mundos copiar los archivos `default.sdf`, `baseWorld.sdf` y `worldGenerator.py` de la carpeta worlds del proyecto a `/PX4-Autopilot/Tools/simulation/gz/worlds`.
En ese directorio ejecutar:

```bash
python3 worldGenerator.py
```

### Modificaciones Necesarias en el Modelo OakD-Lite
Accede al archivo de configuración del modelo `OakD-Lite` y realiza las siguientes modificaciones en `/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf`:

```diff
<sensor name="IMX214" type="camera">
-	<pose>0.01233 -0.03 .01878 0 0 0</pose>
+	<pose>0.01233 -0.03 -0.042 0 0 0</pose>
    <camera>
        <horizontal_fov>1.204</horizontal_fov>
        <image>
+         <format>R8G8B8</format>
+         <format>R8G8B8</format>
-         <width>1920</width>
-         <height>1080</height>
+         <width>1280</width>
+         <height>960</height>
        </image>
        <clip>
        <near>0.1</near>
-       <far>100</far>
+       <far>200</far>
        </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    <topic>camera</topic>
</sensor>
```

```diff
<sensor name="StereoOV7251" type="depth_camera">
-	<pose>0.01233 -0.03 .01878 0 0 0</pose>
+	<pose>0.01233 -0.03 -0.042 0 0 0</pose>
        <camera>
          <horizontal_fov>1.274</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R_FLOAT32</format>
          </image>
          <clip>
-            <near>0.2</near>
-            <far>19.1</far>
+            <near>8</near>
+            <far>80</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <topic>depth_camera</topic>
      </sensor>
```

## Compilación

Para compilar todo el proyecto utiliza el siguiente comando:
```bash
colcon build
```

### Compilación de Paquetes Específicos
Si ya has compilado anteriormente y solo necesitas recompilar los paquetes modificados, utiliza:
```bash
colcon build --packages-select wind_turbine_detection wind_turbine_inspection
```

(Asegúrate de incluir solo los paquetes que has cambiado.)


## Ejecución
Una vez completada la compilación, ejecuta el proyecto con los siguientes comandos:

```bash
source install/setup.bash
ros2 launch wind_turbine_inspection wind_turbine_inspection.launch.py mission_arg:=0 front_inspection:=1 register_after_takeoff:=0
```
Explicación de los parámetros del launch:
- mission_arg indica que molino se quiere inspeccionar
- front_inspection indica si se va realizar la inspeccion delantera o trasera
- register_after_takeoff indica si se quiere registrar inmediatamente despues del takeoff (sin acercarse y colocarse ortogonal al molino)
Estoas parámetros son especialmente útiles para poder probar partes de la misión.

Para indicar al dron que inicie la misión y también cuando se quiere indicar que se giraron las aspas:
```bash
ros2 service call /comenzar_inspeccion std_srvs/srv/Trigger
```

## Pruebas

Se debe indicar en el testing_helper.py en que estados quiero comenzar y terminar de recabar datos.

Para que la simulacion sea headless:
```bash
HEADLESS=1 make px4_sitl gz_x500_gimbal
```


TODO

## Nodos/modulos relevantes
A continueción se describen los nodos o modulos más relevantes.

**drone_control/shortest_path**:
Es un módulo que genera un modelo STL de una turbina eólica y calcula la trayectoria más corta desde una posición inicial hasta una final, teniendo en cuenta una distancia de seguridad.

**drone_control:control**:
Nodo dedicado al control del dron.
Implementa la comunicación bidireccional con el firmware PX4, incluyendo comandos de modo de vuelo, puntos de trayectoria y comandos de vehículo. Publica setpoints de posición y velocidad en tópicos específicos, mientras que también se suscribe a los tópicos de PX4 para obtener información crítica del estado del dron. Además, implementa un sistema de waypoints que permite controlar el movimiento del dron en función de su orientación actual, y tiene la capacidad de enviar setpoints de corrección en tiempo real para adaptarse a cambios en el entorno.

**wind_turbine_inspection:mission_state_handler**:
Es el nodo encargado de gestionar una serie de estados en un ciclo de operación.

**wind_turbine_detection:image_subscriber**:
Se encarga de recibir imágenes de un sensor LiDAR y realizar un procesamiento basado en el estado actual del sistema.
En el caso de que se esté en el estado `OrthogonalAlignmentState` se detecta las aspas y el rotor para poder centrar el rotor y colocar el dron ortogonal al molino.
Si el estado es `RegistrationState` se detecta el centroide del aspa para poder centrar el mismo en el frame y que el aspa no se salga del frame.

## Informe
Para más información, el informe se encuentra en [documentos](/documents).