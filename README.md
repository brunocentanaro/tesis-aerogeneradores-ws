# tesis-aerogeneradores-ws

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
