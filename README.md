# B-BoT
### Simulación de reconocimiento de objetos cotidianos para asistentes robóticos especializados en demencia senil.

<br>

>Lanzar la simulación con Gazebo

```
$ catkin build
$ source devel/setup.bash
$ roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=[steel|titanium] world:=[_world]
```

<br>
**Ejemplo: TIAGo en el entorno 1 de obtención de imágenes para el dataset:**

```
$ roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel world:=get_dataset_objects01
```

![alt text](doc/simulation02.jpg "TIAGo en el entorno de dataset 1")

<br>

>Lanzar el nodo **take_images_node**

```
$ catkin build take_images
$ source devel/setup.bash
$ roslaunch take_images take_images_launch_file.launch
```

<br>

>Lanzar el nodo **yolo_detection_node**

```
$ catkin build yolo_detection_obj
$ source devel/setup.bash
$ roslaunch yolo_detection_obj yolo_detection_launch_file.launch
```

<br>

>Lanzar el nodo **navigation_node**

*Terminal 1*
```
$ catkin build yolo_detection_obj
$ source devel/setup.bash
$ roslaunch navigation mapping_public.launch
```

*Terminal 2*
```
$ rosrun key_teleop key_teleop.py
```

Cuando se haya finalizado el *mapeado* presionamos `q` y guardamos el mapa con el siguiente comando:

```
$ rosservice call /pal_map_manager/save_map "directory: ''"
```
El servicio guardará el mapa en la siguiente ruta:

```
~/.pal/tiago_maps/config
```

*Terminal 1*
```
$ source devel/setup.bash
$ roslaunch navigation navigation_public.launch
```

*Terminal 2*
```
$ source devel/setup.bash
$ rosservice call /global_localization "{}"
$ rosrun key_teleop key_teleop.py
```

*Terminal 3*
```
$ source devel/setup.bash
$ rosservice call /move_base/clear_costmaps "{}"
```

Cuando se haya finalizado la *localización* presionamos `q` y ...

