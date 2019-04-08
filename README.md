# B-BoT
#### Simulación de reconocimiento de objetos cotidianos para asistentes robóticos especializados en demencia senil.

1. Introducción
2. TIAGo: Simulación en Gazebo
3. Nodos ROS
   1. TakeImageNode
   2. YoloDetectionNode
   3. NavigationNode
   4. MoveToPointNode
   5. GetGazeboObjNode
   6. GripperGraspingNode


<br>

## Introducción


<br>

## TIAGo: Simulación en Gazebo

>Lanzar la simulación en Gazebo

```
$ catkin build
$ source devel/setup.bash
$ roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=[steel|titanium] world:=[world]
```

<br>

>Ejemplo de TIAGo en el entorno 1 de obtención de imágenes para el dataset:

```
$ roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel world:=get_dataset_objects01
```

![alt text](doc/simulation02.jpg "TIAGo en el entorno de dataset 1")


<br>

## Nodos implementados

### TakeImagesNode

>Lanzar el nodo **take_images_node**

```
$ catkin build take_images
$ source devel/setup.bash
$ roslaunch take_images take_images_launch_file.launch
```


<br>

### YoloDetectionNode

>Lanzar el nodo **yolo_detection_node**

```
$ catkin build yolo_detection_obj
$ source devel/setup.bash
$ roslaunch yolo_detection_obj yolo_detection_launch_file.launch
```


<br>

### NavigationNode

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
El servicio guardará el mapa en la ruta *~/.pal/tiago_maps/config*.


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

*Terminal 2*
```
$ rosservice call /move_base/clear_costmaps "{}"
```

Cuando se haya finalizado la *localización* presionamos `q` y ...


<br>

### MoveToPointNode


<br>

### GetGazeboObjNode


<br>

### GripperGraspingNode
