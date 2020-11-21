# ROS Final Project

## Introduction
All the files here are the ros packages. You should put it in the workspace and `catkin_make` before running the files. Some of the files not listed here are already exist in TX2.

## Apriltags dectection
Map Info includes the distance between the origin to other tags. To launch the map files, run:
```
roslaunch apriltags_ros localization.launch
roslaunch tutorial hcc_final_map.launch
```

## Image detection
We used [darknet_ros](https://github.com/leggedrobotics/darknet_ros) to run the object detection. Try this to get the weights file:
```
wget --no--check--certificate https://drive.google.com/file/d/1ehcopJCX6J1gD1WtakG76uK7RilAAx0t/view?usp=sharing
```
Here is the [link](https://drive.google.com/file/d/1ehcopJCX6J1gD1WtakG76uK7RilAAx0t/view?usp=sharing) for the weights file if `wget` fail. 
The corresponding files should be put in following places:
```
yolov3_custom.cfg -> .darknet_ros/yolo_network_config/cfg
yolov3-custom.backup -> ./darknet_ros/yolo_network_config/weights
yolov3_custon.yaml -> ./darknet_ros/config
yolov3_custom.launch -> ./darknet_ros/launch
```
To launch the object detection file:
```
roslaunch darknet_ros yolov3_custom.launch
```
    
## Distance info
Before starting this section, you should ensure that the first two steps have already activated. To get the distances from origin to objects and write the info to the json file and also get the images, exectue:
```
rosrun hcc_lab9 get_format.py
```
After pressing **"B"** on joystick, the distance and the images will be saved. All the image files will be in hcc_lab9/images and json file will be right in hcc_lab9. Json file won't be overwritten when it exits. Instead, new information will been appended to it.
