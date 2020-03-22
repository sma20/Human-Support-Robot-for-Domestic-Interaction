Get 3d position
===============

   ![example](.image/example.png)

This is a node for getting 3D position from the output of object detection (yolo).

Prerequisites
=============

* Build this package with *tmc_darknet_msgs* (https://git.hsr.io/tmc/tmc_darknet_ros/tree/master/tmc_darknet_msgs)

Usage
=====

```bash
    $rosrun get_3d_position get_3d_position.py
```

* If you want to check the 3D position in rviz, add an argument and visualize '/3d_position' topic.

```bash
    $rosrun get_3d_position get_3d_position.py _debug:=true
```
