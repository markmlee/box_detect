# box_detect
simple  detection and pose estimation of 1 cardboard box


### Set up
* install realsense ros package as described in KIrobotics wiki page
* run PODO SW on motionPC


### Running box_detect with hubo 

* git clone this repository into your catkin workspace
* connect hubo through PODO SW on motionPC
```sh
$ roslaunch realsense2_camera rs_rgbd.launch
$ rosrun box_detect box_detect
```

* verify box pose TX to PODOLAN by (seeing client connected / display value in ApproachBox AL)


Maintainers
----
KAIST HUBO LABS - ML634@kaist.ac.kr



