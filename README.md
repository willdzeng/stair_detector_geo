# Stair Detector Geo
This is a ROS package that can detect stairs by subscribing a Depth images.
It will publish a topic that contains the bouding box of the stairs on the image.
## Install
```
cd 'your_working_space'/src
git clone https://github.com/willdzeng/stair_detector_geo.git
cd ..
catkin_make
```
## Launch
Change the coresponding topic name to your published depth images in the launch file.
```
<remap from="depth/image" to="/camera/depth/image_rect_color"/>
```
And then do:
```
roslaunch stair_detector_geo stiar_detector.launch
```
## Dynamic Reconfigure
This package is dynamic reconfiguable.
Just run:
```
rosrun rqt_reconfigure rqt_reconfigure
```
## Demo Video
https://youtu.be/QT97yNQZ-Wo

## Reference
[Delmerico, Jeffrey A., et al. "Ascending stairway modeling from dense depth imagery for traversability analysis." Robotics and Automation (ICRA), 2013 IEEE International Conference on. IEEE, 2013.](
http://web.eecs.umich.edu/~jjcorso/pubs/delmerico_ICRA2013_stairs.pdf)
