# limb_manipulation
## In Simulation  
```
$ rosrun limb_manipulation publish_saved_cloud.py ~/catkin_ws/access_teleop/limb_manipulation/bags/ar5.bag  
$ roslaunch limb_manipulation ar.launch cam_image_topic:=/mock_point_cloud  
$ roslaunch limb_manipulation limb_ar_sim.launch  
$ roslaunch limb_manipulation limb_ar_demo.launch  
$ python -m SimpleHTTPServer 8082 
```

## On Fetch  
(on the lab machine)
```
$ roslaunch limb_manipulation limb_ar_real.launch  
$ roslaunch limb_manipulation limb_ar_web.launch  
$ python -m SimpleHTTPServer 8082  
```
(on Fetch)  
```
$ roslaunch ezgripper_driver sake_ezgripper.launch  
```