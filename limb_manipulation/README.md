# limb_manipulation
-Simulation  
$ rosrun applications publish_saved_cloud.py /home/maru/catkin_ws/src/limb_manipulation/bags/ar1.bag  
$ roslaunch limb_manipulation ar.launch cam_image_topic:=/mock_point_cloud  
$ roslaunch limb_manipulation limb_ar_sim.launch  
$ roslaunch limb_manipulation limb_ar_demo.launch  
$ python -m SimpleHTTPServer 8082  

-Real  
$ roslaunch limb_manipulation limb_ar_real.launch  
$ roslaunch limb_manipulation limb_ar_web.launch  
$ python -m SimpleHTTPServer 8082  
(on Fetch)  
$ roslaunch ezgripper_driver sake_ezgripper.launch  