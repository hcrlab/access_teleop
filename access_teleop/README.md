# access_teleop
A novel teleoperation interface to more easily and accessibly perform mobile manipulation operations

## How to launch the AccessTeleop interface
#### 1. In terminal:  
Terminal 1:  
```
$ roslaunch access_teleop fetch_full_launch.launch
```
Terminal 2:  
```
$ cd catkin_ws/src/access_teleop/frontend/html  
$ python -m SimpleHTTPServer 8082  
```
Terminal 3: (only if using ngrok)  
```
$ cd ~/catkin_ws/src/access_teleop/frontend/html  
$ ./ngrok start --all  
```

#### 2. In browser, go to: http://accessteleop.ngrok.io (if using ngrok); go to: localhost:8082 (if using localhost).  


## How to switch between localhost and nrgok  
1. In `app.js`, set the `host` field when initializing `MJPEGCANVAS.Viewer`:  
For `localhost`, use: `host : 'localhost'`.  
For `ngrok`, use: `host : 'rosvideo.ngrok.io'`.  
2. In `mjpegcanvas.js`:  
For `localhost`, uncomment line 328.  
For `ngrok`, uncomment line 332.  

* To change the settings for ngrok, see `ngrok.yml`.


## Source Code
### Frontend：in folder `frontend/html`
#### In folder `common_js`:  
* `app.js`:  
Set up the gripper slider, freeze/unfreeze point cloud button, and three camera views.  
* `mjpegcanvas.js`: Source code for the `MJPEGCANVAS` used by camera views. This source code is copyed from GitHub because a minor change needs to be made when using ngrok.  

#### Interface-specific code:  
* Multi-View Robot Trajectory GUI: `Multi.html`, `multi.js`.  
* Trajectory Plane Robot GUI: Click and Orient: `ClickAndOrient.html`, `clickAndOrient.js`.  
* Trajectory Plane Robot GUI: One Touch: `OneTouch.html`, `oneTouch.js`.  

### Backend: in folder `scripts`
* `access_pointcloud_pub.py`: Point cloud saver and publisher.  
* `camera_info_messages.py`: Properties of camera views.  
* `fetch_access_teleop.py`: Code for the main program. This is the place which sets the initial robot state.    
* `shared_teleop_functions_and_vars.py`: Code for utility functions, camera positions, and end effector pose calculations based on camera positions.  
* `switch_object.sh`: Bash script for deleting and adding models in Gazebo.  

### Gazebo models and world
#### Models: in folder `models`  
* `ball, cube_s/m/l/xl, stone` are the models of the six objects used for grasping acitity specified in ACTION RESEARCH ARM TEST.
* `shelf` is the model of the shelf on the table.
#### World: in folder `worlds`
* `test_zone.sdf`: The world which contains only a table and a shelf on it. Testing object needs to be added using `switch_object.sh`.  
* `test_zone_all_objects.sdf`: The world which contains six tables, with one testing object and one shelf on each table.  

