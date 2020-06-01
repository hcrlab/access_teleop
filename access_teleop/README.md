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

#### 2. In browser:  
* If using ngrok, go to: http://accessteleop.ngrok.io.  
* If using localhost, go to: localhost:8082.  


## How to switch between localhost and nrgok  
1. **In `app.js`:**  
Set the `host` field when initializing `MJPEGCANVAS.Viewer`:  
For `localhost`, use: `host : 'localhost'`.  
For `ngrok`, use: `host : 'rosvideo.ngrok.io'`.  
2. **In `mjpegcanvas.js`:**  
For `localhost`, uncomment line 328.  
For `ngrok`, uncomment line 332.  

* **To change the settings for ngrok, see `ngrok.yml`.**


## Source Code
### **Frontend:** in folder `frontend/html`
#### In folder `common_js`:  
* `app.js`:  
Set up the gripper slider, freeze/unfreeze point cloud button, and three camera views.  
* `mjpegcanvas.js`:  
Source code for the `MJPEGCANVAS` used by camera views. This source code is copyed from GitHub because a minor change needs to be made when using ngrok.  

#### Interface-specific code:  
* Multi-View Robot Trajectory GUI: `Multi.html`, `multi.js`.  
* Trajectory Plane Robot GUI: Click and Orient: `ClickAndOrient.html`, `clickAndOrient.js`.  
* Trajectory Plane Robot GUI: One Touch: `OneTouch.html`, `oneTouch.js`.  

### **Backend:** in folder `scripts`
* `access_pointcloud_pub.py`: Point cloud saver and publisher.  
* `camera_info_messages.py`: Properties of camera views.  
* `fetch_access_teleop.py`: Code for the main program: set the initial robot state, move end effector based on frontend events, add/remove testing objects from Gazebo.   
* `shared_teleop_functions_and_vars.py`: Code for utility functions, camera positions, and end effector pose calculations based on camera positions.  
* `switch_object.sh`: Bash script for deleting and adding models in Gazebo.  
  - Comman-line usage:  
    ```
    $(rospack find access_teleop)/scripts/switch_object.sh [MODEL TO DELETE] [MODEL TO ADD]
    ```
  - Set the first parameter to `NONE` when you only want to add object without deleting anything:  
    ```
    $(rospack find access_teleop)/scripts/switch_object.sh NONE [MODEL TO ADD]
    ```
  - Sample python code for invoking this script:
    ```
    os.system("$(rospack find access_teleop)/scripts/switch_object.sh cube_s cube_m")
    ```  
  - The position of the added testing object is fixed (x: 3.8, y: 3, z: 0.83), edit line 18 to modify the position:  
    ```
    rosrun gazebo_ros spawn_model -file $(rospack find access_teleop)/models/$2/model.sdf -sdf -model $2 -x 3.8 -y 3 -z 0.83
    ```   

### **Gazebo models and world**
#### Models: in folder `models`  
* `ball, cube_s/m/l/xl, stone` are the models of the six objects used for grasping acitity specified in ACTION RESEARCH ARM TEST.
* `shelf` is the model of the shelf on the table.
#### World: in folder `worlds`
* `test_zone.sdf`: The world which contains only a table and a shelf on it. Testing object needs to be added using `switch_object.sh`.  
* `test_zone_all_objects.sdf`: The world which contains six tables, with one testing object and one shelf on each table.  

## Ngrok Setup
Settings for ngrok are specified in `.../.ngrok2/ngrok.yml`. You can do `$ locate ngrok.yml` to find the file location.  

The configurations that works on Ubuntu 14.04 is [here](./files/ngrok_setup.txt).  


See [Ngrok documentation](https://ngrok.com/docs) for details. The [configuration section](https://ngrok.com/docs#config) has details on how to write the configuration file.  

These are two other useful links for Apache setup: 
* [Changing apache2 document root in ubuntu 14.x](https://julienrenaux.fr/2015/04/06/changing-apache2-document-root-in-ubuntu-14-x/)  
* [How To Move an Apache Web Root to a New Location on Ubuntu 16.04](https://www.digitalocean.com/community/tutorials/how-to-move-an-apache-web-root-to-a-new-location-on-ubuntu-16-04)