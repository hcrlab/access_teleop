# access_teleop
A novel teleoperation interface to more easily and accessibly perform mobile manipulation operations

### **How to launch the AccessTeleop interface**
#### 1. In terminal:  
Terminal 1  
```
$ roslaunch access_teleop fetch_full_launch.launch
```
Terminal 2  
```
$ cd catkin_ws/src/access_teleop/frontend/html  
$ python -m SimpleHTTPServer 8082  
```
Terminal 3  
```
$ cd ~/catkin_ws/src/access_teleop/frontend/html  
$ ./ngrok start --all  
```

#### 2. In browser, go to: http://accessteleop.ngrok.io