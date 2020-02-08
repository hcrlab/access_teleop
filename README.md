# access_teleop
A noval teleoperation interface to more easily and accessibly perform mobile manipulation operations

## Run the Limb Manipulation Program in Simulation  
####  1. Launch the simulation:  
`$ roslaunch limb_manipulation limb_sim.launch`
#### 2. Launch the AR marker detector:  
`$ roslaunch limb_manipulation ar.launch cam_image_topic:=/mock_point_cloud`
#### 3. Publish a fake point cloud: 
Bag files of fake point clouds are in `access_teleop/limb_manipulation/bags/` **with prefix `ar_`**.  
To publish a point cloud, do:  
`$ rosrun limb_manipulation publish_saved_cloud.py <PATH_TO_A_POINT_CLOUD_BAG_FILE>`  
Below is one example:  
`$ rosrun limb_manipulation publish_saved_cloud.py ~/catkin_ws/src/access_teleop/limb_manipulation/bags/ar_right_wrist.bag`
#### 4. Launch the interface:  
There are two types of interface: command-line interface and web interface.  
- To launch the command-line interface, do:  
`$ roslaunch limb_manipulation limb_demo.launch`
- To launch the web interface first do:  
`$ roslaunch limb_manipulation limb_web.launch`  
Then, open another terminal and:  
`$ cd ~/catkin_ws/src/access_teleop/limb_manipulation/frontend`  
`$ python -m SimpleHTTPServer 8082`  
Now you can visit the site at `localhost:8082`

### Example commands for performing a limb manipulation action
1. `attach`: close Fetch's gripper for it to grasp SAKE gripper
2. `record`: record the current AR markers the robot detects
3. `parts`: list the current body parts the robot detects
4. `go 0`: go to body part with ID number 0
5. `grasp`: grasp the body part with ID number 0
6. `actions`: list of available actions
7. `do RAEF`: perform the action named: Right Arm Elbow Flexion

## Run the Limb Manipulation Program on Fetch  
#### 1. On the lab machine
- Launch Rviz and all other nodes:  
`$ roslaunch limb_manipulation limb_real.launch`  
- Launch the main program:  
`$ roslaunch limb_manipulation limb_web.launch`  
- Launch the web interface backend:  
`$ cd ~/catkin_ws/src/access_teleop/limb_manipulation/frontend`  
`$ python -m SimpleHTTPServer 8082`  
Now you can visit the site at `localhost:8082`
#### 2. Set up SAKE gripper driver
- Plug the USB cable of SAKE gripper into the USB port on Fetch.  
- Launch the SAKE gripper driver on Fetch (this node must be launched on Fetch)  
`$ roslaunch ezgripper_driver sake_ezgripper.launch`

## Control SAKE Gripper Using Code
- Outside of the limb manipulation program:
```
import rospy
from limb_manipulation_msgs.msg import EzgripperAccess

pub = rospy.Publisher('/ezgripper_access', EzgripperAccess, queue_size=1)
rospy.sleep(0.1)

# Option 1: Use default actions: calibrate, h_close, s_close, open
pub.publish(EzgripperAccess(type='open'))

# Option 2: Specify the percentage to open, and the force of the gripper on your own:  
pub.publish(EzgripperAccess(type='20 100'))
```
- Inside of the limb manipulation program
```
from limb_pbd_server import PbdServer

server = PbdServer()
server.do_sake_gripper_action("h_close")
```

## Control SAKE Gripper Through Command-line
#### 1. Plug the USB cable of SAKE gripper into the USB port of your computer.  
#### 2. Launch the SAKE gripper driver:   
`$ roslaunch limb_manipulation sake_ezgripper.launch`
#### 3. Now you can control the SAKE gripper using:  
- *Option 1*: Use default actions:  
`calibrate, h_close, s_close, open`  
Example:  
`$ rosrun limb_manipulation set_sake_gripper.py open`
- *Option 2*: Specify the percentage to open, and the force of the gripper on your own:  
`$ rosrun limb_manipulation set_sake_gripper.py <percentage open> <effort>`  
*Percentage open*: 0% (close) ---> 100% (open)  
*Effort*: 0 (min effort) ---> 100 (max effort)  
Example:  
`$ rosrun limb_manipulation set_sake_gripper.py 20 100`

