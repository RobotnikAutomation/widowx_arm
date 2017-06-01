# widowx_arm

This ROS package is intended to work with the [WidowX](https://www.roscomponents.com/en/robotic-arms/11-widowx.html#/assembled-no) arm.

* widowx_arm_controller : Controller based on arbotix_python driver to control de arm
* widowx_arm_description : Description of the arm

## Installation and configuration

### Setting up the Arbotix-M board

In order to work with ROS it is necessary to upload the firmware into the Arbotix-M board.

* Download Arduino ide from https://downloads.arduino.cc/arduino-1.0.6-linux64.tgz
  * wget https://downloads.arduino.cc/arduino-1.0.6-linux64.tgz
  
* Extract it into a folder.
* Download the firmware archives from https://github.com/trossenrobotics/arbotix/archive/master.zip
  * wget https://github.com/trossenrobotics/arbotix/archive/master.zip
* Extract it into a folder like ~/Documents/Arduino
* Run arduino from the folder you extracted it previously
  * cd ~/Downloads/arduino-1.0.6
  * ./arduino
* Once Arduino IDE is running, change the Sketchbook folder location to /Documents/Arduino/arbotixmaster or the one you extracted it previously.
  * File->Preferences->Sketchbook Location
  * Tools->Board->Arbotix
  * Tools->Serial Port->/dev/ttyUSBX
  * File->Sketchbook->Arbotix Sketches ->ros
  * Verify + Upload
* The Arbotix is ready to work with ROS!!

### Downloading the package

clone the repo into your workspace and compile it.
```
git clone https://github.com/RobotnikAutomation/widowx_arm.git
```
### Creating the udev rule for the device

In the widowx_arm_controller/config folder there's the file 58-widowx.rules. You have to copy it into the /etc/udev/rules.d folder.

```
sudo cp 58-widowx.rules /etc/udev/rules.d
```

You have to set the attribute ATTRS{serial} with the current serial number of the ftdi device

```
udevadm info -a -n /dev/ttyUSB0 | grep serial 
```
Once modified you have to reload and restart the udev daemon

```
sudo service udev reload
sudo service udev restart
sudo udevadm trigger
```

### Running the controller

```
roslaunch widowx_arm_controller widowx_arm_controller.launch 
```

### Commanding the controller 

```
rostopic pub /joint_1/command std_msgs/Float64 "data: 0.0" 
rostopic pub /joint_2/command std_msgs/Float64 "data: 0.0" 
rostopic pub /joint_3/command std_msgs/Float64 "data: 0.0" 
rostopic pub /joint_4/command std_msgs/Float64 "data: 0.0" 
rostopic pub /joint_5/command std_msgs/Float64 "data: 0.0" 
rostopic pub /gripper_revolute_joint/command std_msgs/Float64 "data: 0.0" 
rostopic pub /gripper_prismatic_joint/command std_msgs/Float64 "data: 0.0"
```

### Visualizing the state

Load the description and run the state publisher

```
roslaunch widowx_arm_description load_description.launch
```

Open the RVIZ tool and add the plugins you need to visualize the arm

```
rosrun rviz rviz
```
