# MOVE_EXCAVACTOR PACKAGE

## Installing the package

Create a catkin workspace. For instructions on how to create the workspace go here. Download and compile the package:

```
cd ~/srcp2-competitors/ros_workspace/
mkdir src
```

Download the move_package and move to this new src/ folder. Then, build the catkin workspace.

```
cd ..
catkin_make
```

## Running and testing the nodes

1. To run and test the node you must have ran the simulator:

```
~/srcp2-competitors/docker/scripts/launch/roslaunch_docker --run-round 2
```

2. Launch the move_excavator.launch file.

```
cd ~/srcp2-competitors/ros_workspace/
source devel/setup.bash
roslaunch move_excavator move_excavator.launch
```

2. To control the excavator arm. Open a new terminal and:

```
cd ~/srcp2-competitors/ros_workspace/
source devel/setup.bash
```

Then choose one of the following:
- Keyboard teleop (https://github.com/wvu-navLab/SRC2-driving/tree/master/teleop_modes).

- Publishing the joint angles.

```
rostopic pub -1 /joints_goal move_excavator/JointGroup "q1: 0.0 
q2: 0.0
q3: 0.0
q4: 0.0"
``` 

- Publishing a (x,y,z,0,0,phi) message (position relative to mobile base and end-effector angle) (NOT STABLE YET).

```
rostopic pub -1 /volatile_pos geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
  ```

4. To check if there's volatile in the bucket. Open a new terminal and:

```
cd ~/srcp2-competitors/ros_workspace/
source ~/srcp2-competitors/ros_workspace/install/setup.bash
rostopic echo /excavator_1/bucket_info
```

5. To check if there's volatile in the hauler. Open a new terminal and:

```
cd ~/srcp2-competitors/ros_workspace/
source ~/srcp2-competitors/ros_workspace/install/setup.bash
rostopic echo /hauler_1/bin_info
```

Outro:
If you haven't installed a teleop node for the rover, do the following:

```
sudo apt-get install ros-melodic-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=excavator_1/skid_cmd_vel
```



