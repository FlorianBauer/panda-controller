# Panda Controller

## Build the project

```
mkdir -p ~/catkin_ws/src/
```
* Copy these project files into the `src`-dir.
* Also add the `panda_moveit_config` from https://github.com/ros-planning/panda_moveit_config
(`git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel`).
* Ensure a `package.xml`-file with all the dependencies exists.

Build with:
```
cd ~/catkin_ws/
source /opt/ros/noetic/setup.bash
catkin build
```

## Run the project in MoveIt

Start a terminal, source the workspace and launch MoveIt:
```
source devel/setup.bash
roslaunch panda_moveit_config demo.launch
# On real hardware:
# roslaunch franka_control franka_control.launch
```

Start another terminal, source the workspace and run the build package:
```
source devel/setup.bash
rosrun panda_controller robot_service
```

To omit sourcing the workspace every time in a new terminal, the setup can be amended into 
`.bashrc` with `echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc`

To check the proper set up of the workspace `echo $ROS_PACKAGE_PATH` should return something like
```
/home/username/path/to/catkin_ws/src:/opt/ros/noetic/share
```
