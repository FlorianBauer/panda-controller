# Panda Controller

## Requirements

The following packages are required and can be installed with `sudo apt install __<package>__`
`qtbase5-dev`
`libssl-dev`
`libavahi-client-dev`
`libavahi-common-dev`


## Build the Project

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
catkin_make
```

## Run the Project in MoveIt

Start a terminal, source the workspace and launch MoveIt:
```
source devel/setup.bash
roslaunch panda_moveit_config demo.launch
# On real hardware [!sic]:
# roslaunch franka_control franka_control.launch
```

Start another terminal, source the workspace and run the built ROS service:
```
source devel/setup.bash
rosrun panda_controller robot_service
```

To start the ROS client in yet another terminal:
```
source devel/setup.bash
rosrun panda_controller robot_client
```

The executables could also be used without `rosrun`. Therefore, simply execute the files
in a sourced environment with `./devel/lib/panda_controller/robot_service` and 
`./devel/lib/panda_controller/robot_client`.

To omit sourcing the workspace every time in a new terminal, the setup can be amended into 
`.bashrc` with `echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc`

To check the proper set up of the workspace `echo $ROS_PACKAGE_PATH` should return something like
```
/home/username/path/to/catkin_ws/src:/opt/ros/noetic/share
```
