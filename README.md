# Panda Controller

## Build the project

```
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/
```
Copy these project files into the `src`-dir.
Ensure a `package.xml`-file with all the dependencies exists.
```
source /opt/ros/melodic/setup.bash
catkin_make
```

## Run the project in MoveIt

Start a terminal, source the workspace and launch MoveIt:
```
source devel/setup.bash
roslaunch panda_moveit_config demo.launch
```

Start another terminal, source the workspace and run the build package:
```
source devel/setup.bash
rosrun panda-controller pick_place
```

To omit sourcing the workspace every time in a new terminal, the setup can be amended into 
`.bashrc` with `echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc`

To check the proper set up of the workspace `echo $ROS_PACKAGE_PATH` should return something like
```
/home/username/path/to/catkin_ws/src:/opt/ros/melodic/share
```
