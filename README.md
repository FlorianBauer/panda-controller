# Panda Controller

## Requirements

### Packages and ROS

1. Install the following basic packages.
```
sudo apt install -y build-essential autoconf libtool pkg-config cmake
```

2. Install [ROS](http://wiki.ros.org/) from the [Install Site](http://wiki.ros.org/ROS/Installation).
It is recommended to install `ROS Noetic` and the `ros-<distro>-desktop-full` package.

3. Install the following packages required for SiLA.
```
sudo apt install qtbase5-dev libssl-dev libavahi-client-dev libavahi-common-dev
```


### Build and Install a Linux Real-Time Kernel

TODO: add install instructions

[Ranka Emika Set-Up Guide](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
[RT Kernel Set-Up Guide](https://medium.com/@patdhlk/realtime-linux-e97628b51d5d)

### Build and Install gRPC

1. Set the environment variable.
```bash
export LOCAL_INSTALL_DIR=$HOME/.local
```

2. Create the install directory.
```bash
mkdir -p $LOCAL_INSTALL_DIR
```

3. Clone the gRPC repository. (The `-b` option defines the branch)
```bash
git clone --recurse-submodules -b v1.34.x https://github.com/grpc/grpc
cd grpc
```

4. Build and install gRPC. (The `-j` option defines the number of jobs/cores used)
```bash
mkdir -p cmake/build
pushd cmake/build
cmake -DgRPC_INSTALL=ON \
      -DgRPC_BUILD_TESTS=OFF \
      -DgRPC_SSL_PROVIDER=package \
      -DCMAKE_INSTALL_PREFIX=$LOCAL_INSTALL_DIR \
      -DBUILD_SHARED_LIBS=ON
      ../..
make -j 4
make install
popd
```

_Links for troubleshooting:_
[sila_cpp build instructions](https://gitlab.com/SiLA2/sila_cpp/-/blob/master/BUILDING.md#grpc)
[gRPC Quickstart](https://grpc.io/docs/languages/cpp/quickstart/)


### Build and Install `sila_cpp`

1. Clone `sila_cpp`
```bash
clone --recurse-submodules https://gitlab.com/SiLA2/sila_cpp.git
```

2. Build `sila_cpp`
```bash
cd path/to/sila_cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$LOCAL_INSTALL_DIR
cmake --build .
```

3. Install `sila_cpp`
```
cmake --install .
```


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
