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

The Real-Time kernel is necessary for dealing with actual hardware. If only a simulation via MoveIt 
and RViz is sufficient, this step can be omitted.

TODO: add install instructions

_Links for troubleshooting:_  
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
      -DBUILD_SHARED_LIBS=ON \
      ../..
make -j 4
make install
popd
```

_Links for troubleshooting:_  
[sila_cpp Build Instructions](https://gitlab.com/SiLA2/sila_cpp/-/blob/master/BUILDING.md#grpc)  
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
```bash
cmake --install .
```

_Links for troubleshooting:_  
[sila_cpp Build Instructions](https://gitlab.com/SiLA2/sila_cpp/-/blob/master/BUILDING.md)


## Build the Project

1. Create a Catkin workspace.
```bash
mkdir -p ~/catkin_ws/src/
```

2. If the recommended simulation mode should be available as well, the MoveIt-config shall also be built.  
    2.1 Change into the workspace `src`-dir: `cd ~/catkin_ws/src/`  
    2.2 Add the `panda_moveit_config` from https://github.com/ros-planning/panda_moveit_config  
        `git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel`

3. Build the project.
```bash
cd ~/catkin_ws/
source /opt/ros/noetic/setup.bash
catkin_make
```

4. Copy the Feature Definition Language files into the directory of the generated binary.
```bash
mkdir -p ./build/panda-controller/bin/meta
cp ./src/panda-controller/meta/SiteManager.sila.xml ./build/panda-controller/bin/meta/SiteManager.sila.xml
cp ./src/panda-controller/meta/RobotController.sila.xml ./build/panda-controller/bin/meta/RobotController.sila.xml
```

## Run the Project in MoveIt

1. Start a terminal, source the workspace and launch MoveIt.
```bash
source devel/setup.bash
roslaunch panda_moveit_config demo.launch
```

2. After RViz has launched, start the actual SiLA Server.
```bash
source devel/setup.bash
./build/panda-controller/bin/PandaControlServer
```

_Troubleshooting:_
* If a `lib*.so: cannot open shared object file: No such file or directory` error is shown on 
start-up of the SiLA server, check the environment was sourced correctly (e.g. with `env`) and is 
not executed as root-user.
* If a `WARNING: Could not open FDL file` appears, check if the corresponding Feature Definition 
Language files are actually available in the `./build/panda-controller/bin/meta/`-directory.
* On an `[registerPublisher] Failed to contact master at [localhost:11311].  Retrying...` error, 
ensure the corresponding roslaunch script is running (`panda_moveit_config` in simulation or the 
launch file for the actual Franka Control Interface (FCI)).

_Various Notes:_
To omit sourcing the workspace every time in a new terminal, the setup can be amended into 
`.bashrc` with `echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc`

To check the proper set up of the workspace `echo $ROS_PACKAGE_PATH` should return something like
```
/home/username/path/to/catkin_ws/src:/opt/ros/noetic/share
```
