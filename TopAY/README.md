# TopAY
_Trajectory Planning for Differential Drive Mobile Manipulators via **Top**ological Paths Search and **A**rc Length-**Y**aw Parameterization_


https://github.com/user-attachments/assets/49d5e778-fa88-4079-9445-b330ba4fae1a

## Setup Guide

We strongly recommend to use Docker to minimize the risk of compatibility issues.
To do so, follow the [Quick Setup](#quick-setup) guide. \
Alternatively, you can setup the environment manually following [Manual Setup](#manual-setup) guide.

### Quick Setup

1. Pull this code repository to your desired path:
```
git clone https://github.com/TopAY-Planner/TopAY.git
cd TopAY
```

2. Build the image with the following command: 
```
docker build -t topay .
xhost +Local:*
```

3. then, create and start a container as follows:
```
docker create -it \
        --env="DISPLAY=$DISPLAY" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --runtime=nvidia \
        --gpus all \
        --name topay \
        topay

docker start topay
```

4. enter the container's bash shell with the following command:
```
docker exec -it topay bash
# exit container with ctrl+d
```

5. Once the container is properly setup, you should be able to compile the source code without extra steps:
```
catkin_make # inside the container
```

### Manual setup

#### Step 1.1: Install Dependencies


```
apt-get update && \
apt-get install -y \
    git \
    vim \
    wget \
    ros-noetic-rosfmt \
    ros-noetic-ackermann-msgs \
    ros-noetic-pybind11-catkin \
    ros-noetic-serial \
    ros-noetic-rqt-multiplot \
    ros-noetic-vrpn-client-ros \
    ros-noetic-ompl \
    ros-noetic-plotjuggler-ros \
    libasio-dev \
    libompl-dev \
    libeigen3-dev \
    libglfw3-dev \
    libglew-dev \
    libdw-dev \
    python3-pip \
    python3-tk \
    xarclock mesa-utils \
    libx11-dev libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev \
    vulkan-utils mesa-vulkan-drivers pigz libegl1
```

#### Step 1.2: Install osqp & osqp-eigen

**Download and install osqp-0.6.2 and osqp-eigen v0.8.0 for mpc controller:**

```
# replace <PATH> with your desired path
cd <PATH>
wget https://github.com/osqp/osqp/releases/download/v0.6.2/complete_sources.tar.gz
tar -xzf complete_sources.tar.gz
rm complete_sources.tar.gz
mkdir <PATH>/osqp/build && cd <PATH>/osqp/build
cmake .. && make -j && make install
```

```
cd <PATH>
wget https://github.com/robotology/osqp-eigen/archive/refs/tags/v0.8.0.tar.gz
tar -xzf v0.8.0.tar.gz
rm v0.8.0.tar.gz
mkdir <PATH>>/osqp-eigen-0.8.0/build && cd <PATH>>/osqp-eigen-0.8.0/build
cmake .. && make -j && make install
```

####  1.3: Install casadi

```
cd <PATH>
wget https://github.com/casadi/casadi/archive/main.tar.gz
tar -xzf main.tar.gz
rm main.tar.gz
sed -i -e 's/WITH_LAPACK_DEF\sOFF/WITH_LAPACK_DEF ON/g' /home/casadi-main/CMakeLists.txt
sed -i -e 's/WITH_QPOASES_DEF\sOFF/WITH_QPOASES_DEF ON/g' /home/casadi-main/CMakeLists.txt
mkdir <PATH>/casadi-main/build && cd <PATH>/casadi-main/build
cmake .. && make -j && make install
```

####  1.4: Install livox

```
cd <PATH>
wget https://github.com/Livox-SDK/Livox-SDK2/archive/master.tar.gz
tar -xzf master.tar.gz
rm master.tar.gz
mkdir <PATH>/Livox-SDK2-master/build && cd <PATH>/Livox-SDK2-master/build
cmake .. && make -j && make install
```

#### Step 2: Build the project

change directory into the TopAY directory and run the following command:

```bash
catkin_make
```

## Run Planner

https://github.com/user-attachments/assets/452e73e3-6ac2-40d6-bec0-3d8007694c1e

```
source devel/setup.zsh

# for planning with arbitrary target; use the '2d Nav Goal' to set a target for the planner
roslaunch planner run_all.launch rviz:=true

# for benchmarking in tables
roslaunch planner benchmark_tables.launch rviz:=false

# for benchmarking in cuboids
roslaunch planner benchmark_cuboids.launch rviz:=false

# for ablation test in tables
roslaunch planner ablation_tables.launch rviz:=false

# for ablation test in cuboids
roslaunch planner ablation_cuboids.launch rviz:=false


# Use the following command to start the process
./start.sh
```

<!-- ################################ -->
<!-- Markdown: Phase A Ranger+CR10 local-MPC smoke acceptance -->
<!-- ################################ -->
### Ranger + CR10 smoke (Phase A)

```
roslaunch planner run_ranger_cr10_smoke.launch rviz:=true

# Use `world` (any non-`target` frame) for a base SE(2) goal + random feasible
# terminal arm sample. This is NOT an EE 6D goal.
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Accept only when **all** of the following hold (plan success alone is insufficient):

1. Planner prints successful optimization / accepts a trajectory.
2. MPC receives that trajectory (`local_mode` path; confirm via MPC/planner logs that tracking starts).
3. `/moma_cmd` publishes continuously during tracking.
4. `fake_moma` state on `/moma_odom` evolves under those commands.
5. Final convergence vs the planned terminal whole-body state:
   - base position error &lt; 0.10 m
   - base yaw error &lt; 5 deg
   - arm joint max |error| &lt; 0.05 rad **vs planned terminal arm q**

<!-- ################################ -->
<!-- Markdown: Phase A regression status (Task 8) -->
<!-- ################################ -->
### Phase A regression status

- Default stack: `roslaunch planner run_all.launch` → `robot:=ranger_cr10`.
- Tracer regression path: `roslaunch planner run_all.launch robot:=tracer7`.
- Standalone CR10 FK / EE-grad / collision-grad / home–folded / mid-link / wrist gates (`test_topay_cr10_fk`) PASS on host without catkin.
- Full `roslaunch` / `catkin_make` / richer Ranger corridor scenes are blocked on hosts missing `ackermann_msgs` (and related ROS package deps). Chassis collision remains a conservative disk — narrow corridors may false-positive; treat richer-scene failures as non-blocking for Phase A.
