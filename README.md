![GitHub repo size](https://img.shields.io/github/repo-size/ab31mohit/autosky_pkg)

# Autosky Aerospace Internship 2024

## Pre installation Requirements 
1. Ubuntu 20.04 LTS Desktop
2. ROS1 Noetic desktop full
3. Git

## Installation steps
### 1. Install Ardupilot and MAVProxy for Ubuntu 20.04 :

- Clone ArduPilot in home directory
```
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

- Install dependencies
```
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

- Reload profile
```
. ~/.profile
```

- Checkout Latest Copter Build
```
git checkout Copter-4.2
git submodule update --init --recursive
```

- Run SITL (Software In The Loop) once to set params
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

### 2. Install Gazebo plugin for APM (ArduPilot Master) :

- Clone this repo in home directory
```
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
```

- build and install plugin
```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

- set model paths
```
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

### 3. Setting up the workspace :


- We'll use `catkin build` instead of `catkin_make`. Please install the following:
```
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
pip3 install osrf-pycommon
```

- Then create & initialize the catkin workspace in your home directory:
```
mkdir -p ~/autosky_ws/src
cd ~/autosky_ws
catkin init
```

## 3. Dependencies installation

- Install `mavros` and `mavlink` from source
```
cd ~/autosky_ws
wstool init src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```

- Add your workspace sourcing command at the end of your `.bashrc` file
```
echo "source ~/autosky_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

- Install geographiclib dependancy (in case they're not installed) 
```
sudo ~/autosky_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```


### 4. Clone the following repos in your workspace : 

- Clone autosky package
```
cd ~/autosky_ws/src
git clone https://github.com/ab31mohit/autosky_pkg.git
```

- set the models present in `autosky_pkg/models` to gazebo model
```
echo "GAZEBO_MODEL_PATH=~/autosky_ws/src/autosky_pkg/models" >> ~/.bashrc
```


- Copy & paste the contents of `default_params` folder to `home/username/ardupilot/Tools/autotest/default_params` directory.   

- Replace `/home/username/ardupilot/Tools/autotest/pysim/vehicleinfo.py` file with *`vehicleinfo.py`* (present in this repository) file.

- Clone turtlebot3 packages

```
cd ~/autosky_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd ~/autosky_ws
catkin build
```

### 4. Testing the installation setups : 

- Launch iris drone and turltlebot3 on runway
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch autosky_pkg runway_drone-ugv.launch
```

- Launch SITL (autopilot) for your drone
```
roscd autosky_pkg/sitl_config
bash ./SITL.sh
```
---------------------------------------------------
- *Note:*   
In case you encounter some error like `bash: sim_vehicle.py command not found` then add the following line in your
`.bashrc` just above your ros workspace sourcing command 
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
```
---------------------------------------------------

- Launch mavros node to communicate between ros messages and sitl's mavlink messages
```
roslaunch autosky_pkg apm.lauch
```

- Use the following MAVProxy commands on sitl terminal to control the drone

```
mode guided
arm throttle
takeoff 5
land
```

- Launch the teleop node to control turtlebot3
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### 5. Setting the YOLO/Darknet ROS image recognition :

- #### Install CUDA ####    
Cuda is a library that allows programs to take advantage of your GPU as a computing resource. YOLO will run without Cuda, but the algorithm is up to 500 x more quick with Cuda.
So if your system has a dedicated GPU (then only install cuda, otherwise don't). To install Cuda, run
```
sudo apt install nvidia-cuda-toolkit
```

- #### Clone YOLO/Darknet repo in your workspace ####
```
cd ~/autosky_ws/src
git clone https://github.com/leggedrobotics/darknet_ros.git
cd darknet_ros/
git submodule update --init --recursive
```

- #### Build Darknet ROS Package####
```
cd ~/autosky_ws
catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release
```

If you run into erros, try running the following command :
```
catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-8
```

- #### Configure YOLO/Darknet

The `ros.yaml` specifies ros parameters. You can find this file under `darknet_ros/darknet_ros/config`. You will need to change the image topic from `/camera/rgb/image_raw` to `/webcam1/image_raw`.    

The file `darknet_ros.launch` will launch the darknet/yolo ros node. You can find this file under `darknet_ros/darknet_ros/launch`. In this file you can choose which version of yolo you would like to run by changing:    
```
<arg name="network_param_file"         default="$(find darknet_ros)/config/yolov2-tiny.yaml"/>    
```
the options are as follows :

- yolov1: Not recommended. this model is old 
- yolov2: more accurate, and faster. 
- yolov3: about as fast as v2, but more accurate. Yolo v3 has a high GPU ram requirement to train and run. If your graphics card does not have enough ram, use yolo v2 
- tiny-yolo: Very fast yolo model. Would recommend for application where speed is most important. Works very well on Nvidia Jetson


### `YOLO/Darknet` References :   
- *https://pjreddie.com/darknet/yolo/*    
- *https://github.com/leggedrobotics/darknet_ros*
