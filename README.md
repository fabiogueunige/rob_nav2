# Rob_nav2
Inside rob_nav2 there is all the code developed for the assigment of Experimantal Robotics course of Robotic Engeneering master degree at the University of Genoa by the students: Andrea Chiappe, Alberto Di Donna and Fabio Guelfi.

# Download 

This code is written for the ros2 humble version, so running it in foxy(previous version of ros) could give backs some errors\\
For download the package, inside the `src` foder of you ros workspace, clone the repositoy, all the code will be inide the master branch of the repository.

 ```bash
 git clone https://github.com/fabiogueunige/rob_nav2.git
  ```

after cloned the repository, come back to workspace foolder and build it with the command

```bash
cd .. 
colcon build --packages-select rob_nav2
```

after repository was build is posible to run launch the code.\\
Inide the package there are three launch file needed to launch:\\
before Launch check the prerequisites part of the README, indie are linked all the needed dependences
- firts launch the robot, world, and the aruco node
  
- the second launch the slam_toolbox packages and the naviguation2 package, resposnable for the map and the navigation on the world
- the thirn launch the plansys2 package responsable for the planning parto of the assignemnt
  
IMPORTANT: First launch the robot and world, after the navigation and as last the planner 

```bash 
ros2 launch rob_nav2 robot_launch.py
``` 
```bash 
ros2 launch rob_nav2 nav_launch.py
``` 
```bash 
ros2 launch rob_nav2 planner_launch.py
``` 

# Pre-requisites

``` bash
sudo apt-get update
sudo apt-get upgrade
```

If return some errors run this command, else skip and run the next

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys E88979FB9B30ACF2
sudo apt-get update
```

```bash
sudo apt-get install ros-humble-control*
sudo apt-get install ros-humble-ros-control*
sudo apt-get install ros-humble-gazebo*
```

``` bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-joint-state-publisher ros-humle-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher
```

Is also important to have this version of OpenCV library, so in case of different version use this code for unistall and install the one with correct version

``` bash
pip uninstall opencv-contrib-python opencv-python
pip install opencv-contrib-python==4.5.4.60 opencv-python==4.5.4.60
```
## Robot and World
Prerequisites for launch the part related to robot and world

### Aruco Markers
In this project we will use the Aruco library so is need to have this package inside the `scr` foder of your workspace 

``` bash
https://github.com/CarmineD8/ros2_aruco.git
``` 

Also to put the models folder inide the .gazebo folder.
So download this git repository in any position you wants
``` bash
 git clone https://github.com/CarmineD8/aruco_ros.git
```
then find inside the folder model and put it inside the `.gazebo` folder on your pc. This folder is usually inisde the `home` folder of your pc.


## Planner and Navigation

### Slam Toolbox

for info about the slam_toolbox go to this github page:

https://github.com/SteveMacenski/slam_toolbox

if you want to clone this repository:
```bash
git clone https://github.com/SteveMacenski/slam_toolbox
```
and got to correct branch 
```bash
cd slam_toolbox
git checkout humble
```

for see the code you can clone the reposiitory inside you `src` folder

For install the slam_toolbox run one no this two commands:
- Run this if you have cloned the repository, this command will find the missing dependences
```bash
rosdep install -q -y -r --from-paths src --ignore-src
```
- or this command if you don't have the package and want to isntall it from apt
```bash
apt install ros-humble-slam-toolbox
```

### Navigation Toolbox

for info about the navigation go to this github page:

https://github.com/ros-planning/navigation2
clone the repository with the command
```bash
git clone https://github.com/ros-planning/navigation2
```
and got to correct branch 
```bash
cd navigation2
git checkout humble
```
for clone the repository call this command:

``` bash
git clone https://github.com/ros-planning/navigation2
``` 

and for the installation part of the navigation system run this command (humble version)
``` bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3*
```

## Planner dependencs

### Plansys2 

Follow this passages:
cone this repository inside your  `scr` folder in ros2 workspace

 ```bash
 git clone https://github.com/PlanSys2/ros2_planning_system.git
 ```
 
 move to the ros2 humble branch 
 
 ```bash
 cd ros2_planning_system/ 
 git checkout humble-devel 
 ```
 
 now inside the workspace run this command for install the missing dependences
 ```bash
 rosdep install -y -r -q --from-path src --ignore-src --rosdistro humble
 ```
 
 if there are errors with rosdep, run this command inside the workspace folder:
 ```bash
 sudo rosdep init
 rosdep update	
 ```
 and now build the workspace with this colcon command
 ```bash
 colcon build --symlink-install
 ```








