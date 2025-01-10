# Planner and Navigation

## Plansys2 

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

## Slam Toolbox

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

## Navigation Toolbox

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








