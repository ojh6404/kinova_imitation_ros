# kinova_imitation_ros

## Setup

```
mkdir -p ~/ros/kinova_ws/src
cd ~/ros/kinova_ws/src
wstool init
wstool merge https://raw.githubusercontent.com/ojh6404/kinova_imitation_ros/master/rosinstall
wstool update
cd ~/ros/kinova_ws/
source /opt/ros/noetic/setup.bash
rosdep install -y -r --from-paths src --ignore-src
catkin build jsk_kinova_startup kinovaeus kinova_imitation
source ~/ros/kinova_ws/devel/setup.bash
```
