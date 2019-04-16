# RBE3002-RandomRobot
By [Alex Tacescu](https://www.alextac.com)

## Setup
1. Ensure ROS Kinetic is set up on your machine

2. Clone the Github Repository

3. Make sure you update all submodules
```txt
git submodule update --init --recursive
```
4. Build your workspace
```txt
catkin_make
```
## Running the demo
1. If you haven't already, source the ROS distribution
```txt
source /opt/ros/kinetic/setup.bash
```

2. Change to the directory containing the files for the demo

3. Build your workspace if you haven't already
```txt
source devel/setup.bash
```

4. Ensure that the correct turtlebot has been selected
```txt
export TURTLEBOT3_MODEL=burger
```
or
```txt
export TURTLEBOT3_MODEL=waffle
```
or
```txt
export TURTLEBOT3_MODEL=waffle_pi
```

### Simulation
Run the following command:
```txt
roslaunch rbe3002_demo demo_sim.launch
```
