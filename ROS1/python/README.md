# ROS 1 Basics
## Creating our first package
1. Create a package
2. Create a node [Python]
3. Compile
4. Run 

### Create a Package 
First we create our catkin workspace and source directory
```bash
mkdir -p ~/catkin_ws/src/  # Create a directory
cd ~/catkin_ws/src/     # Move to src directory
```

We use the catkin_create_pkg command to create a new package called "hello_world". This package depends on rospy.
```bash
catkin_create_pkg hello_world rospy # Create a package
# rospy is python client library for ROS
```

### Create a node
```bash
cd hello_world/ && touch src/hello_world.py # Create a empty file
chmod a+x src/hello_world.py # Execution permission
```

Open `hello_world.py` in your favourite text editor and copy paste the following contents:

```python
#! /usr/bin/env python3 
# Indicate that python3 interpretor is used

import rospy # Python client lib for ROS

rospy.init_node("hello")           # Initiate a node called hello
rate = rospy.Rate(1)               # We create a Rate object to control the execution speed of while loop to 1Hz
while not rospy.is_shutdown():     # Continous loop
   print("My first ROS package Hello world ")
   rate.sleep()                    # We sleep the needed time to maintain the above Rate
```
### Create a launch file

```bash
mkdir launch && touch launch/hello_world.launch # Create a empty lauch file
```
Again open `hello_world.launch` in your favourite text editor and copy paste the following contents:

```xml
<launch>
    <!-- My Package launch file -->
    <node pkg="hello_world" type="hello_world.py" name="hello_world_node"  output="screen">
    </node>
</launch>
```
#### Launch file explained

`
<node pkg="hello_world" type="hello_world.py" name="hello_world_node"  output="screen">
`

### Compile
```bash
cd ~/catkin_ws/ # go to root directory of the packages
catkin build hello_world # Compile a ROS package
source devel/setup.bash # Source the ROS env variable
```
NOTE: We are using `catkin build` from catkin_tools instead of `catkin_make`. To learn more about the differences, refer this [link](https://catkin-tools.readthedocs.io/en/latest/migration.html).

### RUN
```bash
roslaunch hello_world hello_world.launch
```

