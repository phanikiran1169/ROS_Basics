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

### Compile
```bash
cd ~/catkin_ws/ # go to root directory of the packages
catkin build hello_world # Compile a ROS package
source devel/setup.bash # Source the ROS env variable
```
NOTE: We are using `catkin build` from catkin_tools instead of `catkin_make`. To learn more about the differences, refer this [link](https://catkin-tools.readthedocs.io/en/latest/migration.html).

### RUN

Before we run our first program, we need to run the master node i.e. `roscore`. Open a new terminal and run the following command:
```bash
roscore
```
ROS is a centralized system, a master node (program) is always needed for other nodes and should be executed before other nodes. roscore starts this master node.

In the terminal where the file `devel/setup.bash` was sourced, run the following command (if not you can run the command `source devel/setup.bash` in the terminal)
```bash
rosrun hello_world hello_world.py
```

### Create a launch file

#### Launch file explained

One way to execute a program is to launch one node at a time which is what was done above where `roscore` and `hello_world.py` were run separately. This is fine if we have just two nodes, but what do we do if we have multiple nodes? Launching each node one-by-one can get inefficient really quickly.

Fortunately, ROS has a tool called [roslaunch](http://wiki.ros.org/roslaunch) that enables you to launch multiple nodes all at once.

We now write a launch file to run both the nodes. 

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
File structure explained:

1. The root element: &lt;launch&gt; ... &lt;/launch&gt; <br/>
Like every XML document, launch files have exactly one root element. It is called launch. All of the other elements of each launch file should be enclosed between these tags.

2. The node element: &lt;node&gt; ... &lt;/node&gt;
- pkg (required): The name of the package the node belongs to.
- type (required): The name of the executable file that starts the node.
- name (required): The name of the node. This overrides any name that the node would normally assign to itself in its call to ros::init. It should be a relative name (without mention of any namespaces), not a global name.
- respawn (optional): If set to “true” roslaunch restarts the node when it terminates.
- required (optional): If a node is labeled as required then when it is terminated roslaunch terminates all other nodes.
- launch-prefix (optional): Used to insert the given prefix at the start of the command line that runs the node. The prefix “xterm-e” starts a new terminal window.
- output (optional): If set to “screen” (for only a single node) allows the standard output of the node be the terminal and not a log file.
- ns (optional): To set a namespace.

3. The include element: &lt;include&gt; … &lt;/include&gt;
This element allows you to include the contents of another launch file, including all of its nodes and parameters.

```xml
<include file=”path-to-launch-file” />
```

A find substitution to search for a package is usually used in order not to explicitly specify the path:

```xml
<include file=”$(find package-name)/launch-file-name” />
```
4. The arg element: &lt;arg&gt; … &lt;/arg&gt;

This element is used to help make launch files configurable. It is defined as:
```xml
<arg name=”arg-name” />
```
And used with an arg substitution:
```xml
$(arg arg-name)
```
The value of the arguments:
- Is set at the command line:
```bash
roslaunch package-name launch-file-name arg-name:=arg-value
```
- Is defaulted with a value that can be overriden at the command line:
```xml
<arg name=”arg-name” default=”arg-value”/>
```
- Is defaulted with a value that cannot be overriden at the command line:
```xml
<arg name=”arg-name” value=”arg-value”/>
```




