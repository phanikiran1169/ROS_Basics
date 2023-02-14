# ROS 1 Basics

## Creating our first package

#### What is a package ?
Before we create our own package, lets try to understand what a package means. Packages are the main unit for organizing software in ROS. A package may contain ROS runtime processes (nodes), a ROS-dependent library, datasets, configuration files, or anything else that is usefully organized together. Packages are the most atomic build item and release item in ROS. Meaning that the most granular thing you can build and release is a package.

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
![](https://user-images.githubusercontent.com/17789814/218476112-c50a2c72-4cae-4d7f-97cb-32b3d852c0cd.png)

### ROS Computation graph

The Computation Graph is the peer-to-peer network of ROS processes that are processing data together. The basic Computation Graph concepts of ROS are nodes, Master, Parameter Server, messages, services, topics, and bags, all of which provide data to the Graph in different ways.

These concepts are implemented in the ros_comm repository. We explore these concepts as we progress in the tutorials.

### ROS Debug tools / Getting information on ROS graph

#### rosnode
`rosnode` displays information about the ROS nodes that are currently running. Its options are:
```bash
rosnode ping          # test connectivity to node
rosnode list          # list active nodes
rosnode info          # print information about node
rosnode machine       # list nodes running on a particular machine or list machines
rosnode kill          # kill a running node
rosnode cleanup       # purge registration information of unreachable nodes
```

#### rqt_graph
rqt_graph is the easiest way to visualize the publish-subscribe relationships between ROS nodes. Just run the following command in terminal:

```bash
rqt_graph
```
An example when [turtlesim](https://wiki.ros.org/turtlesim) node is run

![](https://user-images.githubusercontent.com/17789814/218673038-5f81a12f-f3a7-4c95-875e-adaa602cb42a.jpg)

#### rostopic
`rostopic` tool displays information about the ROS topics that are currently available. Its options are:
```bash
rostopic bw     # display bandwidth used by topic
rostopic delay  # display delay of topic from timestamp in header
rostopic echo   # print messages to screen
rostopic find   # find topics by type
rostopic hz     # display publishing rate of topic
rostopic info   # print information about active topic
rostopic list   # list active topics
rostopic pub    # publish data to topic
rostopic type   # print topic or field type
```

#### rosmsg
`rosmsg` is a command-line tool for displaying information about ROS Message types

```bash
rosmsg show	     # Show message description
rosmsg info	     # Alias for rosmsg show
rosmsg list	     # List all messages
rosmsg md5	     # Display message md5sum
rosmsg package	  # List messages in a package
rosmsg packages  # List packages that contain messages
```

For example, we can view the structure of a message using the following command"
```bash
rosmsg show geometry_msgs/Twist
```

Output:
```
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

#### rosparam
`rosparam` is a command-line tool for getting, setting, and deleting parameters from the ROS Parameter Server.

```bash
rosparam set	   # set parameter
rosparam get	   # get parameter
rosparam load	   # load parameters from file
rosparam dump	   # dump parameters to file
rosparam delete	# delete parameter
rosparam list	   # list parameter names
```

### Create a node

#### What is a node ?
A ROS node, according to [ROS wiki](http://wiki.ros.org/Nodes), is basically a process that performs computation. It is an executable program running inside your application. You will write many nodes and put them into packages.

Nodes are combined into a graph and communicate with each other using ROS topics, services, actions, etc.

For example, one node controls a laser range-finder, one node controls the wheel motors, one node performs localization, one node performs path planning, one Node provides a graphical view of the system, and so on. A ROS node is written with the use of a ROS client library, such as roscpp or rospy.

You can find more information about ROS nodes in [ROS wiki](http://wiki.ros.org/Nodes).

Let's create a very simple node and try to understand the main components that create a node.

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

### Compile and build the package
```bash
cd ~/catkin_ws/ # go to root directory of the packages
catkin build hello_world # Compile a ROS package
source devel/setup.bash # Source the ROS env variable
```
NOTE: We are using `catkin build` from catkin_tools instead of `catkin_make`. To learn more about the differences, refer this [link](https://catkin-tools.readthedocs.io/en/latest/migration.html).

### RUN your first program !!!

Before we run our first program, we need to run the master node i.e. `roscore`. Open a new terminal and run the following command:
```bash
roscore
```
![](https://user-images.githubusercontent.com/17789814/218477414-0a856813-cb88-4335-a61c-238a1c6ff4e6.png)

#### Why is this needed ?
The simple answer is, ROS is a centralized system, a master node (program) is always needed for other nodes and should be executed before other nodes. 
The ROS Master provides name registration and lookup to the rest of the Computation Graph. Without the Master, nodes would not be able to find each other, exchange messages, or invoke services. `roscore` starts this master node.

In the terminal where the file `devel/setup.bash` was sourced, run the following command (if not you can run the command `source devel/setup.bash` in the terminal)
```bash
rosrun hello_world hello_world.py
```
![](https://user-images.githubusercontent.com/17789814/218477454-91c31201-3387-4799-bdb0-f049547643a9.png)


### Create a launch file

#### Launch file explained

One way to execute a program is to launch one node at a time which is what was done above where `roscore` and `hello_world.py` were run separately. This is fine if we have just two nodes, but what do we do if we have multiple nodes? Launching each node one-by-one can get inefficient really quickly.

Fortunately, ROS has a tool called [roslaunch](http://wiki.ros.org/roslaunch) that enables you to launch multiple nodes all at once.

Basic features:
* Before starting any nodes, roslaunch will determine whether roscore is already running and, if not, start it automatically.
* All of the nodes in a launch file are started at roughly the same time. As a result, you cannot be sure about the order in which the nodes will initialize themselves. Well-written ROS nodes don’t care about the order in which they and their siblings start up.
* By default the standard output from launched nodes is not the terminal but a log file (∼/.ros/log/run_id/node_name-number-stdout.log). You can check your run_id folder and browse the rosout.log file.
* To terminate an active roslaunch, use Ctrl-C . This signal will attempt to gracefully shut down each active node from the launch.

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
   * pkg (required): The name of the package the node belongs to.
   * type (required): The name of the executable file that starts the node.
   * name (required): The name of the node. This overrides any name that the node would normally assign to itself in its call to ros::init. It should be a relative name (without mention of any namespaces), not a global name.
   * respawn (optional): If set to “true” roslaunch restarts the node when it terminates.
   * required (optional): If a node is labeled as required then when it is terminated roslaunch terminates all other nodes.
   * launch-prefix (optional): Used to insert the given prefix at the start of the command line that runs the node. The prefix “xterm-e” starts a new terminal window.
   * output (optional): If set to “screen” (for only a single node) allows the standard output of the node be the terminal and not a log file.
   * ns (optional): To set a namespace.

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

Re-build the package and source the environment variable
```bash
cd ~/catkin_ws/src/      # Move to src directory
catkin build hello_world # Compile the ROS package hello_world
source devel/setup.bash  # Source the ROS env variable
```

#### RUN using launch file

```bash
roslaunch hello_world hello_world.launch
```

![](https://user-images.githubusercontent.com/17789814/218476342-c4a626b8-9100-4802-8e99-2d009420f608.png)

### Messages
Nodes communicate with each other by passing messages. A message is simply a data structure, comprising typed fields.
#### Standard messages
Standard messages ([std_msgs](http://wiki.ros.org/std_msgs)) include common message types representing primitive data types and other basic message constructs, such as multiarrays.

Eg. Bool, Byte, ByteMultiArray, Int16, Int32, Float64, Char etc.

#### Common messages
Common messages ([common_msgs](http://wiki.ros.org/common_msgs)) group messages that are widely used by other ROS packages, i.e. messages for:
* actions ([actionlib_msgs](http://wiki.ros.org/actionlib_msgs))
* diagnostics ([diagnostic_msgs](http://wiki.ros.org/diagnostic_msgs))
* geometric primitives ([geometry_msgs](http://wiki.ros.org/geometry_msgs))
* robot navigation ([nav_msgs](http://wiki.ros.org/nav_msgs))
* common sensors ([sensor_msgs](http://wiki.ros.org/sensor_msgs))

### Topics
Topics provide the means for nodes to communicate. Messages are routed via a transport system with publish / subscribe semantics. A node sends out a message by publishing it to a given topic.

The topic is a name that is used to identify the content of the message. A node that is interested in a certain kind of data will subscribe to the appropriate topic. There may be multiple concurrent publishers and subscribers for a single topic, and a single node may publish and/or subscribe to multiple topics. 

Logically, one can think of a topic as a strongly typed message bus. Each bus has a name, and anyone can connect to the bus to send or receive messages as long as they are the right type.

For example:
Consider a node called `/turtlesim` which is controlled through keyboard. The control command is the velocity. The node `/turtlesim` obtains this information by *subscribing* to the topic `/turtle1/cmd_vel`.
There is another node `/teleop_turtle` that converts the keyboard keystrokes into velocity commands and *publishes* the command velocity on the topic `/turtle1/cmd_vel`. This can be visualized as follows:

![](https://user-images.githubusercontent.com/17789814/218678012-caffb4db-63ac-4355-89e4-db6a3fd41017.png)

One can visualize the details of the topic i.e publishers, subscribers and the type of message it contains through the command:
```bash
rostopic info /turtle1/cmd_vel
```

Output:
```
Type: geometry_msgs/Twist

Publishers: 
 * /teleop_turtle (http://phani:39407/)

Subscribers: 
 * /turtlesim (http://phani:39091/)
```

### ROS parameter server
ROS has a centralized parameter server that keeps track of a collection of values, to be queried by the nodes, that basically store configuration information that does not change (much) over time.

The parameters in this server can be accessed through 3 ways:
* Command line
  * To list existing parameters
    `rosparam list`
  * To query the value of the parameter
    `rosparam get parameter_name`
  * To set the value of a parameter:
    `rosparam set parameter_name parameter_value`
  
* The python interface to ROS parameters is straightforward:
  * rospy.get_param()
  * rospy.set_param()
  
* Launch file
  Parameters can also be set from launch files using the param element:
  ```xml
  <param name=”param-name” value=”param-value” />
  ```

## Creating our second package
Lets try to apply the above concepts by creating our second package. The aim of this package is that we create a publisher node which converts the OpenCV images [cv::Mat](https://docs.opencv.org/3.3.0/d3/d63/classcv_1_1Mat.html) format to ROS compatible format [sensor_msgs.Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) and publishers it on a topic. We create another subscriber node, that subscribes to the same topic, and re-converts the data back to OpenCV image and displays it.

### Create a Package
We create a new package called `image_pipeline`. This package depends on the libraries `rospy`, `cv_bridge`, `image_transport` and `sensor_msgs`

```bash
cd ~/catkin_ws/src/     # Move to src directory
catkin_create_pkg image_pipeline rospy cv_bridge image_transport sensor_msgs
```

### Create a publisher node
```bash
cd image_pipeline/ && touch src/publisher.py # Create a empty file
chmod a+x src/publisher.py # Execution permission
```

Open `publisher.py` in a text editor of your preference and copy paste the following code in publisher.py 

```python
#!/usr/bin/env python3

import sys
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class convertCV2toROS:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=100)
        self.bridge = CvBridge()
        self.image_publisher()
    
    def image_publisher(self):
        frequency = rospy.get_param("/image_acquisition/frequency")
        rate = rospy.Rate(frequency)
        input_type = rospy.get_param("/image_acquisition/input_type")
        
        video_capture = None
        if input_type == "camera_usb":
            video_capture = cv2.VideoCapture(0)
        else:
            video_path = rospy.get_param("/image_acquisition/video/video_path_0")
            video_capture = cv2.VideoCapture(video_path)
        
        while not rospy.is_shutdown():
            ret, frame = video_capture.read()
            if frame is not None:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(msg)
                rate.sleep()

def main(args):
    rospy.init_node('CV2_to_ROS')
    try:
        _ = convertCV2toROS()
    except rospy.ROSInterruptException:
        print("Publisher node shutting down")

if __name__ == '__main__':
    main(sys.argv)
```

#### Code explained

```python
import sys
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
```
 * rospy: To use ROS features such as Topics, Services, Parameters etc. through python
 * sys: To pass/access arguments from interpretor to script and vice versa
 * sensor_msgs.msg: To re-use the Image message type for publishing and subscribing
 * cv_bridge: To convert ROS images to OpenCV images
 * cv2: To re-use features of OpenCV library

```python
class convertCV2toROS:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=100)
        self.bridge = CvBridge()
        self.image_publisher()
```
Define a class convertCV2toROS. 
`self.image_pub = rospy.Publisher("image_topic", Image, queue_size=100)` declares that the node is publishing to the *image_topic* topic using the message type Image. Image here is actually the class sensor_msgs.msg.Image.

In the following two lines ,we initialize a CvBridge object and call the method `image_publisher()`

```python
    def image_publisher(self):
        frequency = rospy.get_param("/image_acquisition/frequency")
        rate = rospy.Rate(frequency)
        input_type = rospy.get_param("/image_acquisition/input_type")
```
We define a method called `image_publisher`.

To access the value assigned in the frequency parameter (stored in ROS Parameter Server) we use the command `rospy.get_param("/image_acquisition/frequency")`. Similarly for `input_type = rospy.get_param("/image_acquisition/input_type")`. These parameters are configured through the config file `param.yaml`. 

`rospy.Rate(frequency)` is a rate object to control the publishing rate (In this example controlled through the frequency of while loop)

```python
        video_capture = None
        if input_type == "camera_usb":
            video_capture = cv2.VideoCapture(0)
        else:
            video_path = rospy.get_param("/image_acquisition/video/video_path_0")
            video_capture = cv2.VideoCapture(video_path)
```
Logic to select the input for the video which is being published


```python
        while not rospy.is_shutdown():
            ret, frame = video_capture.read()
            if frame is not None:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(msg)
                rate.sleep()
```

`msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")` each frame of the video is converted to ROS compatible image i.e. sensor_msgs.Image type. `self.image_pub.publish(msg)` publishes the msg on the topic `image_topic`

```python
def main(args):
    rospy.init_node('image_converter')
    try:
        _ = convertCV2toROS()
    except rospy.ROSInterruptException:
        pass
```
In addition to the standard Python `__main__` check, this catches a `rospy.ROSInterruptException` exception, which can be thrown by `rospy.sleep()` and `rospy.Rate.sleep()` methods when Ctrl-C is pressed or your Node is otherwise shutdown. The reason this exception is raised is so that you don't accidentally continue executing code after the sleep().

### Create a config file
```bash
cd ~/catkin_ws/src/image_pipeline/ && mkdir config/ && touch config/param.yaml
```
Copy paste the following code in `param.yml`

```xml
image_acquisition:
  input_type: "video" #camera_usb, video
  
  camera_usb: 
      camera_node: 1
  
  video:
      video_node: 1
      video_path_0: "/home/sirhawk/catkin_ws/src/image_pipeline/test_data/video/test_1.mp4"
  
  frequency: 10
```

**NOTE**: Do not forget to edit the video file path in video_path_0

### Create a subscriber node
```bash
cd image_pipeline/ && touch src/subscriber.py # Create a empty file
chmod a+x src/subscriber.py # Execution permission
```

Copy paste the following code in `subscriber.py`

```python
#!/usr/bin/env python3

import sys
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


class convertROStoCV2:
    def __init__(self):
        self.image_sub = rospy.Subscriber("image_topic", Image, self.callback)
        self.bridge = CvBridge()
        
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Display", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('ROS_to_CV2', anonymous=True)
    _ = convertROStoCV2()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
```

#### Code explained

The code for `subscriber.py` is very similar to `publisher.py` except that we have introduced a new callback-based mechanism for subscribing to messages.

`self.image_sub = rospy.Subscriber("image_topic", Image, self.callback)`
This declares that your node is subscribes to `image_topic` which of the type `sensor_msgs.msg.Image`. When new messages are received, callback (here it is self.callback) is invoked with the message as the first argument. 

We also changed up the call to `rospy.init_node()`. We've added the `anonymous=True` keyword argument. ROS requires that each node have a unique name. If a node with the same name comes up, it bumps the previous one. This is so that malfunctioning nodes can easily be kicked off the network. The `anonymous=True` flag tells rospy to generate a unique name for the node so that you can have multiple listener.py nodes run easily.

The final addition, `rospy.spin()` simply keeps your node from exiting until the node has been shutdown. Unlike roscpp, `rospy.spin()` does not affect the subscriber callback functions, as those have their own threads.

### Create a Launch file
```bash
mkdir launch && touch launch/image_pipeline.launch # Create a empty lauch file
```
Copy paste the following contents in the launch file

```xml
<launch>
    <!-- Load the parameter param.yaml file to the ROS Parameter Server -->
    <rosparam file="$(find image_pipeline)/config/param.yaml" />
    
    <!-- Launch the publisher node -->
    <node pkg="image_pipeline" type="publisher.py" name="publisher"  output="screen">
    </node>

    <!-- Launch the subscriber node -->
    <node pkg="image_pipeline" type="subscriber.py" name="subscriber"  output="screen">
    </node>
</launch>
```

### Compile and build the package
```bash
cd ~/catkin_ws/                  # go to root directory of the packages
catkin build image_pipeline      # Compile a ROS package
source devel/setup.bash          # Source the ROS env variable
```

### Launch your publisher and subscriber nodes
```bash
roslaunch image_pipeline image_pipeline.launch
```

#### Output

Data acquired by the subscriber node
![](https://user-images.githubusercontent.com/17789814/218756726-3c0246fd-5af3-411d-b364-1f743b8d274b.png)

ROS Graph
![](https://user-images.githubusercontent.com/17789814/218757432-611b248c-a2ee-40d7-8ea5-9af67fabbd61.png)

```bash
rostopic info /image_topic
```

```
Type: sensor_msgs/Image

Publishers: 
 * /publisher (http://phani:46223/)

Subscribers: 
 * /subscriber (http://phani:37603/)
```

### Services
ROS service calls communication has the following features:

* It is bi-directional.
* It one-to-one.

A client node sends some data (called a request) to a server node and waits for a reply. The server, having received this request, takes some action (computing something, configuring hardware or software, changing its own behavior, etc.) and sends some data (called a response) back to the client.

![](https://user-images.githubusercontent.com/17789814/218767053-7f393703-e0af-478c-9a15-cfdbbdd18980.png)

A service data type determines the content of messages by a collection of named fields, and is divided into two parts, the request and the response.

## Creating our third package

### Create a Package
We create a new package called `turtlesim_services`. This package depends on the libraries `rospy`, `turtlesim`, `geometry_msgs`, `std_msgs` and `message_generation

```bash
cd ~/catkin_ws/src/     # Move to src directory
catkin_create_pkg turtlesim_services rospy turtlesim geometry_msgs std_msgs message_generation
```

### Create a publisher-server node
```bash
cd turtlesim_services/ && touch src/pub_vel.py # Create a empty file
chmod a+x src/pub_vel.py # Execution permission
```

Copy paste the following code in `pub_vel.py` 

```python
#!/usr/bin/env python3

import sys
import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist

class turtle:
    def __init__(self):
        self.server = rospy.Service('toggle_forward', Empty, self.toggleForward)
        self.pub_vel = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=100)
        
        self.forward = False
        self.twist = Twist()

        self.commandVel()

    def toggleForward(self, req):
        self.forward = not self.forward
        rospy.loginfo("Now sending %s commands", "Forward" if self.forward else "Rotate")
        return EmptyResponse()

    def commandVel(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():

            msg = self.twist
            msg.linear.x = 1.0 if self.forward else 0.0
            msg.angular.z = 0.0 if self.forward else 1.0
            
            self.pub_vel.publish(msg)
            rate.sleep()

def main(args):
    rospy.init_node("toggle_forward_server")
    _ = turtle()
    
if __name__ == "__main__":
    main(sys.argv)
```

#### Code explained

### Create a Launch file
```bash
mkdir launch && touch launch/turtlesim_services.launch # Create a empty lauch file
```
Copy paste the following contents in the launch file

```xml
<launch>

</launch>
```

### Compile and build the package
```bash
cd ~/catkin_ws/                      # go to root directory of the packages
catkin build turtlesim_services      # Compile a ROS package
source devel/setup.bash              # Source the ROS env variable
```

### Launch your nodes
```bash
roslaunch 
```

**Pro Tip** :grin: : When do we modity `CMakelists.txt` ?

We have to modify the CmakeLists.txt in python too but not as systematically as with roscpp. In python if you want to run a simple node with no dependencies you just have to make sure to use the command `chmod +x your_node.py` to get an executable that rosrun can use. In cpp, whenever you create a node you have to create the executable from the CMakeLists.txt by adding the following line:

`add_executable(talker src/your_node.cpp)`

We don't need to do that in python. But we will have to modify the CMakeList if you want to create a custom message, to use a client/server and to list all the dependencies of your package. ([Ref](https://answers.ros.org/question/306236/do-i-have-to-modify-cmakeliststxt-for-a-python-node/))
