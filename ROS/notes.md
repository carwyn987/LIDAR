From https://docs.ros.org/en/humble/Tutorials.html 
Running Humble Hawksbill (```echo $ROS_DISTRO```)

# Configuration

### Setup
 - Source setup file: ```source /opt/ros/humble/setup.bash```
 - Set domain id: ```export ROS_DOMAIN_ID=0```
 - Limit communication to localhost: ```export ROS_LOCALHOST_ONLY=1```

### Debugging
 - Print environment variables: ```printenv | grep -i ROS```

# Basic Concepts

![Visualization Prompt](media/Nodes-TopicandService.gif)

^ From tutorial in documentation listed.

### ROS 2 Graph
 - A network showing elements that process and communicate data.

### Nodes
 - Modular components responsible for a single task, that can send and receive data from other nodes via topics, services, actions, and parameters.
 - A single executable can contain one or more nodes.

### Topics
 - A topic is a communication medium that sits as an intermediary between two nodes for the purpose of communication. This lets one node publish to it, and another node subscribe to it modularly. More nodes can subscribe or publish to a topic in the future.
 - When seeing a type like ```Type: geometry_msgs/msg/Twist```, this implies a package called ```geometry_msgs```, containing a message called ```Twist```.

# Tools and Commands

## Commands
 - ```ros2 run <package> <executable>``` launches an executable from a package
 - ```ros2 topic list [-t]``` shows a list of all topics currently active in the system [and types].
 - ```ros2 node list``` shows the names of all *running* nodes
 - ```ros2 node info <node_name>``` shows a list of subscribers, publishers, services, and actions, i.e. ROS graph connections.
 - ```ros2 run <package> <executable> --ros-args --remap <key>:=<value>```
  - E.g.: ```ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle```
 - ```ros2 topic echo <topic_name>``` listens and prints all traffic on a topic to stdout.
 - ```ros2 topic info <topic_name>``` to see publisher and subscriber count + type.
 - ```ros2 interface show <package>/<msg>/<msg_type>``` shows the expected contents of a msg_type msg
  - E.g. ```ros2 interface show geometry_msgs/msg/Twist```
 - ```ros2 topic pub <topic_name> <msg_type> '<args>'``` publishes to a topic from the cmdline.
  - Note: ```<args>``` should contain YAML syntax
  - Note: This also seems to publish repeatedly unless the ```--once``` flag is used. Alternatively a ```--rate``` in Hz can be specified.
  - If a timestamp is needed, use ```auto``` (in the case of a header like ```std_msgs/msg/Header```) or ```now``` (in the case of a field such as ```builtin_interfaces/msg/Time```) in the YAML args.
 - ```ros2 topic hz <topic_name>``` can be used to get statistics about the time and frequency of publishing.

## Tools

### Turtlesim
 - lightweight simulator for learning ROS 2
 - Requires running ```unset GTK_PATH``` if invoked in vs-code due to GTK+ (GUI widget) having the wrong search directories.

### rqt
 - GUI tool for ROS 2. Equivalent to ros2 tool?
 - If plugins fail to load, run ```rqt --force-discover```
 - To see the current Node/Topic/Connections graph, run ```rqt_graph``` or ```rqt``` and selecting Plugins > Introspection > Node Graph
  - Note: use the refresh button and you can see cmdline invocations (in progress) graphically as nodes.


# Drivers:

http://wiki.ros.org/Drivers/Tutorials 
http://wiki.ros.org/velodyne 
https://github.com/ros-drivers/velodyne/tree/ros2 

sudo apt-get install ros-humble-velodyne
sudo ifconfig enx3c18a0432566 169.254.133.69


USB Ethernet
Set MAC address in Identity + Permenent + automatic
IPv4 link-local only
IPv6 disabled
