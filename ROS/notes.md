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

![Visualization Prompt](Nodes-TopicandService.gif)

^ From tutorial in documentation listed.

### ROS 2 Graph
 - A network showing elements that process and communicate data.

### Nodes
 - Modular components responsible for a single task, that can send and receive data from other nodes via topics, services, actions, and parameters.
 - A single executable can contain one or more nodes.



# Tools and Commands

## Commands
 - ```ros2 run <package> <executable>``` launches an executable from a package
 - ```ros2 node list``` shows the names of all *running* nodes
 - ```ros2 node info <node_name>``` shows a list of subscribers, publishers, services, and actions, i.e. ROS graph connections.
 - ```ros2 run <package> <executable> --ros-args --remap <key>:=<value>```
  - Example: ```ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle```

## Tools

### Turtlesim
 - lightweight simulator for learning ROS 2
 - Requires running ```unset GTK_PATH``` if invoked in vs-code due to GTK+ (GUI widget) having the wrong search directories.

### rqt
 - GUI tool for ROS 2. Equivalent to ros2 tool?
 - If plugins fail to load, run ```rqt --force-discover```