# This file serves as a general overview (interview prep-ish) of ROS2

ROS2 (Robotics Operating System 2) is a framework for robotic development that is supported on Ubuntu ~22+, and probably embedded systems. It contains many libraries that make robotics development easy and fast, including a visualization software rviz, tutorial packages including turtlesim, simulation software such as Gazebo, and more.
 - Decentralized communication pipeline - Data Distribution Service (DDS)

ROS is highly modular, and breaks development into a few components.
 - Over and under view (?)
 - Nodes - file(s) of c++ / python code that encapsulate a component. This could be a camera, sensor, or actuator. Nodes are usually compiled, but have "node parameters" to allow for reuse (polymorphism?).
 - Topics - components that represent mediums of communication. Topics are hubs that get sent messages (usually a specified type). You can specify a "bag file" to record / log messages during a run, and replay them later.
 - Code is encapsulated in packages.

ROS has a few communication paradigms.
 - publisher-subscriber. Nodes send messages to topics, and other nodes subscribe to listen.
 - services - sending requests to other nodes for information
 - actions - sending a desired state to another node, and receiving feedback (progress updates) and a results eventually.


Notable packages
 - turtlesim
 - rviz
 - gazebo - requires cpp(?)

Notable build and setup tools
 - catkin
 - colcon - e.g. ```colcon build``` in a package to build executables and ros2 run commands.
 - CMakeList.txt - cmake format makefile
 - ament-python, ament-cpp