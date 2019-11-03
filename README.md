# beginner_tutorials
ROS Beginner Tutorials

## Overview:
This ROS Package contains source code to create two rosnodes titled talker and listener. One node simply publishes a string message at a 10 Hz rate to a chatter topic. The other node subscribes the chatter topic and prints "I heard:" followed by the string message that it observed being published to the chatter topic.

Below you will find a summary of the files that were added when completing each tutorial that was followed in order to build this ROS Package. 
Navigating the Wiki:
- It appears that there are no files to add when completing this tutorial.

Navigating the File System:
- It appears that there are no files added to the repository when completing this tutorial.

Creating Package:
- Used the catkin_create_pkg command as outlined in the tutorial. This added a number of additional directories and files. Modified the manifest (package.xml)

Building Package:
- Note the building of this package was actually an instruction in Creating Package tutorial as well. No new files were added. (The build directory resides in the catkin_ws and we were instructed not to include that in our github repository.)

Understanding Nodes:
-  This tutorial primarily focused on running the ros nodes and understanding how they work. No new files were added.

Understanding Topics:
- Again this tutorial did not require the addition of new fields. Topics and messages were explored along with the usage of rqt_graph and rqt_plot

Writing Publisher Subscriber:
- Wrote the Talker and Listener Rosnodes outlined by the tutorial. Modified the cmakelist accordingly.

Examining Publisher Subscriber:
- Ran the publisher and subscriber that was written and built in the last tutorial. Note in order to run these packages the setup.bash script needed to be sourced in each terminal in order to run each node. No additional files added.

Getting Started with roswtf:
- This tutorial shows some example of how to use roswtf and examples of how it may diagnose some problems. This did not introduce new files.

Understanding Servives Params:
-  This tutorial demonstrated how to use services and parameters from the command line to update the background color of the turtle_sim
 example. The background color of turtle sim is saved in as a collection of r g and b parameters. Turtle_sim provides services to update those
parameters.

## Dependencies:
This ros package was developed in ros-kinetic. No other ros distributions are guaranteed to support this package. Note the source code expects to be built using the c++11 standard. (Outlined in the CMakelist.txt file)

## Build instructions:
In order to use the rosnodes provided by this package they must first be built in a catkin workspace. For the instructions the Catkin Workspace directory shall be referred to as <Catkin Workspace> and should be replaced with the users full path to their Catkin Workspace.
- Pull the package from github.
```
git clone https://github.com/cyhap/beginner_tutorials.git <aName>
```
- Place the beginner_tutorials directory in your <Catkin_Workspace>/src folder.
```
mv <aName>/catkin_ws/src/beginner_tutorials <Catkin_Workspace>/src
```
- Change directories to <Catkin_Workspace> and run catkin_make
```
cd <Catkin_Workspace>
catkin_make
```

## Run instructions:
- Open a terminal and being running roscore
```
roscore
```
- Open a new terminal and make sure the <Catkin_Workspace>/devel/setup.bash has been sourced. In order to find the newly built packages.
```
source <Catkin_Workspace>/devel/setup.bash
```
- Run the talker node.
```
rosrun beginner_tutorials talker
```
- One should observe the following Message being printed to screen from the talker node: "Corbyn's Publisher Node #" where # will be a count of how many messages have been published.

- Open a new terminal and again make sure that the <Catkin_Workspace>/devel/setup.bash has been sourced.
```
source <Catkin_Workspace>/devel/setup.bash
```
- Run the listener node.
```
rosrun beginner_tutorials listener
```
- One should observe the follwing Message being printed to screen from the listener node:"I heard: Corbyn's Publisher Node #" where # will be a number.

- Note the rostopic that talker publishes and listener subscribes to is called /chatter

## Caution
Upon testing these instructions it seems like there are two things over concern. Upon building for the first time there seems to be a developer catkin warning pointing out that "beginner_tutorials_generate_messages_cpp"  of target listener and target does not exist. This was a step from the tutorial however I am not 100% sure I understand what that dependency is being added. According to the reading it seems like it would be there to ensure the message dependencies for the ros nodes being built exist. However, if catkin_make is run again, these warnings no longer appear and the nodes still function. I was unable to determine the root cause of these warnings.

Secondly, I have made two different catkin workspaces on my machine in order to facilitate testing. The second workspace that I have seems to also source the original catkin workspave that I originally made. It is unclear to me whether this additional path will appear for other users as well. It should not matter, as the proper path is still added first and the nodes should still build and run as expected however this was worrysome. I'd like to discuss further with the TAs.
