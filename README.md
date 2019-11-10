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
- Navigate to your catkin_ws src directory.
```
cd <Catkin Workspace>/src
```

- Pull the package from github.
```
git clone https://github.com/cyhap/beginner_tutorials.git
```
- The beginner_tutorials directory should now be in your <Catkin_Workspace>/src folder.

- Change directories to <Catkin_Workspace> and run catkin_make install
```
cd <Catkin_Workspace>
catkin_make install
```
- Note the install is only necessary the first time the package is built to ensure that the services are generated and installed.
- Make sure that your catkin workspace is sourced in order to run this package in another terminal. 
```
source <Catkin_Workspace>/devel/setup.bash
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
### Talker Node:
- Begin Running the talker Node.
```
rosrun beginner_tutorials talker _messageFrqHz:=10
```
- One should observe the following Message being printed to screen from the talker node: "Corbyn's Publisher Node #" where # will be a count of how many messages have been published.

### Listener Node:
- Open a terminal and being running roscore
```
roscore
```
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
## Both Nodes Simultaneously (USING ROS LAUNCH):
- Open a new terminal and again make sure that the <Catkin_Workspace>/devel/setup.bash has been sourced.
```
source <Catkin_Workspace>/devel/setup.bash
```
- In order to run both the talker and listener nodes one can use the launch file provided. Simply use the following command in your terminal:
```
roslaunch beginner_tutorials talkAndListen.launch
```
- There are three optional arguments for the roslaunch commandline input. One controls the frequency that the messages are published while the
second allows the user to declare the namespace of both nodes from the command line. Finally, the last option allows the user to record all active topics when this launched. Be wary that if other nodes are running their published topics will be captured in the bag file as well. An example using all arguments is provided below:
```
roslaunch beginner_tutorials talkAndListen.launch desiredNS:="a_new_ns" pubFrqHz:=5 recordChatter:=1
```
The desiredNS argument lets you set the namespace that both the talker and listener nodes appear in.
The pubFrqHz argument allows you to modify the rate at which the messages are published.
The recordChatter argument allows you to record a bag file titled beginnerTutorials.bag.
Note the rosbag file will be located under the ~/.ros directory.

## Bag File Playback
To inspect the contents of the bagfile run the following command:
```
rosbag info <Path to bag File in Results Folder>
```

Use the listener node to observe the messages that were saved in the bag file. The namespace of listener needs to be updated to subscribe to the messages from the bagfile. The namespace it was saved to was "/private/chatter" because a namespace was not provided when the launch file was run initially, thus it used the default namespace of private.

```
rosrun beginner_tutorials listener __ns:=private
```
Now one can start playing the bagfile.
```
rosbag play <Path to bag File in Results Folder>
```
This is an example of the previous command specific to my local machine
```
rosbag play ~/SoftwareDevForRobotics/catkin_ws/src/beginner_tutorials/results/week11_HW_results/beginnerTutorials.bag
```


## Modifying the Ouput of the Talker Node (ROS Service):
The talker node provides a service that allows you to change the base message to something new. It then returns the message that was previously
being printed by the talker node. In order to change the base string that is being printed by the talker node run the following command in
your terminal:
```
rosservice call /change_base_str <"Whatever String you desire."> 
```
This will update the base string that is being published. (The counter will continue to increment and not reset)

Note that the /change_base_str service is under the namespace provided. So if running the talker node using rosrun, then the service will appear
as listed above. If running using another namespace, such as the "private" whic is the namespace used by default when
running using roslaunch without a desiredNS parameter, the service name must also include that namespace. In the case of this example the
service would be "/private/change_base_str" instead. Consider using rosservice list to view the available services.

## Inspecting TF Frames.
 The tf frames are only present from world frame to the talk frame as assigned. The tf graph is provided in the results/week11_HW_results directory in frames.pdf. To inspect the tf frames while the nodes are running one may run the talker node individually or have it running in the collection of nodes that run using roslaunch. See above sections for instructions on both of those operations.


