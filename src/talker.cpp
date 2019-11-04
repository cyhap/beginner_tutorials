/*
 * @copyright Copyright 2019 <Corbyn Yhap>
 *
 * @file talker.cpp
 *
 * @brief A simple publisher that publishes ROS messages on the topic:
 * "chatter"
 *
 */

#include <sstream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/change_base_string.h"

std::string baseString = "Corbyn's Publisher Node.";

/**
 * @brief This function is called when the change_base_str service is invoked.
 *
 * @param beginner_tutorials::change_base_string::Request the string that will
 * be used to update the base string.
 *
 * @param beginner_tutorials::change_base_string::Response the string that was
 * previously the base string.
 *
 * @return bool whether or not updating the base string was successful.
 */
bool changeBaseStr(beginner_tutorials::change_base_string::Request &req,
                   beginner_tutorials::change_base_string::Response &resp) {
  bool success = true;
  resp.oldString = "";
  ROS_DEBUG_STREAM("The Change Base String Service was succesfullly Called!");
  auto sameStr = baseString.compare(req.newString) == 0;
  ROS_WARN_STREAM_COND(sameStr, "That was already the base string.");
  resp.oldString = baseString;
  baseString = req.newString;
  return success;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Registering the service with the master.
  ros::ServiceServer server = n.advertiseService("change_base_str",
                                                  &changeBaseStr);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  auto chatter_pub = n.advertise < std_msgs::String > ("chatter", 1000);

  double messageFrqHz;
  std::string frqParamName = "~messageFrqHz";
  bool paramFound = ros::param::get(frqParamName, messageFrqHz);
  if (!paramFound) {
    ROS_FATAL_STREAM(
        "Could not determine the publish Frequency. (" << frqParamName << ")");
    ROS_ERROR_STREAM(
        "Consider setting: " << frqParamName << " with rosparam and running.");
    exit(1);
  }
  ros::Rate loop_rate(messageFrqHz);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  auto count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << baseString << " " << count;
    msg.data = ss.str();

    ROS_INFO_STREAM(msg.data);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
