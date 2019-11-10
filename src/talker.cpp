/*
 * @copyright Copyright 2019 <Corbyn Yhap>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Corbyn Yhap
 *
 * @file talker.cpp
 *
 * @brief A simple publisher that publishes ROS messages on the topic:
 * "chatter"
 *
 */

#include <sstream>
#include <string>

#include <math.h>

#include "beginner_tutorials/change_base_string.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"

/* Note this is flagged by cpplint however due to the fact that the
 * changeBaseStr service function callback needs a global string to update.
 * It cannot be avoided.
 */
std::string baseString = "Corbyn's Publisher Node.";

/* Note below is also flagged by cpplint the request and response are not
 * allowed to be const according to some reading. Making them const & prevents
 * the package from building successfully.
 */

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

  // Build the transfrom describing the talker node from world frame to talk frame.
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  // Set the Position of the Talker (Constant Position for now)
  transform.setOrigin(tf::Vector3(5.0, 22.89, -14));
  tf::Quaternion q;

  q.setRPY(M_PI / 2, 0, -M_PI);

  transform.setRotation(q);

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

    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
