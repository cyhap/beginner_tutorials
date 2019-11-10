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
 * @file test_beginner_tutorials.cpp
 *
 * @brief This file evaluates the service of the talker node. Makes sure you can
 * change the base string of the talker node as desired.
 *
 */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "beginner_tutorials/change_base_string.h"

// The node handle used by the testing framework. (Initialized in main)
std::shared_ptr<ros::NodeHandle> nh;

// Test to determine whether  or not the base string can be changed via the
// service.
TEST(TalkerPublishing, changeStrService) {
  ros::ServiceClient client = nh
      ->serviceClient<beginner_tutorials::change_base_string>(
      "change_base_str");

  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);

  std::string firstTestString("Test Change String.");
  beginner_tutorials::change_base_string srv;
  srv.request.newString = firstTestString;

  client.call(srv);

  std::string expString = "Corbyn's Publisher Node.";
  EXPECT_EQ(expString.compare(srv.response.oldString), 0);

  std::string secondTestStr("Test Str 2.");
  srv.request.newString = secondTestStr;

  client.call(srv);

  EXPECT_EQ(firstTestString.compare(srv.response.oldString), 0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_talker_change_base_str_service");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
