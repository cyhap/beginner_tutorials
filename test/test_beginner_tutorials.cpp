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
  EXPECT_TRUE(expString.compare(srv.response.oldString) == 0);

  std::string secondTestStr("Test Str 2.");
  srv.request.newString = secondTestStr;

  client.call(srv);

  EXPECT_TRUE(firstTestString.compare(srv.response.oldString) == 0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_talker_change_base_str_service");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
