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
 * @file talker_tf_broadcaster.cpp
 *
 * @brief A node that handles the talker's position and broadcasts it using the
 * tf broadcasting module.
 *
 */

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

#include <math.h>


/**
 * This node simply broadcasts a constant position of the talker.
 */
int main(int argc, char **argv) {

  ros::init(argc, argv, "talker_tf_broadcaster");

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  // Set the Position of the Talker (Constant Position for now)
  transform.setOrigin(tf::Vector3(5.0, 22.89, -14));
  tf::Quaternion q;

  q.setRPY(M_PI / 2, 0, -M_PI);

  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

  ros::spin();

  return 0;
}
