/*
 * Copyright (c) 2018, Houston Mechatronics Inc., JD Yamokoski
 * Copyright (c) 2012, Clearpath Robotics, Inc., Alex Bencz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <mocap_optitrack/rigid_body_publisher.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// このconvertはまだテスト．適当に実装しているので修正が必要だと思われる．
namespace tf2 {
void convert(geometry_msgs::msg::PoseStamped pose, tf2::Transform& transform) {
  transform.setOrigin(tf2::Vector3{pose.pose.position.x, pose.pose.position.y,
                                   pose.pose.position.z});
  transform.setRotation(
      tf2::Quaternion{pose.pose.orientation.x, pose.pose.orientation.y,
                      pose.pose.orientation.z, pose.pose.orientation.w});
}
}  // namespace tf2

namespace mocap_optitrack {

namespace utilities {
geometry_msgs::msg::PoseStamped getRosPose(RigidBody const& body,
                                           bool newCoordinates) {
  geometry_msgs::msg::PoseStamped poseStampedMsg;
  if (newCoordinates) {
    // Motive 1.7+ coordinate system
    poseStampedMsg.pose.position.x = -body.pose.position.x;
    poseStampedMsg.pose.position.y = body.pose.position.z;
    poseStampedMsg.pose.position.z = body.pose.position.y;

    poseStampedMsg.pose.orientation.x = -body.pose.orientation.x;
    poseStampedMsg.pose.orientation.y = body.pose.orientation.z;
    poseStampedMsg.pose.orientation.z = body.pose.orientation.y;
    poseStampedMsg.pose.orientation.w = body.pose.orientation.w;
  } else {
    // y & z axes are swapped in the Optitrack coordinate system
    poseStampedMsg.pose.position.x = body.pose.position.x;
    poseStampedMsg.pose.position.y = -body.pose.position.z;
    poseStampedMsg.pose.position.z = body.pose.position.y;

    poseStampedMsg.pose.orientation.x = body.pose.orientation.x;
    poseStampedMsg.pose.orientation.y = -body.pose.orientation.z;
    poseStampedMsg.pose.orientation.z = body.pose.orientation.y;
    poseStampedMsg.pose.orientation.w = body.pose.orientation.w;
  }
  return poseStampedMsg;
}
}  // namespace utilities

RigidBodyPublisher::RigidBodyPublisher(rclcpp::Node* nh,
                                       Version const& natNetVersion,
                                       PublisherConfiguration const& config)
    : config(config), tfPublisher(tf2_ros::TransformBroadcaster(nh)) {
  if (config.publishPose)
    posePublisher = nh->create_publisher<geometry_msgs::msg::PoseStamped>(
        config.poseTopicName, 1000);

  if (config.publishPose2d)
    pose2dPublisher = nh->create_publisher<geometry_msgs::msg::Pose2D>(
        config.pose2dTopicName, 1000);

  // Motive 1.7+ uses a new coordinate system
  useNewCoordinates = (natNetVersion >= Version("1.7"));

  ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  system_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  buffer = std::make_shared<tf2_ros::Buffer>(system_clock);
  // Silence error about dedicated thread's being necessary
  buffer->setUsingDedicatedThread(true);
}

RigidBodyPublisher::~RigidBodyPublisher() {}

void RigidBodyPublisher::publish(rclcpp::Time const& time,
                                 RigidBody const& body) {
  // don't do anything if no new data was provided
  if (!body.hasValidData()) {
    return;
  }

  // NaN?
  if (body.pose.position.x != body.pose.position.x) {
    return;
  }

  geometry_msgs::msg::PoseStamped pose =
      utilities::getRosPose(body, useNewCoordinates);
  pose.header.stamp = time;

  if (config.publishPose) {
    pose.header.frame_id = config.parentFrameId;
    posePublisher->publish(pose);
  }

  tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
                    pose.pose.orientation.z, pose.pose.orientation.w);

  // publish 2D pose
  if (config.publishPose2d) {
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = pose.pose.position.x;
    pose2d.y = pose.pose.position.y;

    pose2d.theta = tf2::getYaw(q);
    pose2dPublisher->publish(pose2d);
  }

  if (config.publishTf) {
    // publish transform
    tf2::Transform transform;
    // transform.setOrigin(tf2::Vector3(pose.pose.position.x,
    // pose.pose.position.y,
    //                                  pose.pose.position.z));

    // Handle different coordinate systems (Arena vs. rviz)
    // transform.setRotation(q);

    rclcpp::Time rclcpp_time = system_clock->now();
    tf2::TimePoint tf2_time(
        std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

    geometry_msgs::msg::TransformStamped transform_stamped;
    auto a =
        tf2::Stamped<tf2::Transform>(transform, tf2_time, config.parentFrameId);

    geometry_msgs::msg::Transform gt;
    tf2::convert(pose, transform);
    tf2::convert(transform, gt);

    transform_stamped.header.stamp = time;
    transform_stamped.header.frame_id = config.parentFrameId;
    transform_stamped.child_frame_id = config.childFrameId;
    transform_stamped.transform = gt;
    tfPublisher.sendTransform(transform_stamped);
  }
}

RigidBodyPublishDispatcher::RigidBodyPublishDispatcher(
    rclcpp::Node* nh, Version const& natNetVersion,
    PublisherConfigurations const& configs) {
  for (auto const& config : configs) {
    rigidBodyPublisherMap[config.rigidBodyId] = RigidBodyPublisherPtr(
        new RigidBodyPublisher(nh, natNetVersion, config));
  }
}

void RigidBodyPublishDispatcher::publish(
    rclcpp::Time const& time, std::vector<RigidBody> const& rigidBodies) {
  for (auto const& rigidBody : rigidBodies) {
    auto const& iter = rigidBodyPublisherMap.find(rigidBody.bodyId);

    if (iter != rigidBodyPublisherMap.end()) {
      (*iter->second).publish(time, rigidBody);
    }
  }
}

}  // namespace mocap_optitrack