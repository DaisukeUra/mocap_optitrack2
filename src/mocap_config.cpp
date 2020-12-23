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
#include "mocap_optitrack/mocap_config.h"

// パラメータサーバから一気にパラメータをとってくるなにかぽい
// #include <XmlRpcValue.h>

namespace mocap_optitrack {

namespace impl {
template <typename T>
bool check_and_get_param(rclcpp::Node& config_node, std::string const& key,
                         T& value) {
  return false;
}

template <>
bool check_and_get_param<std::string>(rclcpp::Node& config_node,
                                      std::string const& key,
                                      std::string& value) {
  if (config_node.get_parameter(key).get_type() ==
      rclcpp::ParameterType::PARAMETER_STRING) {
    value = config_node.get_parameter(key).as_string();
    return true;
  }

  return false;
}
}  // namespace impl

// Server description defaults
const int ServerDescription::Default::CommandPort = 1510;
const int ServerDescription::Default::DataPort = 9000;
const std::string ServerDescription::Default::MulticastIpAddress = "224.0.0.1";

// Param keys
namespace rosparam {
namespace keys {
const std::string MulticastIpAddress = "optitrack_config/multicast_address";
const std::string CommandPort = "optitrack_config/command_port";
const std::string DataPort = "optitrack_config/data_port";
const std::string Version = "optitrack_config/version";
const std::string RigidBodies = "rigid_bodies";
const std::string PoseTopicName = "pose";
const std::string Pose2dTopicName = "pose2d";
const std::string ChildFrameId = "child_frame_id";
const std::string ParentFrameId = "parent_frame_id";
}  // namespace keys
}  // namespace rosparam

ServerDescription::ServerDescription()
    : commandPort(ServerDescription::Default::CommandPort),
      dataPort(ServerDescription::Default::DataPort),
      multicastIpAddress(ServerDescription::Default::MulticastIpAddress) {}

void NodeConfiguration::fromRosParam(rclcpp::Node::SharedPtr nh,
                                     ServerDescription& serverDescription,
                                     PublisherConfigurations& pubConfigs) {
  // Get server cconfiguration from ROS parameter server
  if (nh->has_parameter(rosparam::keys::MulticastIpAddress)) {
    nh->get_parameter(rosparam::keys::MulticastIpAddress,
                      serverDescription.multicastIpAddress);
  } else {
    // RCLCPP_WARN_STREAM isn't implemented yet in dashing.
    RCLCPP_WARN(nh->get_logger(),
                "Could not get multicast address, using default: %s",
                serverDescription.multicastIpAddress);
  }

  if (nh->has_parameter(rosparam::keys::CommandPort)) {
    nh->get_parameter(rosparam::keys::CommandPort,
                      serverDescription.commandPort);
  } else {
    // RCLCPP_WARN_STREAM isn't implemented yet in dashing.
    RCLCPP_WARN(nh->get_logger(),
                "Could not get command port, using default: %d",
                serverDescription.commandPort);
  }

  if (nh->has_parameter(rosparam::keys::DataPort)) {
    nh->get_parameter(rosparam::keys::DataPort, serverDescription.dataPort);
  } else {
    // RCLCPP_WARN_STREAM isn't implemented yet in dashing.
    RCLCPP_WARN(nh->get_logger(), "Could not get data port, using default: %d",
                serverDescription.dataPort);
  }

  if (nh->has_parameter(rosparam::keys::Version)) {
    nh->get_parameter(rosparam::keys::Version, serverDescription.version);
  } else {
    // RCLCPP_WARN_STREAM isn't implemented yet in dashing.
    RCLCPP_WARN(nh->get_logger(), "Could not get server version, using auto");
  }

  // Parse rigid bodies section
  if (nh->has_parameter(rosparam::keys::RigidBodies)) {
    rclcpp::Parameter param;
    rclcpp::ParameterValue;

    //    XmlRpc::XmlRpcValue bodyList;
    //  nh->get_parameter(rosparam::keys::RigidBodies, bodyList);
    auto bodyList = nh->list_parameters({"ridig_bodies"}, 10);

    if (bodyList.names.size() > 0) {
      // bodyList は たぶん
      // .first = 名前
      // .second = 設定
      // になってると思う
      // それを踏まえるとまず名前を抜き出さないといけない
      std::set<std::string> bodyIdList;
      for (auto name : bodyList.names) {
        bodyIdList.insert(name);
      }

      for (auto const& iter : bodyList) {
        std::string strBodyId = iter.first;
        XmlRpc::XmlRpcValue bodyParameters = iter.second;

        if (bodyParameters.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          // Load configuration for this rigid body from ROS
          PublisherConfiguration publisherConfig;
          std::sscanf(strBodyId.c_str(), "%d", &publisherConfig.rigidBodyId);

          bool readPoseTopicName = impl::check_and_get_param(
              bodyParameters, rosparam::keys::PoseTopicName,
              publisherConfig.poseTopicName);

          if (!readPoseTopicName) {
            RCLCPP_WARN(
                nh->get_logger(),
                "Failed to parse %s for body `%d`. Pose publishing disabled.",
                rosparam::keys::PoseTopicName, publisherConfig.rigidBodyId);
            publisherConfig.publishPose = false;
          } else {
            publisherConfig.publishPose = true;
          }

          bool readPose2dTopicName = impl::check_and_get_param(
              bodyParameters, rosparam::keys::Pose2dTopicName,
              publisherConfig.pose2dTopicName);

          if (!readPose2dTopicName) {
            ROS_WARN_STREAM(rclcpp::get_logger("server"),
                            "Failed to parse "
                                << rosparam::keys::Pose2dTopicName
                                << " for body `" << publisherConfig.rigidBodyId
                                << "`. Pose publishing disabled.");
            publisherConfig.publishPose2d = false;
          } else {
            publisherConfig.publishPose2d = true;
          }

          bool readChildFrameId = impl::check_and_get_param(
              bodyParameters, rosparam::keys::ChildFrameId,
              publisherConfig.childFrameId);

          bool readParentFrameId = impl::check_and_get_param(
              bodyParameters, rosparam::keys::ParentFrameId,
              publisherConfig.parentFrameId);

          if (!readChildFrameId || !readParentFrameId) {
            if (!readChildFrameId)
              ROS_WARN_STREAM(rclcpp::get_logger("server"),
                              "Failed to parse "
                                  << rosparam::keys::ChildFrameId
                                  << " for body `"
                                  << publisherConfig.rigidBodyId
                                  << "`. TF publishing disabled.");

            if (!readParentFrameId)
              ROS_WARN_STREAM(rclcpp::get_logger("server"),
                              "Failed to parse "
                                  << rosparam::keys::ParentFrameId
                                  << " for body `"
                                  << publisherConfig.rigidBodyId
                                  << "`. TF publishing disabled.");

            publisherConfig.publishTf = false;
          } else {
            publisherConfig.publishTf = true;
          }

          pubConfigs.push_back(publisherConfig);
        }
      }
    }
  }
}

}  // namespace mocap_optitrack