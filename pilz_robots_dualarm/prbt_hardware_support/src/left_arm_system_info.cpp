/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <prbt_hardware_support/left_arm_system_info.h>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <sensor_msgs/JointState.h>

#include <pilz_utils/wait_for_service.h>
#include <pilz_utils/wait_for_message.h>
#include <canopen_chain_node/GetObject.h>
#include <prbt_hardware_support/system_info_exception.h>

namespace prbt_hardware_support
{

static const std::string LEFT_ARM_CANOPEN_GETOBJECT_SERVICE_NAME{"/left_arm/driver/get_object"};
static const std::string LEFT_ARM_CANOPEN_NODES_PARAMETER_NAME{"/left_arm/driver/nodes"};
static const std::string LEFT_ARM_JOINT_STATE_TOPIC {"left_arm/joint_states"};

static const std::string LEFT_ARM_GET_FIRMWARE_VERION_OBJECT{"100A"};

// Currently the string is defined to be 41 characters long, but the last character can be omitted.
// This is currently under investigation. See https://github.com/PilzDE/pilz_robots/issues/299.
static constexpr std::size_t FIRMWARE_STRING_LENGTH{40};

LeftArmSystemInfo::LeftArmSystemInfo(ros::NodeHandle &nh)
    : joint_names_( getNodeNames(nh) )
{
  // Wait till CAN is up and running.
  // Reason: If the first CAN service call happens before
  // the CAN is fully initialized, the CAN will not start properly.
  pilz_utils::waitForMessage<sensor_msgs::JointState>(LEFT_ARM_JOINT_STATE_TOPIC);

  pilz_utils::waitForService(LEFT_ARM_CANOPEN_GETOBJECT_SERVICE_NAME);
  canopen_srv_get_client_ = nh.serviceClient<canopen_chain_node::GetObject>(LEFT_ARM_CANOPEN_GETOBJECT_SERVICE_NAME);
}

std::string LeftArmSystemInfo::getFirmwareVersionOfJoint(const std::string& joint_name)
{
  canopen_chain_node::GetObject srv;
  srv.request.node = joint_name;
  srv.request.object = LEFT_ARM_GET_FIRMWARE_VERION_OBJECT;
  srv.request.cached = false;

  ROS_INFO_STREAM("Call \"get firmware\" service for \"" << joint_name << "\"");
  if (!canopen_srv_get_client_.call(srv))
  {
    throw SystemInfoException("CANopen service to request firmware version failed");
  }

  if (!srv.response.success)
  {
    throw SystemInfoException(srv.response.message);
  }

  srv.response.value.resize(FIRMWARE_STRING_LENGTH);

  return srv.response.value;
}

FirmwareCont LeftArmSystemInfo::getFirmwareVersions()
{
  FirmwareCont versions;
  for(const auto& joint : joint_names_)
  {
    versions[joint] = getFirmwareVersionOfJoint(joint);
  }
  return versions;
}

std::vector<std::string> LeftArmSystemInfo::getNodeNames(const ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue rpc;
  if (!nh.getParam(LEFT_ARM_CANOPEN_NODES_PARAMETER_NAME, rpc))
  {
    throw SystemInfoException("Could not read node names");
  }

  std::vector<std::string> node_names;
  for (auto& rpci : rpc)
  {
    auto node_name = rpci.first.c_str();
    node_names.push_back(node_name);
  }
  return node_names;
}

} // namespace prbt_hardware_support
