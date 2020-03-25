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

#include <ros/ros.h>

#include <pilz_utils/wait_for_service.h>

#include <prbt_hardware_support/left_arm_stop1_executor.h>
#include <prbt_hardware_support/service_function_decl.h>

const std::string LEFT_ARM_HOLD_SERVICE{"left_arm_joint_trajectory_controller/hold"};
const std::string LEFT_ARM_UNHOLD_SERVICE{"left_arm_joint_trajectory_controller/unhold"};
const std::string LEFT_ARM_RECOVER_SERVICE{"driver/recover"};
const std::string LEFT_ARM_HALT_SERVICE{"driver/halt"};

// LCOV_EXCL_START
namespace prbt_hardware_support
{

bool callService(ros::ServiceClient& srv_client)
{
  std_srvs::Trigger trigger;
  ROS_DEBUG_STREAM("Calling service: " << srv_client.getService() << ")");
  bool call_success = srv_client.call(trigger);
  if (!call_success)
  {
    ROS_ERROR_STREAM("No success calling service: " << srv_client.getService());
  }

  if (!trigger.response.success)
  {
    ROS_ERROR_STREAM("Service: " << srv_client.getService()
                     << " failed with error message:\n"
                     << trigger.response.message);
  }
  return call_success && trigger.response.success;
}

} // namespace prbt_hardware_support

using namespace prbt_hardware_support;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "left_arm_stop1_executor");
  ros::NodeHandle nh;

  pilz_utils::waitForService(LEFT_ARM_HOLD_SERVICE);
  ros::ServiceClient left_arm_hold_srv = nh.serviceClient<std_srvs::Trigger>(LEFT_ARM_HOLD_SERVICE);

  pilz_utils::waitForService(LEFT_ARM_UNHOLD_SERVICE);
  ros::ServiceClient left_arm_unhold_srv = nh.serviceClient<std_srvs::Trigger>(LEFT_ARM_UNHOLD_SERVICE);

  pilz_utils::waitForService(LEFT_ARM_RECOVER_SERVICE);
  ros::ServiceClient left_arm_recover_srv = nh.serviceClient<std_srvs::Trigger>(LEFT_ARM_RECOVER_SERVICE);

  pilz_utils::waitForService(LEFT_ARM_HALT_SERVICE);
  ros::ServiceClient left_arm_halt_srv = nh.serviceClient<std_srvs::Trigger>(LEFT_ARM_HALT_SERVICE);

  TServiceCallFunc left_arm_hold_func = std::bind(callService, left_arm_hold_srv);
  TServiceCallFunc left_arm_unhold_func = std::bind(callService, left_arm_unhold_srv);
  TServiceCallFunc left_arm_recover_func = std::bind(callService, left_arm_recover_srv);
  TServiceCallFunc left_arm_halt_func = std::bind(callService, left_arm_halt_srv);

  LeftArm_Stop1Executor left_arm_stop1_executor(left_arm_hold_func, left_arm_unhold_func, left_arm_recover_func, left_arm_halt_func);
  ros::ServiceServer left_arm_sto_serv = nh.advertiseService("safe_torque_off",
                                                    &LeftArm_Stop1Executor::updateStoCallback,
                                                    &left_arm_stop1_executor);

  ros::spin();

  return EXIT_FAILURE;
}
// LCOV_EXCL_STOP
