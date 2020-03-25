#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>

sensor_msgs::JointState dualarm_jointState_msg;

void JointStateCB(const sensor_msgs::JointState& message_holder)
{
    dualarm_jointState_msg.header.stamp = ros::Time::now();
    dualarm_jointState_msg.name[0] = message_holder.name[0];
    dualarm_jointState_msg.position.push_back(message_holder.position[0]);
    dualarm_jointState_msg.velocity[0] = message_holder.velocity[0];
    dualarm_jointState_msg.effort[0] = message_holder.effort[0];
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_state_combine_node");
    ros::NodeHandle nh;

    ros::Subscriber left_arm_joint_state_subscriber = nh.subscribe("joint_states", 1, JointStateCB);
    // ros::Subscriber right_arm_joint_state_subscriber = nh.subscribe("right_arm/joint_states", 1, JointStateCB);

    ros::Publisher dualarm_joint_state_publisher = nh.advertise<sensor_msgs::JointState>("dualarm_joint_states", 1);
    while(ros::ok())
    {
        dualarm_joint_state_publisher.publish(dualarm_jointState_msg);
    }
    ros::spin();
    return 0;
}