#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

dualarm_joint_state = JointState()
left_arm_joint_state = JointState()
right_arm_joint_state = JointState()


def leftArmJointStateCB(message_holder):
    left_arm_joint_state.name = message_holder.name
    left_arm_joint_state.position = message_holder.position
    left_arm_joint_state.velocity = message_holder.velocity
    left_arm_joint_state.effort = message_holder.effort


def RightArmJointStateCB(message_holder):
    right_arm_joint_state.name = message_holder.name
    right_arm_joint_state.position = message_holder.position
    right_arm_joint_state.velocity = message_holder.velocity
    right_arm_joint_state.effort = message_holder.effort


if __name__ == '__main__':
    rospy.init_node("joint_state_combine_node")
    rospy.Subscriber('left_arm/joint_states', JointState, leftArmJointStateCB)
    rospy.Subscriber('right_arm/joint_states', JointState, RightArmJointStateCB)
    pub = rospy.Publisher("joint_states", JointState, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        dualarm_joint_state.header.stamp = rospy.Time.now()
        dualarm_joint_state.name = left_arm_joint_state.name\
                                 + right_arm_joint_state.name
        dualarm_joint_state.position = tuple(left_arm_joint_state.position)\
                                     + tuple(right_arm_joint_state.position)
        dualarm_joint_state.velocity = tuple(left_arm_joint_state.velocity)\
                                     + tuple(right_arm_joint_state.velocity)
        dualarm_joint_state.effort = tuple(left_arm_joint_state.effort)\
                                     + tuple(right_arm_joint_state.effort)
        pub.publish(dualarm_joint_state)
        rate.sleep()
    rospy.spin()
