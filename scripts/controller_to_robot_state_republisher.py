#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState

armState = JointState()
armState.name = 11*[""]
armState.position = 11*[0]

armState.name[0] ="first_to_base"
armState.position[0] = 0.0
armState.name[1] ="second_to_first"
armState.position[1] = 0.0
armState.name[2] ="third_to_second"
armState.position[2] = 0.0
armState.name[3] ="fourth_to_third"
armState.position[3] = 0.0
armState.name[4] ="fifth_to_fourth"
armState.position[4] = 0.0
armState.name[5] ="right_gripper_to_fifth_link"
armState.position[5] = 0.0
armState.name[6] ="left_gripper_to_fifth_link"
armState.position[6] = 0.0
armState.name[7] ="baseToWheelFrontRight"
armState.position[7] = 0.0
armState.name[8] ="baseToWheelFrontLeft"
armState.position[8] = 0.0
armState.name[9] ="baseToWheelRearRight"
armState.position[9] = 0.0
armState.name[10] ="baseToWheelRearLeft"
armState.position[10] = 0.0


def armStateUpdater(msg, nodeID):
    global armState
    armState.position[nodeID] = msg.process_value

if __name__ == "__main__":
    rospy.init_node("controller_to_robot_state_republisher")
    joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=1000)
    rate = rospy.Rate(100) # 10hz
    controllerSubscribers = []
    for i in range(0, 7):
        controllerSubscribers.append(rospy.Subscriber('/chrzaszcz/joint'+str(i+1)+'_position_controller/state', JointControllerState, armStateUpdater, (i)))

    count = 0

    # rospy.spin()
    while not rospy.is_shutdown():
        # rospy.loginfo(hello_str)
        armState.header.stamp = rospy.Time.now()
        joint_state_publisher.publish(armState)
        rate.sleep()
        # count += 1
