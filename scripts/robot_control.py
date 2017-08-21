#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from keyboard.msg import Key
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math

cmd_vel = Twist()
jointController = [0.0, math.radians(-70.0), math.radians(70.0), math.radians(70.0), 0.0, 0.035, -0.035]

class keyboard:
    def __init__(self):
        self.KEY_A = 97
        self.KEY_B = 98
        self.KEY_C = 99
        self.KEY_D = 100
        self.KEY_E = 101
        self.KEY_F = 102
        self.KEY_G = 103
        self.KEY_H = 104
        self.KEY_I = 105
        self.KEY_J = 106
        self.KEY_K = 107
        self.KEY_L = 108
        self.KEY_M = 109
        self.KEY_N = 110
        self.KEY_O = 111
        self.KEY_P = 112
        self.KEY_Q = 113
        self.KEY_R = 114
        self.KEY_S = 115
        self.KEY_T = 116
        self.KEY_U = 117
        self.KEY_V = 118
        self.KEY_W = 119
        self.KEY_X = 120
        self.KEY_Y = 121
        self.KEY_Z = 122
        self.KEY_SPACE = 32

keyboard_codes = keyboard()

def keyboardCallback(msg):
    global cmd_vel, jointController
    # print "callbacked!"
    # cmd_vel.header.stamp = rospy.Time.now()
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    if msg.code == keyboard_codes.KEY_W:
        cmd_vel.linear.x = 0.5
    elif msg.code == keyboard_codes.KEY_S:
        cmd_vel.linear.x = -0.5
    elif msg.code == keyboard_codes.KEY_D:
        cmd_vel.angular.z = -5.0
    elif msg.code == keyboard_codes.KEY_A:
        cmd_vel.angular.z = 5.0
    elif msg.code == keyboard_codes.KEY_E:
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = -5.0
    elif msg.code == keyboard_codes.KEY_Q:
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = 5.0
    elif msg.code == keyboard_codes.KEY_Z:
        cmd_vel.linear.x = -0.5
        cmd_vel.angular.z = -5.0
    elif msg.code == keyboard_codes.KEY_C:
        cmd_vel.linear.x = -0.5
        cmd_vel.angular.z = 5.0

    elif msg.code == keyboard_codes.KEY_R:
        if jointController[0] < math.radians(70.0):
            jointController[0] += 0.02
    elif msg.code == keyboard_codes.KEY_F:
        if jointController[0] > math.radians(-70.0):
            jointController[0] -= 0.02

    elif msg.code == keyboard_codes.KEY_T:
        if jointController[1] < math.radians(70.0):
            jointController[1] += 0.02
    elif msg.code == keyboard_codes.KEY_G:
        if jointController[1] > math.radians(-70.0):
            jointController[1] -= 0.02

    elif msg.code == keyboard_codes.KEY_Y:
        if jointController[2] < math.radians(70.0):
            jointController[2] += 0.02
    elif msg.code == keyboard_codes.KEY_H:
        if jointController[2] > math.radians(-70.0):
            jointController[2] -= 0.02

    elif msg.code == keyboard_codes.KEY_U:
        if jointController[3] < math.radians(70.0):
            jointController[3] += 0.02
    elif msg.code == keyboard_codes.KEY_J:
        if jointController[3] > math.radians(-70.0):
            jointController[3] -= 0.02

    elif msg.code == keyboard_codes.KEY_I:
        if jointController[4] < 0.0:
            jointController[4] += 0.02
    elif msg.code == keyboard_codes.KEY_K:
        if jointController[4] > -math.pi:
            jointController[4] -= 0.02

    elif msg.code == keyboard_codes.KEY_O:
        if jointController[5] < 0.035:
            jointController[5] += 0.001
            jointController[6] -= 0.001
    elif msg.code == keyboard_codes.KEY_L:
        if jointController[5] > 0:
            jointController[5] -= 0.001
            jointController[6] += 0.001

    elif msg.code == keyboard_codes.KEY_SPACE:
        jointController = [0.0, math.radians(-70.0), math.radians(70.0), math.radians(70.0), 0.0, 0.0, 0.0]


rospy.init_node("robot_control")

rospy.Subscriber('/keyboard_reader/keydown', Key, keyboardCallback)
cmd_vel_publisher = rospy.Publisher('/chrzaszcz/cmd_vel', Twist, queue_size=1)
requested_pose_publisher = rospy.Publisher('/requested_pose', JointState, queue_size=1)

controllerPublishers = []
for i in range(0, 7):
    controllerPublishers.append(rospy.Publisher('/chrzaszcz/joint'+str(i+1)+'_position_controller/command', Float64, queue_size = 1))

rate = rospy.Rate(10)
requested_pose = JointState()
requested_pose.name = 7*[""]
requested_pose.position = 7*[0.0]
requested_pose.name[0] ="first_to_base"
requested_pose.name[1] ="second_to_first"
requested_pose.name[2] ="third_to_second"
requested_pose.name[3] ="fourth_to_third"
requested_pose.name[4] ="fifth_to_fourth"
requested_pose.name[5] ="right_gripper_to_fifth_link"
requested_pose.name[6] ="left_gripper_to_fifth_link"
mapping = [1, -1, 1, -1, 1, 1, 1]

while not rospy.is_shutdown():
    requested_pose.header.stamp = rospy.Time.now()
    for i in range(0,7):
        requested_pose.position[i] = mapping[i]*jointController[i]

    requested_pose_publisher.publish(requested_pose)
    cmd_vel_publisher.publish(cmd_vel)
    for i in range(0, 7):
        controllerPublishers[i].publish(jointController[i])

    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    rate.sleep()
