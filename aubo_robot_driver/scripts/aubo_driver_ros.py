# -*- coding: UTF-8 -*-
# AUBO Robot Driver with ROS Noetic support
import sys
import os
import math
import socket
import multiprocessing
import threading
import time
import json
from pathlib import Path

# 引入 ROS 库
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# 导入 AUBO SDK
try:
    import libpyauboi5
except ImportError as e:
    raise ImportError("Failed to import AUBO SDK (libpyauboi5.so)")

# Constants for the AUBO robot SDK
RS_SUCC = 0

# TCP Client Parameters
TCP_PORT = 8891
tcp_client = None
tcp_process = None
tcp_stop_event = None
global_rtde_mode = None
ROBOT_MOVING = None
data_buffer = None


def initialize_globals():
    global tcp_stop_event, global_rtde_mode, ROBOT_MOVING, data_buffer
    tcp_stop_event = multiprocessing.Event()
    global_rtde_mode = multiprocessing.Value('b', False)
    ROBOT_MOVING = multiprocessing.Value('b', False)
    manager = multiprocessing.Manager()
    data_buffer = manager.Value(str, "")


class ComRobot:
    """Robot class for programming AUBO robots"""

    def __init__(self):
        self.RSHD = None
        self.CONNECTED = False

    def connect(self, ip, port):
        try:
            if libpyauboi5.initialize() != RS_SUCC:
                rospy.logerr("Failed to initialize the custom robot SDK")
                return False

            self.RSHD = libpyauboi5.create_context()

            if libpyauboi5.login(self.RSHD, ip, port) == RS_SUCC:
                self.CONNECTED = True
                return True
            else:
                return False
        except Exception as e:
            rospy.logerr(f"Error during connect: {e}")
            return False

    def disconnect(self):
        try:
            self.CONNECTED = False
            if self.RSHD:
                libpyauboi5.destory_context(self.RSHD)
                libpyauboi5.uninitialize()
            return True
        except Exception as e:
            rospy.logerr(f"Error during disconnect: {e}")
            return False

    def get_joint_positions(self):
        """Get the current joint positions in degrees."""
        try:
            positions = libpyauboi5.get_current_waypoint(self.RSHD)
            if 'joint' in positions and all(isinstance(pos, (float, int)) for pos in positions['joint']):
                return [math.degrees(float(rad)) for rad in positions['joint']]
            else:
                raise ValueError("Received invalid joint positions from SDK")
        except Exception as e:
            rospy.logerr(f"Error during get_joint_positions: {e}")
            return None

    def set_joint_maxacc(self, maxacc):
        """Set the maximum acceleration for all joints."""
        try:
            result = libpyauboi5.set_joint_maxacc(self.RSHD, maxacc)
            if result == RS_SUCC:
                rospy.loginfo(f"Set joint max acceleration to {maxacc}")
            else:
                rospy.logwarn(f"Failed to set joint max acceleration to {maxacc}, SDK returned: {result}")
        except Exception as e:
            rospy.logerr(f"Error during set_joint_maxacc to {maxacc}: {e}")

    def set_joint_maxvelc(self, maxvelc):
        """Set the maximum velocity for all joints."""
        try:
            result = libpyauboi5.set_joint_maxvelc(self.RSHD, maxvelc)
            if result == RS_SUCC:
                rospy.loginfo(f"Set joint max velocity to {maxvelc}")
            else:
                rospy.logwarn(f"Failed to set joint max velocity to {maxvelc}, SDK returned: {result}")
        except Exception as e:
            rospy.logerr(f"Error during set_joint_maxvelc to {maxvelc}: {e}")

    def set_end_max_line_acc(self, maxacc):
        """Set the maximum linear acceleration for the end effector."""
        try:
            result = libpyauboi5.set_end_max_line_acc(self.RSHD, maxacc)
            if result == RS_SUCC:
                rospy.loginfo(f"Set end effector max linear acceleration to {maxacc}")
            else:
                rospy.logwarn(f"Failed to set end effector max linear acceleration to {maxacc}, SDK returned: {result}")
        except Exception as e:
            rospy.logerr(f"Error during set_end_max_line_acc to {maxacc}: {e}")

    def set_end_max_line_velc(self, maxvelc):
        """Set the maximum linear velocity for the end effector."""
        try:
            result = libpyauboi5.set_end_max_line_velc(self.RSHD, maxvelc)
            if result == RS_SUCC:
                rospy.loginfo(f"Set end effector max linear velocity to {maxvelc}")
            else:
                rospy.logwarn(f"Failed to set end effector max linear velocity to {maxvelc}, SDK returned: {result}")
        except Exception as e:
            rospy.logerr(f"Error during set_end_max_line_velc to {maxvelc}: {e}")

    def init_motion_profile(self):
        """Initialize the global motion profile."""
        try:
            result = libpyauboi5.init_global_move_profile(self.RSHD)
            if result == RS_SUCC:
                rospy.loginfo("Motion profile initialized successfully")
            else:
                rospy.logwarn(f"Failed to initialize motion profile, SDK returned: {result}")
        except Exception as e:
            rospy.logerr(f"Error during init_motion_profile: {e}")


# ROS 功能
state_pub = None
status_pub = None


def command_callback(msg):
    """回调函数，处理收到的ROS指令"""
    command = msg.data.strip()
    RunCommand(command)


def publish_joint_states(robot):
    """持续发布关节状态到 /joint_states 话题"""
    rate = rospy.Rate(10)  # 设置发布频率为 10 Hz
    joint_names = [
        "shoulder_joint", "upperArm_joint", "foreArm_joint",
        "wrist1_joint", "wrist2_joint", "wrist3_joint"
    ]

    while not rospy.is_shutdown():
        try:
            # 获取关节位置（弧度）
            joint_positions = robot.get_joint_positions()
            if joint_positions is not None:
                # 创建 JointState 消息
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = rospy.Time.now()
                joint_state_msg.name = joint_names
                joint_state_msg.position = joint_positions
                joint_state_msg.velocity = []  # 如果没有速度数据，可以留空
                joint_state_msg.effort = []  # 如果没有力矩数据，可以留空

                # 发布 JointState 消息
                state_pub.publish(joint_state_msg)
                rospy.loginfo(f"Published joint states: {joint_positions}")
            else:
                rospy.logwarn("Failed to get joint positions. Skipping this iteration.")
        except Exception as e:
            rospy.logerr(f"Error in publish_joint_states: {e}")

        rate.sleep()

def RunCommand(cmd_line):
    """命令处理逻辑，保留原有逻辑"""
    global ROBOT, ROBOT_MOVING, global_rtde_mode

    def line_2_values(line):
        values = []
        for word in line:
            try:
                number = float(word)
                values.append(number)
            except ValueError:
                pass
        return values

    try:
        cmd_words = cmd_line.split()
        if len(cmd_words) == 0:
            return

        cmd = cmd_words[0]
        cmd_values = line_2_values(cmd_words[1:])

        if cmd == "CONNECT":
            if len(cmd_words) >= 3:
                ip_address = cmd_words[1]
                port = int(cmd_words[2])
                status_pub.publish("working")
                if ROBOT.connect(ip_address, port):
                    rospy.loginfo("Robot connected successfully")
                    status_pub.publish("ready") 
                else:
                    rospy.logerr("Failed to connect to the robot")
                    status_pub.publish("error") 

        elif cmd == "DISCONNECT":
            status_pub.publish("working") 
            ROBOT.disconnect()
            rospy.loginfo("Robot disconnected")
            status_pub.publish("ready") 

        elif cmd == "MOVJ":
            status_pub.publish("working") 
            joint_positions_rad = [math.radians(deg) for deg in cmd_values[:6]]
            result = libpyauboi5.move_joint(ROBOT.RSHD, joint_positions_rad)
            if result == RS_SUCC:
                rospy.loginfo(f"Joint Moved to target joints (degrees): {cmd_values[:6]}")
                status_pub.publish("ready") 
            else:
                rospy.logerr("Failed to move robot")
                status_pub.publish("error") 

        elif cmd == "MOVL":
            status_pub.publish("working") 
            joint_positions_rad = [math.radians(deg) for deg in cmd_values[:6]]
            result = libpyauboi5.move_line(ROBOT.RSHD, joint_positions_rad)
            if result == RS_SUCC:
                rospy.loginfo(f"Line Moved to target joints (degrees): {cmd_values[:6]}")
                status_pub.publish("ready") 
            else:
                rospy.logerr("Failed to move robot")
                status_pub.publish("error") 

        elif cmd == "MOVC":
            status_pub.publish("working") 
            current_joints = ROBOT.get_joint_positions()
            mid_joints_rad = [math.radians(deg) for deg in cmd_values[:6]]
            end_joints_rad = [math.radians(deg) for deg in cmd_values[6:12]]
            if current_joints:
                libpyauboi5.remove_all_waypoint(ROBOT.RSHD)
                libpyauboi5.add_waypoint(ROBOT.RSHD, [math.radians(deg) for deg in current_joints])
                libpyauboi5.add_waypoint(ROBOT.RSHD, mid_joints_rad)
                libpyauboi5.add_waypoint(ROBOT.RSHD, end_joints_rad)
                libpyauboi5.set_circular_loop_times(ROBOT.RSHD, 0)
                result = libpyauboi5.move_track(ROBOT.RSHD, 2)
                if result == RS_SUCC:
                    rospy.loginfo(f"Moved in circular motion from start (degrees): {current_joints}, through mid (degrees): {cmd_values[:6]}, to end (degrees): {cmd_values[6:12]}")
                    status_pub.publish("ready") 
                else:
                    rospy.logwarn("Circular motion failed.")
                    status_pub.publish("error") 
            else:
                rospy.logerr("Failed to get current joint positions")
                status_pub.publish("error") 

        elif cmd == "SPEED":
            if len(cmd_values) == 4:
                status_pub.publish("working")

                # Initialize motion profile
                ROBOT.init_motion_profile()

                joint_speed = cmd_values[1] * (math.pi / 180)  # deg/s to rad/s
                joint_acceleration = cmd_values[3] * (math.pi / 180)  # deg/s2 to rad/s2
                linear_speed = cmd_values[0] / 1000  # mm/s to m/s
                linear_acceleration = cmd_values[2] / 1000  # m/s2

                if joint_speed >= 0:
                    joint_maxvelc = (joint_speed,) * 6
                    ROBOT.set_joint_maxvelc(joint_maxvelc)
                if joint_acceleration >= 0:
                    joint_maxacc = (joint_acceleration,) * 6
                    ROBOT.set_joint_maxacc(joint_maxacc)
                if linear_speed >= 0:
                    ROBOT.set_end_max_line_velc(linear_speed)
                if linear_acceleration >= 0:
                    ROBOT.set_end_max_line_acc(linear_acceleration)

                status_pub.publish("ready")
            else:
                rospy.logwarn(f"Invalid number of parameters for SPEED command: {len(cmd_values)}")
                status_pub.publish("error") 

        else:
            rospy.logwarn(f"Unknown command: {cmd_line}")
            status_pub.publish("error") 
    except Exception as e:
        rospy.logerr(f"Error processing command '{cmd_line}': {e}")
        status_pub.publish("error") 


def RunMain():
    """ROS 主函数"""
    global state_pub, ROBOT, status_pub
    initialize_globals()
    ROBOT = ComRobot()

    rospy.init_node('aubo_robot_driver', anonymous=True)
    state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
    rospy.Subscriber("/robot_commands", String, command_callback)

    # 创建一个线程持续发布关节状态
    # joint_state_thread = threading.Thread(target=publish_joint_states, args=(ROBOT,))
    # joint_state_thread.daemon = True
    # joint_state_thread.start()
    port = 8899
    ip = rospy.get_param("/joint_state_publisher/ip", "127.0.0.1")
    if ROBOT.connect(ip, port):
        rospy.loginfo("Robot connected successfully")
        status_pub.publish("ready") 
    else:
        rospy.logerr("Failed to connect to the robot")
        status_pub.publish("error") 


    rospy.loginfo("AUBO robots with ROS interface running")
    rospy.spin()


if __name__ == "__main__":
    try:
        RunMain()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down AUBO Robot Driver")
