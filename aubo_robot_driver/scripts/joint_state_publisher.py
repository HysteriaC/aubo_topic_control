#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import socket
import math
import json

def extract_complete_packet(data_buffer):
    """
    从缓冲区中提取完整的数据包
    :param data_buffer: 当前缓冲区字符串
    :return: (完整数据包，剩余缓冲区)
    """
    PACK_BEGIN = "<PACK_BEGIN"
    PACK_END = "PACK_END>"

    start_pos = data_buffer.find(PACK_BEGIN)
    end_pos = data_buffer.find(PACK_END)

    if start_pos == -1 or end_pos == -1:
        return None, data_buffer

    if start_pos >= end_pos:
        return None, data_buffer

    start_pos += len(PACK_BEGIN)
    length_pos = start_pos
    start_pos += 8

    length_str = data_buffer[length_pos:start_pos]
    try:
        data_length = int(length_str)
    except ValueError:
        return None, data_buffer

    if end_pos - start_pos != data_length:
        return None, data_buffer

    json_str = data_buffer[start_pos:start_pos + data_length]
    complete_packet = json_str
    remaining_buffer = data_buffer[end_pos + len(PACK_END):]

    return complete_packet, remaining_buffer


def parse_joint_data(complete_packet):
    """
    解析 JSON 数据，提取关节状态
    :param complete_packet: 完整的 JSON 字符串
    :return: 关节位置列表（单位：度）
    """
    try:
        root = json.loads(complete_packet)
        joint_positions = []

        if "RobotJointStatus" in root and "jointPos" in root["RobotJointStatus"]:
            joint_pos = root["RobotJointStatus"]["jointPos"]
            if isinstance(joint_pos, list):
                joint_positions = [pos for pos in joint_pos]
        else:
            rospy.logwarn("Failed to find jointPos in JSON data.")

        return joint_positions
    except json.JSONDecodeError as e:
        rospy.logerr(f"Failed to parse JSON data. Error: {e}")
        return None


def tcp_joint_state_publisher(ip="127.0.0.1", port=8891):
    """
    TCP 客户端，用于接收关节状态数据并发布到 /joint_states
    :param ip: 服务端 IP 地址
    :param port: 服务端端口
    """
    rospy.init_node("joint_state_publisher")
    joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    # 定义关节名称（与 URDF 文件中的名称保持一致）
    joint_names = [
        "shoulder_joint", "upperArm_joint", "foreArm_joint",
        "wrist1_joint", "wrist2_joint", "wrist3_joint"
    ]

    # 创建 TCP 客户端
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client_socket.connect((ip, port))
        rospy.loginfo(f"Connected to server at {ip}:{port}")
    except ConnectionRefusedError:
        rospy.logerr(f"Failed to connect to {ip}:{port}. Please check if the server is running.")
        return

    data_buffer = ""

    rate = rospy.Rate(50)  # 设置发布频率为 50 Hz
    while not rospy.is_shutdown():
        try:
            # 接收数据
            data = client_socket.recv(1024).decode("utf-8", errors="replace")
            if not data:
                rospy.logwarn("Connection closed by server.")
                break

            # 将数据添加到缓冲区
            data_buffer += data

            # 提取完整的数据包
            complete_packet, data_buffer = extract_complete_packet(data_buffer)

            if complete_packet:
                # 解析关节状态
                joint_positions = parse_joint_data(complete_packet)

                if joint_positions is not None:
                    # 创建 JointState 消息
                    joint_state_msg = JointState()
                    joint_state_msg.header = Header(stamp=rospy.Time.now())
                    joint_state_msg.name = joint_names
                    joint_state_msg.position = [pos for pos in joint_positions]
                    joint_state_msg.velocity = []  # 留空（可选）
                    joint_state_msg.effort = []  # 留空（可选）

                    # 发布消息
                    joint_state_pub.publish(joint_state_msg)
                    rospy.loginfo(f"Published joint states: {joint_positions}")
                else:
                    rospy.logwarn("Failed to parse joint data.")
        except Exception as e:
            rospy.logerr(f"Error in TCP client: {e}")
            break

    # 关闭套接字
    client_socket.close()
    rospy.loginfo("Socket closed.")


if __name__ == "__main__":
    try:
        ip = rospy.get_param("/joint_state_publisher/ip", "127.0.0.1")  # 默认 IP
        rospy.loginfo(f"Received IP: {ip}")
        tcp_joint_state_publisher(ip=ip, port=8891)
    except rospy.ROSInterruptException:
        pass

