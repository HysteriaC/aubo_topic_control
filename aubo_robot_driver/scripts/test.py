#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

# 全局变量用于存储机器人状态
robot_status = None

def status_callback(msg):
    global robot_status
    robot_status = msg.data
    rospy.loginfo(f"Received robot status: {robot_status}")

# 封装函数：订阅并等待状态为 "ready" 或 "error"
def wait_for_status(timeout=None):
    """
    等待机器人状态为 "ready" 或 "error"。
    :param timeout: 超时时间（秒），默认为 None，表示无限等待。
    :return: "ready", "error", 或 "timeout"
    """
    global robot_status

    # 初始化 ROS 节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node("status_waiter", anonymous=True)

    # 订阅 /robot_status 话题
    rospy.Subscriber("/robot_status", String, status_callback)

    rospy.loginfo("Waiting for robot status (ready or error)...")

    start_time = rospy.get_time() if timeout else None
    
    robot_status = None

    # 循环等待目标状态
    while not rospy.is_shutdown():
        if robot_status == "ready":
            rospy.loginfo("Robot is ready. Proceeding with next steps.")
            return "ready"
        elif robot_status == "error":
            rospy.logwarn("Robot encountered an error. Halting operation.")
            return "error"

        # 检查超时
        if timeout and (rospy.get_time() - start_time > timeout):
            rospy.logwarn("Timeout reached while waiting for robot status.")
            return "timeout"

        rospy.sleep(0.1)  # 每隔 0.1 秒检查一次状态

    # 如果 ROS 节点关闭，返回超时
    return "timeout"

def connect_robot():
    # 初始化 ROS 节点
    # rospy.init_node('command_publisher', anonymous=True, disable_signals=True)
    # pub = rospy.Publisher('/robot_commands', String, queue_size=10, latch=True)

    # 创建消息
    # msg = String()
    # msg.data = "MOVJ 40 40 40 40 40 40"

    # 发布消息
    # pub.publish(msg)
    # print("Message published:", msg.data)

    # CONNECT ROBOT
    robot = String()
    robot.data = "CONNECT 192.168.63.128 8899"
    pub.publish(robot)
    print("CONNECT TO ROBOT:", robot.data)
    result = wait_for_status(timeout=None)
    while result != "ready":
        rospy.logwarn(f"Exiting program due to robot error.{result}")
        rospy.sleep(0.1)
    rospy.loginfo("CONNECT ROBOT SUCCSS")
    
def publish_command():
    msg = String()
    msg.data = "SPEED 50.0 200.0 1500.000 150.000"
    pub.publish(msg)
    print("Message published:", msg.data)
    result = wait_for_status(timeout=None)
    while result != "ready":
        rospy.logwarn(f"Exiting program due to robot error.{result}")
        rospy.sleep(0.1)

    msg = String()
    msg.data = "MOVJ 0 -7 -75 21.6 -90 0"
    pub.publish(msg)
    print("Message published:", msg.data)
    result = wait_for_status(timeout=None)
    while result != "ready":
        rospy.logwarn(f"Exiting program due to robot error.{result}")
        rospy.sleep(0.1)

    msg = String()
    msg.data = "MOVL -39 -5 -73.6 21.3 -90 -40"
    pub.publish(msg)
    print("Message published:", msg.data)
    result = wait_for_status(timeout=None)
    while result != "ready":
        rospy.logwarn(f"Exiting program due to robot error.{result}")
        rospy.sleep(0.1)

    msg = String()
    msg.data = "MOVJ 0 -7 -75 21.6 -90 0"
    pub.publish(msg)
    print("Message published:", msg.data)
    result = wait_for_status(timeout=None)
    while result != "ready":
        rospy.logwarn(f"Exiting program due to robot error.{result}")
        rospy.sleep(0.1)

    msg = String()
    msg.data = "MOVC -12 15 -52 23 -90 -12 -39 -5 -73.6 21.3 -90 -40"
    pub.publish(msg)
    print("Message published:", msg.data)
    result = wait_for_status(timeout=None)
    while result != "ready":
        rospy.logwarn(f"Exiting program due to robot error.{result}")
        rospy.sleep(0.1)

    msg = String()
    msg.data = "MOVJ 0 0 0 0 0 0"
    pub.publish(msg)
    print("Message published:", msg.data)
    result = wait_for_status(timeout=None)
    while result != "ready":
        rospy.logwarn(f"Exiting program due to robot error.{result}")
        rospy.sleep(0.1)
    

if __name__ == "__main__":
    rospy.init_node('command_publisher', anonymous=True, disable_signals=True)
    pub = rospy.Publisher('/robot_commands', String, queue_size=10, latch=True)
    # connect_robot()
    i = 0
    while i < 3:
        publish_command()
        i += 1

