import sys
import queue
import rclpy
import threading
import subprocess
from functools import partial
from time import sleep
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SetInitialPose
from rclpy.action.client import GoalStatus

class RobotMoveNode(Node):
    def __init__(self):
        super().__init__('robot_move_node')

        self.QOS_RKL10T = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        self.goal_poses = {
            "Table_1": [1.4, -0.58],
            "Table_2": [2.4, -0.58],
            "Table_3": [3.6, -0.58],
            "Table_4": [1.4, 0.48],
            "Table_5": [2.4, 0.48],
            "Table_6": [3.6, 0.48],
            "Table_7": [1.4, 1.6],
            "Table_8": [2.4, 1.6],
            "Table_9": [3.6, 1.6],
            "Starting_Point": [0.0, 0.0],
        }

        self.queue = queue.Queue()
        self.processing_queue = False
        self.returning_to_start = False
        self.timer = self.create_timer(0.1, self.process_queue)

        self.delivery_request_subscriber = self.create_subscription(
            String,
            'delivery_request_topic',
            self.delivery_request_callback,
            10
        )

        self.set_initial_pose_service_client = self.create_client(SetInitialPose, '/set_initial_pose')
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.status_publisher_ = self.create_publisher(String, 'robot_status_topic', 10)
        self.publish_status("서빙로봇 대기중")

        while not self.set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_initial_pose not available, waiting again...')

        self.set_initial_pose(x=0.0, y=0.0, z=0.0, w=1.0)

    def process_queue(self):
        if not self.processing_queue and not self.queue.empty():
            self.processing_queue = True
            cmd, val = self.queue.get()
            if cmd == 'navigate_to_pose_send_goal':
                self.navigate_to_pose_send_goal(val)

    def delivery_request_callback(self, msg):
        table_id = msg.data
        self.get_logger().info(f"Received delivery request for table: {table_id}")
        if table_id in self.goal_poses:
            self.get_logger().info(f"Adding {table_id} to queue for navigation.")
            self.queue.put(['navigate_to_pose_send_goal', table_id])
        else:
            self.get_logger().warning(f"Table ID {table_id} not found in goal_poses")

    def set_initial_pose(self, x, y, z, w):
        req = SetInitialPose.Request()
        req.pose.header.frame_id = 'map'
        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)
        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)

        try:
            future = self.set_initial_pose_service_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f"Initial pose set to x={x}, y={y}, z={z}, w={w}.")
        except Exception as e:
            self.get_logger().error(f"Failed to set initial pose: {e}")

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_publisher_.publish(msg)

    def navigate_to_pose_send_goal(self, table_id):
        self.get_logger().info(f"Sending navigation goal for table: {table_id}")
        if table_id == 'Starting_Point' : self.publish_status('로봇 복귀중')
        pose = self.goal_poses[table_id]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.send_goal_future = self.navigate_to_pose_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback
        )
        self.send_goal_future.add_done_callback(partial(self.navigate_to_pose_action_goal, table_id))

    def navigate_to_pose_action_goal(self, table_id, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            self.processing_queue = False
            return

        self.get_logger().info("Goal accepted.")
        self.publish_status("로봇 서빙중")

        self.action_result_future = goal_handle.get_result_async()
        self.action_result_future.add_done_callback(partial(self.navigate_to_pose_action_result, table_id))

    def navigate_to_pose_action_feedback(self, feedback_msg):
        pass

    def navigate_to_pose_action_result(self, table_id, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Action succeeded for {table_id}.")

            if table_id != "Starting_Point":
                self.get_logger().info("Returning to starting point after delivery.")
                self.publish_status("로봇 서빙완료")
                self.returning_to_start = True
                self.navigate_to_pose_send_goal("Starting_Point")
            else:
                self.returning_to_start = False
                self.publish_status("서빙로봇 대기중")

                self.get_logger().info("Waiting 10 seconds at starting point before processing next command.")
                sleep(5)

                if not self.queue.empty():
                    self.get_logger().info("Processing next command in the queue.")
                    self.processing_queue = True
                    cmd, val = self.queue.get()
                    self.navigate_to_pose_send_goal(val)
                else:
                    self.get_logger().info("Queue is empty. Waiting for new commands.")
        else:
            self.get_logger().info(f"Action failed with status: {result.status}")

        if not self.returning_to_start:
            self.processing_queue = False


def main(args=None):
    rclpy.init(args=args)
    node = RobotMoveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()