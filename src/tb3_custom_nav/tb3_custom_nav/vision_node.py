import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from tb3_interfaces.msg import ObjectPosition
from tb3_interfaces.action import ExecuteTask
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class VisionNode(LifecycleNode):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.sub = None
        self.pub = None
        self.cmd_vel_pub = None
        self.image_pub = None
        self._action_server = None

        self.action_cb_group = MutuallyExclusiveCallbackGroup()
        self.vision_cb_group = MutuallyExclusiveCallbackGroup()

        self.color_ranges = {
            'red': (np.array([0, 120, 70]), np.array([10, 255, 255])),
            'blue': (np.array([100, 150, 0]), np.array([140, 255, 255])),
            'green': (np.array([35, 50, 50]), np.array([85, 255, 255])),
            'yellow': (np.array([20, 100, 100]), np.array([30, 255, 255]))
        }

        self.current_target_color = None
        self.target_found = False

    def on_configure(self, state: LifecycleState):
        self.pub = self.create_lifecycle_publisher(ObjectPosition, '/detected_object', 10)
        self.cmd_vel_pub = self.create_lifecycle_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_lifecycle_publisher(Image, '/camera/image_annotated', 10)

        self._action_server = ActionServer(
            self, ExecuteTask, 'execute_task', self.execute_callback,
            callback_group=self.action_cb_group
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10,
            callback_group=self.vision_cb_group
        )
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        self.destroy_subscription(self.sub)
        return super().on_deactivate(state)

    def execute_callback(self, goal_handle):
        task_name = goal_handle.request.task_name
        self.get_logger().info(f'收到视觉识别任务: {task_name}')

        if "red" in task_name:
            self.current_target_color = "red"
        elif "blue" in task_name:
            self.current_target_color = "blue"
        elif "yellow" in task_name:
            self.current_target_color = "yellow"
        elif "green" in task_name:
            self.current_target_color = "green"
        else:
            self.get_logger().warn("未知的颜色类型！")
            goal_handle.abort()
            return ExecuteTask.Result(success=False, message="Unknown color")

        self.target_found = False
        self.get_logger().info(f'开始原地旋转并寻找颜色: {self.current_target_color} ...')

        search_twist = Twist()
        search_twist.angular.z = 0.3
        stop_twist = Twist()

        timeout = 30.0
        start_time = time.time()

        while not self.target_found:
            if time.time() - start_time > timeout:
                self.get_logger().error('寻找超时，未发现目标！')
                self.cmd_vel_pub.publish(stop_twist)
                self.current_target_color = None
                goal_handle.abort()
                return ExecuteTask.Result(success=False, message="Timeout")

            self.cmd_vel_pub.publish(search_twist)
            time.sleep(0.1)

        # 找到目标后停止底盘，并短暂保留带框图像。
        self.cmd_vel_pub.publish(stop_twist)
        self.get_logger().info('目标已锁定，停止旋转。保持画面展示 3 秒钟...')

        # 独立图像回调组在等待期间继续更新画面。
        time.sleep(3.0)

        self.current_target_color = None
        goal_handle.succeed()
        return ExecuteTask.Result(success=True, message="Target Found")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 在任务完成前持续发布当前颜色的检测画面。
        if self.current_target_color:
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower, upper = self.color_ranges[self.current_target_color]
            mask = cv2.inRange(hsv_image, lower, upper)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 500:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(cv_image, f'Found {self.current_target_color}!', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    self.target_found = True # 通知动作服务器找到了

        annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_pub.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
