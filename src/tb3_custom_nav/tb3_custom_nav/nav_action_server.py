import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose as Nav2NavigateToPose
from tb3_interfaces.action import NavigateToPose

class CustomNavServer(LifecycleNode):
    def __init__(self):
        super().__init__('custom_nav_server')
        self._action_server = None
        self._nav2_client = None
        self._callback_group = ReentrantCallbackGroup()

    def on_configure(self, state):
        self.get_logger().info('Configuring Nav Server...')
        self._action_server = ActionServer(self, NavigateToPose, 'custom_navigate_to_pose', self.execute_callback, callback_group=self._callback_group)
        self._nav2_client = ActionClient(self, Nav2NavigateToPose, 'navigate_to_pose', callback_group=self._callback_group)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating Nav Server...')
        return TransitionCallbackReturn.SUCCESS

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received nav goal...')
        # 调用 Nav2 的 action
        nav2_goal = Nav2NavigateToPose.Goal()
        nav2_goal.pose = goal_handle.request.target_pose

        result_msg = NavigateToPose.Result()
        if not self._nav2_client.wait_for_server(timeout_sec=10.0):
            goal_handle.abort()
            result_msg.success = False
            result_msg.message = 'Nav2 action server unavailable'
            return result_msg
        nav2_handle = await self._nav2_client.send_goal_async(nav2_goal)
        if not nav2_handle.accepted:
            goal_handle.abort()
            result_msg.success = False
            result_msg.message = 'Nav2 rejected the goal'
            return result_msg
        result = await nav2_handle.get_result_async()
        result_msg.success = result.status == GoalStatus.STATUS_SUCCEEDED
        result_msg.message = 'Arrived' if result_msg.success else 'Nav2 navigation failed'
        if result_msg.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result_msg

def main(args=None):
    rclpy.init(args=args)
    node = CustomNavServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
