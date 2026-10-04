import math
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.action import ActionClient
from tb3_interfaces.action import NavigateToPose, ExecuteTask

class TaskPlanner(LifecycleNode):
    def __init__(self):
        # 开启 allow_undeclared_parameters，允许动态读取 YAML 里的各种航点坐标
        super().__init__('task_planner', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

    def on_activate(self, state):
        self.get_logger().info('任务规划器已激活！准备派发任务...')
        self.nav_client = ActionClient(self, NavigateToPose, 'custom_navigate_to_pose')
        self.task_client = ActionClient(self, ExecuteTask, 'execute_task')

        # 延迟 1 秒再启动序列，确保导航 Server 已经完全就绪
        self.timer = self.create_timer(1.0, self.start_sequence)
        return super().on_activate(state)

    def start_sequence(self):
        self.timer.cancel() # 定时器只触发一次

        # 读取 YAML 中的任务列表
        sequence_param = self.get_parameter('task_sequence').value
        if not sequence_param:
            self.get_logger().error("任务序列为空！请检查 tasks.yaml 中的节点名称配置。")
            return

        self.sequence = sequence_param
        self.current_task_index = 0
        self.get_logger().info(f"已加载 {len(self.sequence)} 个任务。开始执行序列...")
        self.execute_next_task()

    def execute_next_task(self):
        # 检查是否所有任务都已完成
        if self.current_task_index >= len(self.sequence):
            self.get_logger().info("\n==================================\n所有任务均已圆满完成！\n==================================")
            return

        task_name = self.sequence[self.current_task_index]
        self.get_logger().info(f'\n---> 正在执行任务 {self.current_task_index + 1}/{len(self.sequence)}: [{task_name}]')

        if "go_to" in task_name:
            self.send_nav_goal(task_name)
        elif "detect" in task_name:
            self.send_vision_goal(task_name)
        else:
            self.get_logger().warn(f"未知的任务类型: {task_name}，跳过该任务。")
            self.task_completed(True)

    def send_nav_goal(self, task_name):
        # 从参数服务器读取坐标 [x, y, yaw]
        coords = self.get_parameter(f'waypoints.{task_name}').value
        if coords is None:
            self.get_logger().error(f"缺少任务 {task_name} 的坐标点配置！")
            self.task_completed(False)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.pose.position.x = float(coords[0])
        goal_msg.target_pose.pose.position.y = float(coords[1])
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.pose.orientation.z = math.sin(float(coords[2]) / 2.0)
        goal_msg.target_pose.pose.orientation.w = math.cos(float(coords[2]) / 2.0)

        self.get_logger().info("正在等待导航动作服务器(Navigation Server)就绪...")
        self.nav_client.wait_for_server()
        self.get_logger().info(f"已发送目标坐标: (x={coords[0]}, y={coords[1]})")

        # 异步发送目标
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被服务器拒绝。')
            self.task_completed(False)
            return
        self.get_logger().info('接收目标成功，机器人正在前往目的地...')

        # 异步等待结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('已成功到达目的地！')
            self.task_completed(True)
        else:
            self.get_logger().error('导航失败。')
            self.task_completed(False)

    def send_vision_goal(self, task_name):
        self.get_logger().info(f"正在向视觉系统下发识别任务: {task_name}...")

        goal_msg = ExecuteTask.Goal()
        goal_msg.task_name = task_name

        # 确保动作服务器就绪
        self.task_client.wait_for_server()

        # 异步发送请求
        send_goal_future = self.task_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.vision_goal_response_callback)

    def vision_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('视觉任务被服务器拒绝！')
            self.task_completed(False)
            return

        self.get_logger().info('视觉系统已开始执行扫描...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.vision_result_callback)

    def vision_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('视觉系统成功检测到目标物体！')
            self.task_completed(True)
        else:
            self.get_logger().error(f'视觉检测失败: {result.message}')
            self.task_completed(False)

    def task_completed(self, success):
        if success:
            self.current_task_index += 1
            self.execute_next_task()
        else:
            self.get_logger().error("由于发生错误，任务序列已终止。")

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
