"""Configure and activate mission nodes after their lifecycle transitions."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tb3_custom_nav'), 'config', 'tasks.yaml')
    common_params = {
        'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool),
    }

    def node(executable, name, parameters):
        return LifecycleNode(
            package='tb3_custom_nav', executable=executable,
            name=name, namespace='', output='screen', parameters=parameters)

    vision = node('vision_node', 'vision_node', [common_params])
    navigation = node('nav_action_server', 'custom_nav_server', [common_params])
    planner = node('task_planner_node', 'task_planner', [config, common_params])

    def change_state(target, transition):
        return EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(target),
            transition_id=transition))

    handlers = []
    for target in (vision, navigation, planner):
        handlers.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=target, goal_state='inactive',
            entities=[change_state(target, Transition.TRANSITION_ACTIVATE)])))

    # Configure each downstream node once the preceding node is active.
    for previous, following in ((vision, navigation), (navigation, planner)):
        handlers.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=previous, goal_state='active',
            entities=[change_state(following, Transition.TRANSITION_CONFIGURE)])))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *handlers, vision, navigation, planner,
        change_state(vision, Transition.TRANSITION_CONFIGURE),
    ])
