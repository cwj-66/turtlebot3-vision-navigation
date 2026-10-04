from setuptools import setup
import os
from glob import glob

package_name = 'tb3_custom_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 1. 注册 launch 文件夹，确保 ros2 launch 能找到 mission.launch.py
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        # 2. 注册 config 文件夹，确保能加载 tasks.yaml
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cwj-66',
    maintainer_email='258235651+cwj-66@users.noreply.github.com',
    description='Custom navigation and task execution for TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 3. 注册你的所有 Python 节点
            # 格式：'生成的可执行文件名 = 包名.文件名(不带.py):入口函数名'
            'vision_node = tb3_custom_nav.vision_node:main',
            'nav_action_server = tb3_custom_nav.nav_action_server:main',
            'task_planner_node = tb3_custom_nav.task_planner_node:main',
        ],
    },
)
