"""Gazebo 월드 하나에 Pinky 두 대. 실물과 같은 토픽·프레임 이름을 쓰되 도메인만 다르다(robot1=25, robot2=27; 실물은 15/17).

도메인을 실물과 다르게 두는 이유: LOCALHOST 격리만으로는 실물 로봇이 이 PC를 peer로 알고 먼저 찾아오는 경우와
같은 PC의 실물용 터미널(도메인 15/17)이 시뮬을 보는 경우를 막지 못한다. 도메인이 다르면 어떤 설정이든 만나지 않는다.
"""
import re
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            UnsetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

DESCRIPTION = Path(get_package_share_directory('pinky_description'))
GZ_SIM = Path(get_package_share_directory('pinky_gz_sim'))
ROS_GZ_SIM = Path(get_package_share_directory('ros_gz_sim'))
FLEET = Path(get_package_share_directory('pinky_fleet'))


def gazebo_urdf(robot):
    urdf = xacro.process_file(str(DESCRIPTION / 'urdf/robot.urdf.xacro'),
                              mappings={'is_sim': 'true', 'cam_tilt_deg': '0'}).toxml()
    # 제조사 xacro의 namespace 인자는 프레임 이름까지 바꾸고 센서를 떨어뜨리므로 쓰지 않는다.
    # 대신 Gazebo 토픽 태그만 /<robot>/ 아래로 옮겨 두 로봇이 cmd_vel·scan·tf를 공유하지 않게 한다.
    return re.sub(r'<(topic|odom_topic|tf_topic)>/?', rf'<\1>/{robot}/', urdf)


def robot(name):
    arg = lambda key: LaunchConfiguration(f'{name}_{key}')
    # scoped 그룹이라 ROS_DOMAIN_ID는 이 안에서만 유효하다. (TimerAction을 넣으면 도메인을 잃는다)
    return GroupAction([
        SetEnvironmentVariable('ROS_DOMAIN_ID', arg('domain')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(DESCRIPTION / 'launch/upload_robot.launch.py')),
            launch_arguments={'is_sim': 'true'}.items()),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', name, '-string', gazebo_urdf(name),
                        ['-x=', arg('x')], ['-y=', arg('y')], '-z=0.1', ['-Y=', arg('yaw')]]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', namespace=name, output='screen',
             parameters=[{'config_file': str(FLEET / 'params/sim_bridge.yaml'),
                          'expand_gz_topic_names': True}]),
    ])


def generate_launch_description():
    config = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(FLEET / 'worlds/good_map.world'),
                              description='실제 방(good3 지도)을 벽으로 세운 월드(물리 step 4ms)'),
        DeclareLaunchArgument('map', default_value=str(FLEET / 'maps/good3.yaml'),
                              description='world에 맞는 지도. 월드를 바꾸면 지도도 같이 바꾼다'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo 화면 표시'),
        DeclareLaunchArgument('headless_rendering', default_value='false',
                              description='화면 없는 PC에서 라이다·카메라 렌더링(EGL)'),
        DeclareLaunchArgument('fleet', default_value='true',
                              description='Nav2 두 개와 웹 대시보드(multi_robot.launch.py)도 실행'),
        # 생성 위치: good3 지도의 빈 곳(가장 가까운 벽까지 robot1 0.18 m, robot2 0.36 m). robot1은 왼쪽 방, robot2는 오른쪽 방
        DeclareLaunchArgument('robot1_domain', default_value='25'),
        DeclareLaunchArgument('robot1_x', default_value='0.5'),
        DeclareLaunchArgument('robot1_y', default_value='0.5'),
        DeclareLaunchArgument('robot1_yaw', default_value='0.0'),
        DeclareLaunchArgument('robot2_domain', default_value='27'),
        DeclareLaunchArgument('robot2_x', default_value='2.0'),
        DeclareLaunchArgument('robot2_y', default_value='0.5'),
        DeclareLaunchArgument('robot2_yaw', default_value='0.0'),

        # 시뮬은 이 PC 안에서만 통신한다. 실물용 DDS 설정(peer 목록, 프로파일, 디스커버리 서버)은 자식에게 물려주지 않는다.
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        UnsetEnvironmentVariable('ROS_STATIC_PEERS'),
        UnsetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE'),
        UnsetEnvironmentVariable('ROS_DISCOVERY_SERVER'),
        UnsetEnvironmentVariable('ROS_SUPER_CLIENT'),
        UnsetEnvironmentVariable('CYCLONEDDS_URI'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(DESCRIPTION.parent)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(GZ_SIM / 'models')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ROS_GZ_SIM / 'launch/gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-r -s -v2 ',
                            PythonExpression(["'--headless-rendering ' if '", config('headless_rendering'),
                                              "'.lower() in ('true', '1') else ''"]),
                            config('world')],
                'on_exit_shutdown': 'true'}.items()),
        # on_exit_shutdown을 명시하지 않으면 위 include의 'true'가 새어 들어와 GUI 창을 닫는 순간 전부 꺼진다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ROS_GZ_SIM / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'false'}.items(),
            condition=IfCondition(config('gui'))),

        robot('robot1'),
        robot('robot2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(FLEET / 'launch/multi_robot.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'map': config('map'),
                              'robot1_domain': config('robot1_domain'),
                              'robot2_domain': config('robot2_domain'),
                              # 생성 위치를 알고 있으니 초기 위치를 사람이 찍지 않아도 된다
                              **{f'{r}_initial_pose': [config(f'{r}_x'), ',', config(f'{r}_y'), ',', config(f'{r}_yaw')]
                                 for r in ('robot1', 'robot2')}}.items(),
            condition=IfCondition(config('fleet'))),
    ])
