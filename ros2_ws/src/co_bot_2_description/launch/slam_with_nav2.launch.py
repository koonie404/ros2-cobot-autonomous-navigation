import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 우리 로봇 패키지(가방)의 주소를 찾아요
    myPkgPath = FindPackageShare('myrosbot_one').find('myrosbot_one')
    
    # 2. Nav2(내비게이션) 패키지의 주소를 찾아요
    nav2PkgPath = FindPackageShare('nav2_bringup').find('nav2_bringup')

    # 설정 파일들의 위치를 정확히 알려줘요 (내비게이션 설정과 화면 설정)
    navParamsFile = os.path.join(myPkgPath, 'config', 'navigation.yaml')
    rvizConfigFile = os.path.join(myPkgPath, 'config', 'navigation.rviz')

    # (1) 로봇 기본 준비 (센서, 모터 등을 깨워요)
    robotBringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(myPkgPath, 'launch', 'robot_bringup.launch.py')
        )
    )

    # (2) 키보드 리모컨 준비 (우리가 직접 운전할 수 있게 해요)
    teleopLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(myPkgPath, 'launch', 'teleop_twist_keyboard.launch.py')
        )
    )

    # (3) Nav2 + SLAM 실행 (지도를 그리면서 길을 찾아가는 똑똑한 기능을 켜요)
    nav2Launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2PkgPath, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'True',
            'map': ' ',
            'params_file': navParamsFile
        }.items()
    )

    # (4) RViz2 실행 (로봇이 보는 세상을 우리 눈에 보이게 화면으로 띄워요)
    rvizNode = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rvizConfigFile],
        output='screen'
    )

    # 이 모든 명령들을 한 바구니에 담아서 실행해요!
    return LaunchDescription([
        robotBringup,
        teleopLaunch,
        nav2Launch,
        rvizNode
    ])
