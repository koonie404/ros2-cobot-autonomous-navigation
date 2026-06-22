from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 여기서 'teleopKeyboardNode'라는 이름표를 붙여서 일꾼을 설정해요!
    teleopKeyboardNode = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        parameters=[{
            'speed': 0.5,
            'turn': 0.1
        }],
        output='screen',
        # ⭐ 이 부분이 핵심이에요! 새 터미널 창(xterm)을 열어서 실행하라는 명령이에요.
        prefix='xterm -e',
        emulate_tty=True
    )

    # 설정한 일꾼을 실행 리스트에 담아줍니다.
    return LaunchDescription([
        teleopKeyboardNode
    ])
