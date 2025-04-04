"""Basic streaming test launch"""

# Testing file only

from launch import LaunchDescription
from launch_ros.actions import Node

CAM_NUMBERS = [31]
CAM_QUALITIES = [2]

def generate_launch_description():
    ld = LaunchDescription()

    # Start node
    ld.add_action(
        Node(
            executable='CameraManager',
            package='camera_manager',
            parameters=[
                {'stream_prestart' : CAM_NUMBERS},
                {'stream_prestart_qual' : CAM_QUALITIES}
            ],
            # output={'both' : 'log'}, # Mute executable output
        )
    )

    return ld