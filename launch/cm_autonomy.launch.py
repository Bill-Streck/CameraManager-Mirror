"""Basic autonomy launch"""

# XXX In preview phase for testing

from launch import LaunchDescription
from launch_ros.actions import Node

AUTONOMY_CAM_NUMBERS = [27, 28, 29, 30] # temp
AUTONOMY_CAM_QUALITIES = [7, 7, 7, 7]

def generate_launch_description():
    ld = LaunchDescription()

    # Start node
    ld.add_action(
        Node(
            executable='CameraManager',
            package='camera_manager',
            parameters=[
                {'cameras_prestart' : AUTONOMY_CAM_NUMBERS},
                {'cam_prestart_qual' : AUTONOMY_CAM_QUALITIES}
            ],
            output={'both' : 'log'}, # Mute executable output
        )
    )

    return ld