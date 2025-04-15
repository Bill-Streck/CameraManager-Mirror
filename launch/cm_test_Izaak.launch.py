"""Basic autonomy launch"""

# XXX In preview phase for testing

from launch import LaunchDescription
from launch_ros.actions import Node

CAM_NUMBERS = [32] # temp

def generate_launch_description():
    ld = LaunchDescription()

    # Start node
    ld.add_action(
        Node(
            executable='CameraManager',
            package='camera_manager',
            parameters=[
                {'cameras_prestart' : CAM_NUMBERS},
                {'cam_prestart_qual' : 4}
            ],
            output={'both' : 'log'}, # Mute executable output
        )
    )

    return ld