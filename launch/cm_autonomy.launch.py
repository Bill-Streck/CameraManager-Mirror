"""Basic autonomy launch"""

# XXX In preview phase for testing

from launch import LaunchDescription
from launch_ros.actions import Node
from ..libs.CM_CommandBuilder import * # XXX Hopefully temp?

AUTONOMY_CAM_NUMBERS = [1, 3, 18, 19]

def generate_launch_description():
    ld = LaunchDescription()

    # Start node
    ld.add_action(
        Node(
            executable='CameraManager',
            package='camera_manager',
            # arguments=[]
        )
    )

    return ld