"""Early es practice launch"""

# Testing file only

from launch import LaunchDescription
from launch_ros.actions import Node

# Set the cameras you want to start. -1 is the wrist camera.
# TODO add -1 when wrist is working
CAM_NUMBERS = [25, 26, 21]
# Front, Right, Left in that order

# Set the quality for each camera based on settings.hpp. Ordered with the above.
# If any cameras above don't get a value, quality defaults to 2 (320x180, 10fps)
CAM_QUALITIES = [4, 4, 4]
# all set to 640x360, 15 fps, turn down to 3 for 10fps.
# TODO I'll make an option for this resolution at 5fps soon

def generate_launch_description():
    ld = LaunchDescription()

    # Start node
    ld.add_action(
        Node(
            executable='CameraManager',
            package='camera_manager',
            parameters=[
                # This specifically starts the cameras for streaming.
                # See other launch files for local start examples.
                # Both streaming and local prestarts can be used simultaneously.
                {'stream_prestart' : CAM_NUMBERS},
                {'stream_prestart_qual' : CAM_QUALITIES}
            ],
            # output={'both' : 'log'}, # Mute executable output
        )
    )

    return ld