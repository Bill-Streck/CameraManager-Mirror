# CameraManager

CameraManager is a comprehensive camera management solution for Daedalus. CameraManager is built out of modular components and lasting, public libraries to maximize adaptability for the future.

Please note this README is geared towards the primary computer system on the rover and not towards the Nvidia Jetson. The Jetson version of CameraManager excludes streaming and debug utilities.

## License

CameraManager is licensed under the MIT License.

## Table of Contents

- [Inputs and Outputs](#inputs-and-outputs)
    - [ROS2 Inputs](#ros2-inputs)
    - [ROS2 Outputs](#ros2-outputs)
    - [Streaming Output](#streaming-output)

## Inputs and Outputs

Lists all inputs and outputs of CameraManager. See the launch folder for example uses of different parameters. Note some launch files have content geared towards old autonomy testing methods, prior to replugging cameras in the Jetson.

#### ROS2 Inputs

CameraManager has one input for the modular commands. Detailed comments are found in robot_interfaces/msg/CameraManagerCommand.msg.

- Type: robot_interfaces/msg/CameraManagerCommand
- Uses: Local start/stop, stream start/stop (independently addressable), force restarts and attribute modification
- Default topic: "/camera_manager"
- Topic set parameter (launch file): "cm_sub_topic"

#### ROS2 Outputs

Images (local machine) are output as the generic ROS2 image msg.

- Type: sensor_msgs/Image (standard ROS2 library)
- Uses: Local image analysis (aruco, object, ...)
- Default topic: "/image_topic"
- Topic set parameter (launch file): "cm_image_topic"

CameraManager has a debug channel. At the moment, this is not implemented.

- Type: std_msgs/UInt32 (standard ROS2 library)
- Uses: Error and info codes in response to events.
    - While not implemented, there are multiple clearly commented spots where it should be implemented. This is largely dependent on actually be able to use the response, which is not under consideration at this moment.
- Default topic: "/camera_manager_debug"
- Topic set parameter (launch file): "cm_pub_topic"

CameraManager also has a metadata channel that is geared towards image processing nodes. This is particularly important on the Jetson version of the repository.

- Type: robot_interfaces/msg/ImageMetadata
- Uses: Outputs focal length, sensor height, image height and image width
- Default topic: "/CM_cam_meta"
- Topic set parameter (launch file): "cm_meta_topic"

#### Streaming Output

Unlike most other parts of the system, CameraManager does not stream over ROS2. This is to avoid the side effects of TCP interaction.

TODO awaiting