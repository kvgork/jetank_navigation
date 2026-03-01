"""
Launch file for the ICM-20948 IMU driver.

Launches the ICM-20948 9-axis IMU driver for the Waveshare IMX219-83 Stereo
Camera module. The IMU is connected via I2C to the Jetson Orin Nano 40-pin
header (pins 3/5 = /dev/i2c-7).

The node publishes:
  - imu/data_raw       (sensor_msgs/Imu)            Raw accelerometer and gyroscope data, no orientation
  - imu/magnetic_field (sensor_msgs/MagneticField)   Raw magnetometer data from AK09916
  - imu/temperature    (sensor_msgs/Temperature)     IMU die temperature

Usage:
  ros2 launch jetank_navigation imu.launch.py
  ros2 topic echo /imu/data_raw
  ros2 topic echo /imu/magnetic_field
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    config_file = PathJoinSubstitution(
        [FindPackageShare("jetank_navigation"), "config", "icm20948.yaml"]
    )

    imu_node = Node(
        package="jetank_navigation",
        executable="icm20948_node",
        name="icm20948_imu",
        output="screen",
        parameters=[
            config_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        imu_node,
    ])
