from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
  params = join(
    get_package_share_directory("multi_object_tracking"), "params", "multi_object_tracking.yaml"
  )

  kitti_publisher_node = Node(
    package="multi_object_tracking",
    executable="kitti_publisher_node",
    name="kitti_publisher_node",
    parameters=[params]
  )

  tracking_node = Node(
    package="multi_object_tracking",
    executable="multi_object_tracking_node",
    name="multi_object_tracking_node"
  )

  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    arguments=["-d", join(get_package_share_directory("multi_object_tracking"), "rviz/", "vis.rviz")]
  )

  return LaunchDescription([
    kitti_publisher_node,
    tracking_node,
    rviz_node
  ])
