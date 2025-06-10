# pointcloud.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='depth_image_proc_container',
            namespace='camera',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # Correct plugin name below:
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz',
                    remappings=[
                        ('image', 'depth/image_rect_raw'),
                        ('camera_info', 'depth/camera_info'),
                        ('points', 'depth/points'),
                    ],
                ),
                # For RGB points, use:
                # ComposableNode(
                #     package='depth_image_proc',
                #     plugin='depth_image_proc::PointCloudXyzrgbNode',
                #     name='point_cloud_xyzrgb',
                #     remappings=[
                #         ('rgb/image', 'color/image_raw'),
                #         ('depth/image', 'depth/image_rect_raw'),
                #         ('camera_info', 'color/camera_info'),
                #         ('points', 'color/points'),
                #     ],
                # ),
            ],
            output='screen',
        ),
    ])
