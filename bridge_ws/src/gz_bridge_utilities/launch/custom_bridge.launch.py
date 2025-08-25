from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
 
        Node(
            package='ros_gz_image',
            namespace='',
            executable='image_bridge',
            name='bridge_images',
            arguments=["/camera",
                       "/depth_camera",]

        ),

        Node(
            package='ros_gz_bridge',
            namespace='',
            executable='parameter_bridge',
            name='bridge_poses',
            arguments=["/model/x500_depth_0/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V","--ros-args",
                          "-r", "/model/x500_depth_0/pose:=/gazebo/model/state",]
        ),

        Node(
            package='ros_gz_bridge',
            namespace='',
            executable='parameter_bridge',
            name='bridge_camera_info',
            arguments=["/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo","--ros-args"]
        ),




        Node(
            package='ros_gz_bridge',
            namespace='',
            executable='parameter_bridge',
            name='bridge_points',
            arguments=["/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                       "--ros-args",
                          "-r", "/depth_camera/points:=/gazebo/points",]
        ),

        Node(
            package='ros_gz_bridge',
            namespace='',
            executable='parameter_bridge',
            name='bridge_tfs',
            arguments=["/model/x500_depth_0/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V","--ros-args",
                          "-r", "/model/x500_depth_0/pose:=/tf",]
        ),

        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "x500_depth_0"]
        #),

        Node(
            package='tf2_ros',
            namespace = 'scan_to_map',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "camera_point_cloud", "x500_depth_0/OakD-Lite/base_link/StereoOV7251"]
        ),

        Node(
            package='tf2_ros',
            namespace = 'attach_point_cloud',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "x500_depth_0/OakD-Lite/base_link", "camera_point_cloud"]
        ),

        # Node(
        #     package='depth_image_proc',
        #     #plugin='depth_image_proc::PointCloudXyzNode',
        #     executable='point_cloud_xyz_node',
        #     name='point_cloud_xyz',
        #     remappings=[('image_rect', '/depth_camera'),
        #                 ('camera_info', '/camera_info'),
        #                  ('/points', '/im_proc/points'),
        #                 ]
        # )
        
    ])
