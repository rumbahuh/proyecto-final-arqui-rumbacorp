import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('programa6')
    #param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    detector_cmd = Node(package='camera',
                        executable='yolo_detection',
                        output='screen',
                        #parameters=[params_file],
                        remappings=[
                          ('input_detection', '/yolo/detections'),
                          ('output_detection_2d', 'detection_2d'),
                        ])

    #detector_cmd = Node(package='camera',
     #                   executable='hsv_filter',
      #                  output='screen',
       #                 #parameters=[params_file],
        #                remappings=[
         #                 ('input_image', '/color/image_raw'),
          #                ('camera_info', '/color/camera_info'),
           #               ('output_detection_2d', 'detection_2d'),
            #            ])
    
    detector_2d_3d = Node(package='camera',
                        executable='detection_2d_to_3d_depth',
                        output='screen',
                        #parameters=[params_file],
                        remappings=[
                          ('input_depth', '/rgb/image_raw'),
                          ('input_detection_2d', 'detection_2d'),
                          ('camera_info', '/rgb/camera_info'),
                          ('output_detection_3d', 'detection_3d'),
                        ])

    control_launch = Node(package='programa6',
                        executable='control_main',
                        output='screen',
                        )

    tf_launch = Node(package='programa6',
                        executable='tf_main',
                        output='screen',
                        )
    

    ld = LaunchDescription()
    ld.add_action(control_launch)
    ld.add_action(tf_launch)
    ld.add_action(detector_2d_3d)
    ld.add_action(detector_cmd)

    return ld



