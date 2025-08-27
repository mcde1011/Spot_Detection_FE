from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch-Argumente (bei Bedarf anpassen/überschreiben)
    model_path = LaunchConfiguration('model_path')
    device     = LaunchConfiguration('device')
    imgsz      = LaunchConfiguration('imgsz')
    conf       = LaunchConfiguration('conf')
    iou        = LaunchConfiguration('iou')

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='/home/dennis/ros2_ws/src/ricoh_theta_ros/resource/best.pt'),
        DeclareLaunchArgument('device',     default_value='auto'),   # "cpu", "0" (GPU) oder "auto"
        DeclareLaunchArgument('imgsz',      default_value='640'),
        DeclareLaunchArgument('conf',       default_value='0.25'),
        DeclareLaunchArgument('iou',        default_value='0.45'),

        # 1) Ricoh-Publisher (sendet alle 6 Ansichten auf EIN Topic /ricoh_theta/views)
        Node(
            package='ricoh_theta_ros',
            executable='ricoh_publisher',
            name='ricoh_publisher',
            output='screen',
            # falls dein Publisher Parameter hat, hier unter "parameters=[{...}]"
        ),

        # YOLO: 6 Eingangs-Topics
        Node(
            package='ricoh_theta_ros',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
                'imgsz': LaunchConfiguration('imgsz'),
                'conf': LaunchConfiguration('conf'),
                'iou': LaunchConfiguration('iou'),
                'input_topics': [
                    '/ricoh_theta/front',
                    '/ricoh_theta/right',
                    '/ricoh_theta/back',
                    '/ricoh_theta/left',
                    '/ricoh_theta/up',
                    '/ricoh_theta/down',
                ],
                'output_namespace': '/detections',
                'publish_annotated': True,
            }]
        ),

        # Mosaic: Multi-Topic-Modus
        # Node(
        #     package='ricoh_theta_ros',
        #     executable='mosaic_viewer',
        #     name='mosaic_viewer',
        #     output='screen',
        #     parameters=[{
        #         'use_single_topic': False,
        #         'topics': [
        #             '/ricoh_theta/front',
        #             '/ricoh_theta/right',
        #             '/ricoh_theta/back',
        #             '/ricoh_theta/left',
        #             '/ricoh_theta/up',
        #             '/ricoh_theta/down',
        #         ],
        #         'tile_width': 420,
        #         'tile_height': 420,
        #         'grid_rows': 2,
        #         'grid_cols': 3,
        #         'publish_mosaic': True,
        #         'mosaic_topic': '/ricoh_theta/mosaic'
        #     }]
        # ),

    ])
