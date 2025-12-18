from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 入力画像トピック引数
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='Input image topic name'
    )

    # 出力画像トピック引数
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/qrcode_image',
        description='Output image topic name with QR code overlay'
    )

    image_topic = LaunchConfiguration('image_topic')
    output_topic = LaunchConfiguration('output_topic')

    qr_code_reader_node = Node(
        package='qr',
        executable='qr_code_reader',
        name='qr_code_reader',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'output_topic': output_topic
        }]
    )

    return LaunchDescription([
        image_topic_arg,
        output_topic_arg,
        qr_code_reader_node
    ])
