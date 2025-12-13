from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Получаем путь к пакету
    pkg_dir = get_package_share_directory('imu_csv_publisher')
    
    # Путь к CSV файлу (по умолчанию в директории пакета)
    csv_file = os.path.join(pkg_dir, 'ros_imu_data.csv')
    
    # Создаем ноду
    imu_publisher_node = Node(
        package='imu_csv_publisher',
        executable='csv_imu_publisher',
        name='csv_imu_publisher',
        output='screen',
        parameters=[{
            'csv_file': csv_file,
            'publish_frequency': 100.0,  # Hz
            'publish_mag': True,
            'frame_id': 'imu_link'
        }]
    )
    
    return LaunchDescription([
        imu_publisher_node
    ])