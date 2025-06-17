from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# def generate_launch_description():
     
#     package_dir = get_package_share_directory('dirigible_control')
#     world_path = os.path.join(package_dir, 'worlds', 'empty.world')
#     models_path = os.path.join(package_dir, 'models')

#     if not os.path.exists(world_path):
#         raise FileNotFoundError(f"Arquivo de mundo não encontrado: {world_path}")

#     print(f"World path (Caminho Mundo): {world_path}")
#     print(f"Models path (Caminho model): {models_path}")

#     return LaunchDescription([
#         # Adiciona o diretório de modelos ao GAZEBO_MODEL_PATH
#         SetEnvironmentVariable( 
#             name='GAZEBO_MODEL_PATH',
#             value=models_path
#         ),

#         ExecuteProcess(
#             cmd=['gazebo', '--verbose', world_path],
#             output='screen'
#         ),

#         Node(
#             package='dirigible_control',
#             executable='dirigible_controller',
#             output='screen'
#         )
#     ])

def generate_launch_description():
    # Caminho para o diretório de instalação do Gazebo
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # Caminho para o seu arquivo de mundo
    my_world_path = os.path.join(get_package_share_directory('dirigible_control'), 'worlds', 'empty.world')
    
    # Iniciar o Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': my_world_path}.items()
    )

    return LaunchDescription([
        gazebo_launch,
        # Adicione outros nós ROS aqui, se necessário
    ])