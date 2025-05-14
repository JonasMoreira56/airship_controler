from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obter o diretório do pacote
    package_dir = get_package_share_directory('dirigible_control')

    # Construir o caminho para o arquivo world
    # AGORA APONTA PARA O ARQUIVO .world NA PASTA worlds
    world_path = os.path.join(package_dir, 'worlds', 'world_airship.world')

    # Verificar se o arquivo de MUNDO existe
    if not os.path.exists(world_path):
        raise FileNotFoundError(f"Arquivo de mundo não encontrado: {world_path}")

    # Imprime o caminho do mundo para depuração
    print(f"World path: {world_path}")

    return LaunchDescription([

        # Iniciar o Gazebo com o ARQUIVO DE MUNDO
        ExecuteProcess(
            # AGORA PASSA O CAMINHO DO MUNDO PARA O GAZEBO
            cmd=['gazebo', '--verbose', world_path], # Adicionado --verbose para mais detalhes
            output='screen'
        ),

        # Iniciar o nó ROS 2 (sem alterações aqui)
        Node(
            package='dirigible_control',
            executable='dirigible_controller',
            output='screen'
        )
    ])