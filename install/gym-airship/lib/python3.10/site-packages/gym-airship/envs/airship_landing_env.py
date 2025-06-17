import gym
from gym import spaces
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty # Para resetar a simulação no Gazebo
import math
import numpy as np
import time

# Certifique-se de que o rclpy seja inicializado e finalizado corretamente
# em um processo separado ou gerenciado de forma assíncrona para não bloquear o Gym
# Para simplificar, neste exemplo, vamos supor que o ROS esteja rodando e
# que os tópicos e serviços estejam disponíveis.
# Em uma implementação robusta, você pode querer gerenciar o lifecycle do ROS dentro da classe.

class AirshipLandingEnv(gym.Env):
    metadata = {'render_modes': ['human'], 'render_fps': 30}

    def __init__(self):
        super(AirshipLandingEnv, self).__init__()

        # Defina o espaço de ação:
        # Por exemplo, para um dirigível, podemos controlar a velocidade linear (x, y, z) e angular (z)
        # Assumindo que você usa Twist para controle de velocidade.
        # Os limites devem ser baseados nas capacidades do seu dirigível no Gazebo.
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0, -1.0, -1.0]),  # Min linear x, y, z, angular z
            high=np.array([1.0, 1.0, 1.0, 1.0]),   # Max linear x, y, z, angular z
            dtype=np.float32
        )

        # Defina o espaço de observação:
        # Por exemplo, a posição do dirigível (x, y, z) e a distância até o alvo.
        # Você pode precisar adicionar mais estados, como velocidade, orientação, etc.
        self.observation_space = spaces.Box(
            low=np.array([-np.inf, -np.inf, -np.inf, 0.0]), # x, y, z, distance_to_target
            high=np.array([np.inf, np.inf, np.inf, np.inf]),
            dtype=np.float32
        )

        self.target_position = np.array([0.0, 0.0, 0.5])  # Posição de pouso alvo (x, y, z) no Gazebo
                                                        # Ajuste este valor conforme seu ambiente Gazebo
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.previous_distance = float('inf')

        # Inicialização do ROS (maneira simplificada para o exemplo)
        # Em um ambiente real, você pode querer gerenciar o contexto ROS de forma mais robusta.
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node('airship_gym_env_node')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.node.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.reset_simulation_service = self.node.create_client(Empty, '/reset_simulation')


    def _odom_callback(self, msg):
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        self.current_position[2] = msg.pose.pose.position.z
        # Para um controle mais preciso, você também pode querer extrair a orientação (quaternions)
        # e velocidades (linear e angular) da mensagem Odometry.

    def _get_obs(self):
        distance_to_target = np.linalg.norm(self.current_position - self.target_position)
        return np.array([
            self.current_position[0],
            self.current_position[1],
            self.current_position[2],
            distance_to_target
        ], dtype=np.float32)

    def _get_info(self):
        return {
            "current_x": self.current_position[0],
            "current_y": self.current_position[1],
            "current_z": self.current_position[2],
            "target_x": self.target_position[0],
            "target_y": self.target_position[1],
            "target_z": self.target_position[2],
        }

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # 1. Chamar o serviço de reset do Gazebo
        # Isso é crucial para reiniciar a simulação e o dirigível para uma posição inicial.
        # Você precisará configurar um serviço no Gazebo ou em um nó ROS para fazer isso.
        # Se você não tiver um serviço, precisará simular um reset através de teletransporte
        # do dirigível para uma posição inicial (o que pode não ser tão limpo).
        self.node.get_logger().info("Chamando serviço de reset do Gazebo...")
        if not self.reset_simulation_service.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("Serviço /reset_simulation não disponível!")
            # Trate o erro ou lance uma exceção
        else:
            request = Empty.Request()
            future = self.reset_simulation_service.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            if future.result() is not None:
                self.node.get_logger().info("Simulação do Gazebo resetada.")
            else:
                self.node.get_logger().error("Falha ao chamar o serviço de reset do Gazebo.")


        # 2. Esperar um curto período para o Gazebo se estabilizar após o reset
        time.sleep(1.0) # Ajuste conforme necessário para o seu simulador

        # 3. Obter a observação inicial após o reset
        # É importante que a posição inicial do dirigível seja definida pelo Gazebo
        # ou por uma configuração inicial específica após o reset.
        rclpy.spin_once(self.node, timeout_sec=0.1) # Processa uma mensagem de odometria para obter a posição atual
        observation = self._get_obs()
        info = self._get_info()
        self.previous_distance = np.linalg.norm(self.current_position - self.target_position)

        self.node.get_logger().info(f"Ambiente resetado. Posição inicial: {self.current_position}")
        return observation, info

    def step(self, action):
        # Converter a ação para uma mensagem Twist e publicar
        twist_msg = Twist()
        twist_msg.linear.x = float(action[0])
        twist_msg.linear.y = float(action[1])
        twist_msg.linear.z = float(action[2])
        twist_msg.angular.z = float(action[3]) # Supondo controle yaw
        self.publisher.publish(twist_msg)

        # Espere um pequeno período para a ação ter efeito na simulação
        time.sleep(0.1) # Ajuste este valor para controlar a granularidade do tempo do seu passo.
                        # Um valor muito pequeno pode não dar tempo suficiente para a simulação reagir.
                        # Um valor muito grande pode tornar o treinamento lento.

        # Processar as mensagens ROS para obter o estado atual
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # Calcular observação, recompensa, done e info
        observation = self._get_obs()
        current_distance = np.linalg.norm(self.current_position - self.target_position)

        reward = 0.0
        terminated = False
        truncated = False

        # Recompensa por se aproximar do alvo
        if current_distance < self.previous_distance:
            reward += 1.0  # Recompensa por se aproximar
        else:
            reward -= 0.5  # Penalidade por se afastar

        # Recompensa por estar na posição alvo (pouso)
        landing_threshold = 0.1 # Distância em metros para considerar "pousado"
        z_landing_threshold = 0.1 # Altura z para considerar "pousado"
        if current_distance < landing_threshold and abs(self.current_position[2] - self.target_position[2]) < z_landing_threshold:
            reward += 100.0 # Grande recompensa pelo pouso
            terminated = True # O episódio termina com sucesso

        # Penalidade por colisão ou sair dos limites (adicione sua lógica de colisão/limites)
        # Por exemplo, se a altura for muito baixa (colisão com o chão antes do alvo)
        if self.current_position[2] < -0.1: # Exemplo: abaixo do chão
            reward -= 50.0
            terminated = True

        # Penalidade por tempo (para encorajar o pouso rápido)
        reward -= 0.1 # Pequena penalidade a cada passo

        self.previous_distance = current_distance
        info = self._get_info()

        return observation, reward, terminated, truncated, info

    def render(self):
        # Se você quiser visualizar o ambiente, você pode adicionar lógica aqui.
        # Por exemplo, imprimir a posição do dirigível ou usar uma biblioteca de visualização.
        pass

    def close(self):
        # Limpeza de recursos ROS
        self.node.destroy_node()
        # rclpy.shutdown() # Cuidado ao chamar rclpy.shutdown() se outros nós estiverem ativos
        pass