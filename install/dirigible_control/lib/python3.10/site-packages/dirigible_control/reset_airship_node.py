# (Exemplo - não faz parte do pacote Gym)
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist

class ResetAirship(Node):
    def __init__(self):
        super().__init__('reset_airship_node')
        self.reset_service = self.create_service(Empty, '/reset_simulation', self.handle_reset)
        self.set_model_state_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.initial_pose = Pose()
        self.initial_pose.position.x = 0.0
        self.initial_pose.position.y = 0.0
        self.initial_pose.position.z = 2.0 # Altura inicial do dirigível
        self.get_logger().info('Reset Airship Service Ready.')

    def handle_reset(self, request, response):
        self.get_logger().info('Recebido pedido de reset.')
        model_state = ModelState()
        model_state.model_name = 'blimp_w_camera' # Substitua pelo nome do seu modelo no Gazebo
        model_state.pose = self.initial_pose
        model_state.twist = Twist() # Zera a velocidade

        req = SetModelState.Request()
        req.model_state = model_state

        if not self.set_model_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Serviço /gazebo/set_model_state não disponível!")
            return response

        future = self.set_model_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Dirigível resetado para a posição inicial.')
        else:
            self.get_logger().error('Falha ao resetar o dirigível.')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ResetAirship()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()