import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  # Essencial para aplicar a transformação

class LidarToRobotTransformer(Node):
    def __init__(self):
        super().__init__('lidar_to_robot_transformer')
        
        # 1. Inicializa o Buffer e o Listener do TF2
        # Isso fica escutando as articulações do CR3 em tempo real
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 2. Subscriber: Onde você recebe o ponto que o LiDAR encontrou
        # Estou assumindo que a mensagem é do tipo PointStamped
        self.subscription = self.create_subscription(
            PointStamped,
            '/ponto_detectado_lidar',
            self.point_callback,
            10
        )
            
        # 3. Publisher: Publica o ponto já convertido para o robô tentar alcançar
        self.publisher = self.create_publisher(PointStamped, '/alvo_base_cr3', 10)

    def point_callback(self, msg: PointStamped):
        # Nome dos frames (verifique se batem com o seu my_dobot_descriptor)
        target_frame = 'base_link'  # A base do Dobot CR3
        source_frame = msg.header.frame_id  # Provavelmente 'lidar_link'

        try:
            # 4. Busca a transformação exata no instante atual
            # Como o braço está se movendo, precisamos da posição de agora
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time() # Pega a transformação mais recente disponível
            )
            
            # 5. A mágica acontece aqui: aplica a matriz de transformação ao ponto
            ponto_convertido = tf2_geometry_msgs.do_transform_point(msg, transform)
            
            self.get_logger().info(
                f'Caixa! Lidar: X:{msg.point.x:.2f}, Y:{msg.point.y:.2f}, Z:{msg.point.z:.2f} | '
                f'Alvo CR3: X:{ponto_convertido.point.x:.2f}, Y:{ponto_convertido.point.y:.2f}, Z:{ponto_convertido.point.z:.2f}'
            )
            
            # Publica o ponto convertido para o seu controlador de movimento ler
            self.publisher.publish(ponto_convertido)
            
        except Exception as e:
            self.get_logger().warn(f'Aguardando árvore de TF: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LidarToRobotTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()