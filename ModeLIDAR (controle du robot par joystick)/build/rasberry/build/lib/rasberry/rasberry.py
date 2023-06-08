import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket


class CommunicationNode(Node):
    def __init__(self):
        super().__init__('communication_node')
        self.publisher = self.create_publisher(String, 'robot/communication', 10)
        self.subscription = self.create_subscription(
            String, 'robot/communication', self.communication_callback, 10)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def communication_callback(self, msg):
        # Gérer les messages reçus du nœud de contrôle
        received_message = msg.data
        # Effectuer les opérations nécessaires avec les données reçues

    def connect_to_raspberry(self, ip_address, port):
        try:
            self.socket.connect((ip_address, port))
            self.get_logger().info('Connected to Raspberry Pi at {}:{}'.format(ip_address, port))
        except socket.error as e:
            self.get_logger().error('Failed to connect to Raspberry Pi: {}'.format(str(e)))

    def run(self):
        while rclpy.ok():
            # Recevoir des données de la Raspberry Pi
            try:
                data = self.socket.recv(1024)
                if data:
                    # Traiter les données reçues
                    received_message = data.decode()
                    # Effectuer les opérations nécessaires avec les données reçues
            except socket.error as e:
                self.get_logger().error('Socket error: {}'.format(str(e)))

            # Autres opérations à effectuer périodiquement, si nécessaire

    def destroy(self):
        self.socket.close()


def main(args=None):
    rclpy.init(args=args)
    communication_node = CommunicationNode()
    communication_node.connect_to_raspberry('192.168.0.100', 5000)  # Adresse IP et port de la Raspberry Pi
    try:
        communication_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        communication_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
