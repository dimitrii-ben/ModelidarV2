import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial


class STM32Node(Node):
    def __init__(self):
        super().__init__('stm32_node')
        self.publisher = self.create_publisher(Float32, 'robot/motor_control', 10)
        self.subscription = self.create_subscription(
            Float32, 'robot/motor_control', self.motor_control_callback, 10)
        self.serial = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)

    def motor_control_callback(self, msg):
        # Gérer les commandes de contrôle des moteurs reçues du nœud de contrôle
        motor_speed = msg.data
        # Envoyer les commandes de contrôle à la carte STM32

    def send_command_to_stm32(self, command):
        self.serial.write(command.encode())

    def run(self):
        while rclpy.ok():
            # Lire les données de la carte STM32
            try:
                data = self.serial.readline().decode().strip()
                if data:
                    # Traiter les données reçues de la carte STM32
                    # Effectuer les opérations nécessaires avec les données reçues
            except serial.SerialException as e:
                self.get_logger().error('Serial communication error: {}'.format(str(e)))

            # Autres opérations à effectuer périodiquement, si nécessaire

    def destroy(self):
        self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    stm32_node = STM32Node()
    try:
        stm32_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        stm32_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



if __name__ == '__main__':
    main()
