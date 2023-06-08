import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class JoyNode(Node):
    def __init__(self):
        super().__init__('joy_node')
        self.publisher = self.create_publisher(Twist, 'robot/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        self.joy_values = {'axis': [0.0, 0.0], 'buttons': [0, 0, 0, 0]}

    def joy_callback(self, msg):
        # Mise à jour des valeurs du joystick
        self.joy_values['axis'] = msg.axes
        self.joy_values['buttons'] = msg.buttons

    def run(self):
        rate = self.create_rate(10)  # Fréquence de publication
        twist_msg = Twist()

        while rclpy.ok():
            # Utilisation des valeurs du joystick pour générer la commande Twist
            linear_speed = self.joy_values['axis'][1] * 2.0  # Facteur d'échelle pour la vitesse linéaire
            angular_speed = self.joy_values['axis'][0] * 1.5  # Facteur d'échelle pour la vitesse angulaire

            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = angular_speed

            # Publication de la commande Twist
            self.publisher.publish(twist_msg)

            # Attente pour respecter la fréquence de publication
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    joy_node = JoyNode()
    try:
        joy_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        joy_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
