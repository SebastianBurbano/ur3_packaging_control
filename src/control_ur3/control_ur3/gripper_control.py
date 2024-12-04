#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        self.get_logger().info("Nodo de control del gripper iniciado. Enviando comando para abrir el gripper...")
        self.open_gripper()
        raise SystemExit

    def open_gripper(self):
        # Publicar el comando en el topic correspondiente
        pub = self.create_publisher(String, '/urscript_interface/script_command', 10)
        msg = String()
        msg.data = (
            "def my_prog():\n"
            "  set_digital_out(0, False)\n"
            "  textmsg(\"motion finished\")\n"
            "end"
        )
        pub.publish(msg)
        self.get_logger().info("Comando enviado para abrir el gripper.")

def main(args=None):
    rclpy.init(args=args)
    gripper_control_node = GripperControlNode()
    rclpy.spin(gripper_control_node)  # Usamos spin_once ya que es un nodo simple
    rclpy.shutdown()

if __name__ == '__main__':
    main()