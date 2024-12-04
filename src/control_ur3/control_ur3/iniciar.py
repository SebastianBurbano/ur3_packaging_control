#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class Reconnect(Node):
    def __init__(self):
        super().__init__('reconnect_node')
        self.get_logger().info("Nodo de reconexión iniciado. Intentando reconectar el driver...")
        self.reconnect_driver()
        raise SystemExit

    def reconnect_driver(self):
        cli = self.create_client(Trigger, '/dashboard_client/play')
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("El servicio '/dashboard_client/play' no está disponible.")
            return

        req = Trigger.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("Conexión con el driver restablecida exitosamente.")
        else:
            self.get_logger().error("Error al intentar reconectar el driver.")

def main(args=None):
    rclpy.init(args=args)
    reconnect_node = Reconnect()
    rclpy.spin(reconnect_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
