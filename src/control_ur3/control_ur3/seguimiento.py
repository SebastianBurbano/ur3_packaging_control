#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import signal
from sensor_msgs.msg import JointState
import psutil  # Importa psutil para identificar procesos

class MonitorPositionNode(Node):
    def __init__(self):
        super().__init__('monitor_position')

        self.target_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]  # Posición objetivo
        self.subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def find_publisher_pid(self):
        for proc in psutil.process_iter(attrs=['pid', 'name', 'cmdline']):
            if 'publisher_joint_trajectory_controller' in proc.info['cmdline']:
                return proc.info['pid']
        return None

    def joint_state_callback(self, msg):
        current_position = list(msg.position)  # Obtener la posición actual de las articulaciones

        # Comparar si el robot ha llegado a la posición objetivo
        if self.has_reached_position(current_position):
            self.get_logger().info('El robot ha alcanzado la posición objetivo, deteniendo nodo publicador.')
            pid = self.find_publisher_pid()
            if pid:
                os.kill(pid, signal.SIGINT)  # Terminar el proceso del nodo publicador

    def has_reached_position(self, current_position, tolerance=0.05):  # Puedes ajustar el valor de tolerancia
    # Comparar cada articulación con la posición objetivo dentro del rango de tolerancia
        return all(abs(current - target) <= tolerance for current, target in zip(current_position, self.target_position))
    
def main(args=None):
    rclpy.init(args=args)
    monitor_position = MonitorPositionNode()
    rclpy.spin(monitor_position)
    rclpy.shutdown()

if __name__ == '__main__':
    main()