#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import os

class JointStateCaptureNode(Node):
    def __init__(self):
        super().__init__('joint_state_capture')
        
        self.file_path = 'trajectories.yaml'
        self.current_trajectory_key = self.declare_parameter('trajectory_key', 'traj0').value
        self.trajectories = {"traj0": []}
        self.point_count = 0
        self.time_increment = 4.0   # Tiempo fijo entre puntos
        self.captured = False  # Controla si ya se capturó un punto

        self.subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.get_logger().info("Nodo de captura de estado de articulaciones iniciado.")

    def joint_state_callback(self, msg):

        # Leer el archivo si existe
        trajectories = {}
        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as file:
                try:
                    trajectories = yaml.safe_load(file) or {}
                except yaml.YAMLError:
                    self.get_logger().warning('El archivo YAML estaba corrupto, se sobreescribirá.')
                    trajectories = {}
        
        # Verificar o inicializar la trayectoria actual
        if self.current_trajectory_key not in trajectories:
            trajectories[self.current_trajectory_key] = []

        # Agregar el nuevo punto
        point = {
            'joint_names': list(msg.name),
            'position': list(msg.position),
            'velocity': list(msg.velocity),
            'timestamp': {
                 "sec": 5,  # Tiempo relativo entre puntos
                "nanosec": 0
            },
        }

        # Agregar el punto a la trayectoria
        trajectories[self.current_trajectory_key].append(point)
        self.point_count += 1


        # Guardar el archivo actualizado
        with open(self.file_path, 'w') as file:
            yaml.dump(trajectories, file)

        self.get_logger().info(f'Punto capturado y guardado en {self.file_path}.')
        
        # Marcar como capturado y detener el nodo
        self.captured = True
        self.get_logger().info("Punto capturado. Nodo finalizado.")
        raise SystemExit  # Salir del nodo
    
    def set_new_trajectory(self, trajectory_name):
        self.current_trajectory_key = trajectory_name
        self.point_count = 0  # Reiniciar el contador de tiempo para la nueva trayectoria
        self.get_logger().info(f'Cambió a la trayectoria {trajectory_name}.')

def main(args=None):
    rclpy.init(args=args)

    node = JointStateCaptureNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("joint_state_capture").info("Nodo finalizado correctamente.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()