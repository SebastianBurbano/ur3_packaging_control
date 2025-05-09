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
        # Declarar los parametros
        self.current_sequence = self.declare_parameter('sequence_name', 'empaque_4').value
        self.clear_sequence = self.declare_parameter('clear_sequence', False).value
        self.gripper_action = self.declare_parameter('gripper_action', 'none').value
        self.time_step = self.declare_parameter('time_step', 4.0).value
        self.movement_type = self.declare_parameter('movement_type', 'moveJ').value

        # Declarar parámetros para moveL
        # (valor por defecto para que no falle si no se pasa nada desde la CLI)
        self.num_steps = self.declare_parameter('num_steps', 5).value
        self.interp_p_time = self.declare_parameter('interp_p_time', 2.0).value
        
        self.current_trajectory_key = None  # Esto se asignará dinámicamente
        self.sequences = {'pickup_point': {},'empaque_4': [], 'empaque_6': [], 'empaque_12': []}

        self.point_count = 0
        self.captured = False  # Controla si ya se capturó un punto

        # Limpiar la secuencia si se solicita
        if self.clear_sequence:
            self.clear_active_sequence()
            self.get_logger().info("Secuencia limpiada. Nodo finalizado.")
            raise SystemExit  # Salir del nodo después de limpiar

        self.subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.get_logger().info(f"Nodo de captura iniciado. Secuencia activa: {self.current_sequence}")

    def clear_active_sequence(self):
        # Leer el archivo si existe
        sequences = {}
        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as file:
                try:
                    sequences = yaml.safe_load(file) or {}
                except yaml.YAMLError:
                    self.get_logger().warning('El archivo YAML estaba corrupto, se sobreescribirá.')
                    sequences = {}

        # Borrar la secuencia actual
        if self.current_sequence in sequences:
            sequences[self.current_sequence] = {}
            with open(self.file_path, 'w') as file:
                yaml.dump(sequences, file)
            self.get_logger().info(f"Secuencia {self.current_sequence} borrada correctamente.")
        else:
            self.get_logger().info(f"La secuencia {self.current_sequence} no existe. No se realizó ningún cambio.")

    def joint_state_callback(self, msg):
        # Leer el archivo si existe
        sequences = {}
        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as file:
                try:
                    sequences = yaml.safe_load(file) or {}
                except yaml.YAMLError:
                    self.get_logger().warning('El archivo YAML estaba corrupto, se sobreescribirá.')
                    sequences = {}

        # Verificar o inicializar la secuencia actual
        if self.current_sequence not in sequences:
            sequences[self.current_sequence] = {}

        # Asignar automáticamente una nueva clave de trayectoria
        current_sequence_data = sequences[self.current_sequence]
        next_traj_index = len(current_sequence_data)  # Índice de la siguiente trayectoria
        self.current_trajectory_key = f"traj{next_traj_index}"

        # Agregar el nuevo punto
        point = {
            'joint_names': list(msg.name),
            'movement_type': self.movement_type,
            'position': list(msg.position),
            'velocity': list(msg.velocity),
            'timestamp': {
                "sec": int(self.time_step),  # Se usa el parámetro time_step
                "nanosec": int((self.time_step - int(self.time_step)) * 1e9)  # Parte decimal a nanosegundos
            },
            'gripper_action': self.gripper_action,  # Agregar la acción del gripper
        }
        
         # Solo si es moveL, agregamos num_steps e interp_p_time (como "time_step" para interpolation)
        if self.movement_type == "moveL":
            point["num_steps"] = self.num_steps
            # OJO: en 'control_movimiento.py' se usa "time_step" para la interpolación.
            # Por eso aquí mapeamos interp_p_time -> time_step
            point["time_step"] = self.interp_p_time

        # Agregar el punto a la nueva trayectoria
        current_sequence_data[self.current_trajectory_key] = [point]
        self.point_count += 1
        
        sequences[self.current_sequence] = current_sequence_data

        # Guardar el archivo actualizado
        with open(self.file_path, 'w') as file:
            yaml.dump(sequences, file)

        self.get_logger().info(f'Punto capturado y guardado en {self.file_path} bajo la secuencia {self.current_sequence}.')

        # Marcar como capturado y detener el nodo
        self.captured = True
        self.get_logger().info("Punto capturado. Nodo finalizado.")
        raise SystemExit  # Salir del nodo
    
    def set_new_sequence(self, sequence_name):
        if sequence_name in self.sequences:
            self.current_sequence = sequence_name
            self.point_count = 0  # Reiniciar el contador de tiempo
            self.get_logger().info(f'Cambió a la secuencia {sequence_name}.')
        else:
            self.get_logger().error(f"La secuencia {sequence_name} no existe.")

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
