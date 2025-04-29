#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from ur_msgs.srv import SetIO
from std_srvs.srv import Trigger
import asyncio
#from asyncua import Client
import yaml


def reorder_joint_data(joint_names_from_yaml, joint_data, expected_joint_order):
    # Crear un diccionario que mapea los nombres de las juntas al índice de sus datos
    joint_map = {name: i for i, name in enumerate(joint_names_from_yaml)}

    # Reorganizar los datos en el orden esperado
    reordered_data = [0.0] * len(expected_joint_order)
    for i, joint_name in enumerate(expected_joint_order):
        if joint_name in joint_map:
            reordered_data[i] = joint_data[joint_map[joint_name]]
    return reordered_data


def load_trajectories_from_yaml(file_path, sequence_name, expected_joint_order):
    with open(file_path, "r") as file:
        data = yaml.safe_load(file)
    
    if sequence_name not in data:
        raise ValueError(f"La secuencia '{sequence_name}' no se encontró en el archivo YAML.")

    trajectories = {}
    for traj_name, points in data[sequence_name].items():
        trajectory = []
        joint_names_from_yaml = points[0]["joint_names"]  # Tomar el orden de las juntas del YAML
        for pt in points:
            trajectory.append({
                "positions": reorder_joint_data(joint_names_from_yaml, pt["position"], expected_joint_order),
                "velocities": reorder_joint_data(joint_names_from_yaml, pt["velocity"], expected_joint_order),
                "time_from_start": Duration(
                    sec=pt["timestamp"]["sec"], nanosec=pt["timestamp"]["nanosec"]
                ),
                "gripper_action": pt.get("gripper_action", "none")  # Acción del gripper
            })
        trajectories[traj_name] = trajectory
    return trajectories


class TrajectoryGripperController(Node):
    def __init__(self):
        super().__init__('trajectory_gripper_controller')

        # Declarar parámetros
        self.declare_parameter('opcua_url', 'opc.tcp://192.168.0.230:4840')
        self.declare_parameter('sensor_node_id', 'ns=4;i=2')
        self.declare_parameter("delay_between_trajectories", 0.3)
        self.delay_between_trajectories = self.get_parameter("delay_between_trajectories").value
        self.declare_parameter('controller_name', 'scaled_joint_trajectory_controller')
        self.declare_parameter('trajectory_file', 'trajectories.yaml')
        self.declare_parameter('sequence_name', 'empaque_4')
        self.num_packages = self.declare_parameter('num_packages', 1).value
        self.current_package_count = 0  # Contador de paquetes completados
        self.declare_parameter(
            'joints',
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

        """ #Configuración OPC-UA
        self.opcua_url = self.get_parameter('opcua_url').value
        self.sensor_node_id = self.get_parameter('sensor_node_id').value
        self.loop = asyncio.new_event_loop()  # Crear un nuevo bucle de eventos
        
        # Configuración del cliente OPC-UA
        self.opcua_client = Client(self.opcua_url)
        self.get_logger().info("Conectando al servidor OPC-UA...")
        self.loop.run_until_complete(self.connect_opcua()) """
        
        # Configuración del cliente de acción
        controller_name = self.get_parameter('controller_name').value + '/follow_joint_trajectory'
        self.joints = self.get_parameter('joints').value
        self.trajectory_file = self.get_parameter('trajectory_file').value
        self.sequence_name = self.get_parameter('sequence_name').value
    
        if not self.joints:
            raise ValueError('Debe especificar los nombres de las juntas.')

        self.action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f'Esperando al servidor de acciones en {controller_name}...')
        self.action_client.wait_for_server()

        # Cargar trayectorias
        self.start_point_data = load_trajectories_from_yaml(self.trajectory_file, 'start_point', self.joints)
        self.tapas_point_data = load_trajectories_from_yaml(self.trajectory_file, 'tapas_point', self.joints)
        self.pickup_point_data = load_trajectories_from_yaml(self.trajectory_file, 'pickup_point', self.joints)
        self.trajectory_data = load_trajectories_from_yaml(self.trajectory_file, self.sequence_name, self.joints)

        # Cola de acciones
        self.action_queue = []
        self.current_action = None
        self.build_action_queue()


        # Iniciar la ejecución
        self.execute_next_action()

    async def connect_opcua(self):
        try:
            await self.opcua_client.connect()
            self.sensor_node = self.opcua_client.get_node(self.sensor_node_id)
            self.get_logger().info("Conexión OPC-UA exitosa.")
        except Exception as e:
            self.get_logger().error(f"Error al conectar al servidor OPC-UA: {e}")
            rclpy.shutdown()

    def product_present(self):
        """Verificar presencia del producto leyendo del servidor OPC-UA."""
        try:
            value = self.loop.run_until_complete(self.sensor_node.read_value())
            self.get_logger().info(f"Valor del sensor OPC-UA: {value} (Tipo: {type(value)})")
            
            if isinstance(value, bool):  # Manejar si el valor es booleano
                return value
            else:
                self.get_logger().warn(f"El valor del sensor no es booleano: {value}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error al leer el sensor OPC-UA: {e}")
            
            # Si el error es una sesión inválida, intentamos reconectar
            if "BadSessionIdInvalid" in str(e):
                self.reconnect_opcua()
            
            return False  # Asumimos que el producto no está presente si no podemos leer
    
    def reconnect_opcua(self):
        """Intentar reconectar el cliente OPC-UA en caso de error de sesión."""
        self.get_logger().info("Intentando reconectar OPC-UA...")

        try:
            self.loop.run_until_complete(self.opcua_client.disconnect())
        except Exception as e:
            self.get_logger().warn(f"No se pudo cerrar la conexión anterior: {e}")

        try:
            self.loop.run_until_complete(self.opcua_client.connect())
            self.sensor_node = self.opcua_client.get_node(self.sensor_node_id)
            self.get_logger().info("Reconexión OPC-UA exitosa.")
        except Exception as e:
            self.get_logger().error(f"Error al reconectar OPC-UA: {e}")


    
    def build_action_queue(self):

        # 1. Agregar la secuencia pickup_point inicial
        for traj_name, points in self.tapas_point_data.items():
            self.action_queue.append(('tapas_point_trajectory', traj_name))
            for pt in points:
                if pt['gripper_action'] != 'none':
                    self.action_queue.append(('gripper', pt['gripper_action']))
        
        for traj_name, points in self.pickup_point_data.items():
            self.action_queue.append(('pickup_point_trajectory', traj_name))
            for pt in points:
                if pt['gripper_action'] != 'none':
                    self.action_queue.append(('gripper', pt['gripper_action']))
        
        for traj_name, points in self.start_point_data.items():
            self.action_queue.append(('start_point_trajectory', traj_name))
            for pt in points:
                if pt['gripper_action'] != 'none':
                    self.action_queue.append(('gripper', pt['gripper_action'])) 
            

        # 2. Alternar trayectorias principales y pickup_point (excepto después de la última trayectoria)
        trajectory_names = list(self.trajectory_data.keys())
        num_trajectories = len(trajectory_names)

        for i, traj_name in enumerate(trajectory_names):
            # Agregar la trayectoria principal
            self.action_queue.append(('trajectory', traj_name))
            for pt in self.trajectory_data[traj_name]:
                if pt['gripper_action'] != 'none':
                    self.action_queue.append(('gripper', pt['gripper_action']))
            
            for traj_name, points in self.start_point_data.items():
                self.action_queue.append(('start_point_trajectory', traj_name))
                for pt in points:
                    if pt['gripper_action'] != 'none':
                        self.action_queue.append(('gripper', pt['gripper_action']))

            # Agregar tapas_point y pickup_point antes de cada empaquetado (excepto en la última)
            if i < num_trajectories - 1:
                for tapas_traj_name, tapas_points in self.tapas_point_data.items():
                    self.action_queue.append(('tapas_point_trajectory', tapas_traj_name))
                    for tapas_pt in tapas_points:
                        if tapas_pt['gripper_action'] != 'none':
                            self.action_queue.append(('gripper', tapas_pt['gripper_action']))

                for pickup_traj_name, pickup_points in self.pickup_point_data.items():
                    self.action_queue.append(('pickup_point_trajectory', pickup_traj_name))
                    for pickup_pt in pickup_points:
                        if pickup_pt['gripper_action'] != 'none':
                            self.action_queue.append(('gripper', pickup_pt['gripper_action']))
                
                for start_traj_name, start_points in self.start_point_data.items():
                    self.action_queue.append(('start_point_trajectory', start_traj_name))
                    for start_pt in start_points:
                        if start_pt['gripper_action'] != 'none':
                            self.action_queue.append(('gripper', start_pt['gripper_action']))

            
        

    def execute_next_action(self):
        if not self.action_queue:
            # Verificar si se ha alcanzado el número de paquetes
            self.current_package_count += 1
            if self.current_package_count >= self.num_packages:
                self.get_logger().info(f"Se completaron las {self.num_packages} repeticiones.")
                self.shutdown()
                return

            # Reiniciar la cola de acciones para repetir la secuencia
            self.get_logger().info(f"Reiniciando secuencia. Paquete actual: {self.current_package_count}/{self.num_packages}")
            self.build_action_queue()  # Reconstruir la cola de acciones

        self.current_action = self.action_queue.pop(0)
        action_type, action_data = self.current_action

        if action_type == 'start_point_trajectory':
            self.get_logger().info(f"Ejecutando trayectoria inicial: {action_data}")
            self.execute_start_point_trajectory(action_data)
        elif action_type == 'tapas_point_trajectory':
            self.get_logger().info(f"Ejecutando trayectoria de {action_data}")
            self.execute_tapas_point_trajectory(action_data)
        elif action_type == 'pickup_point_trajectory':
            self.get_logger().info(f"Ejecutando trayectoria de {action_data}")
            self.execute_pickup_point_trajectory(action_data)
        elif action_type == 'trajectory':
            self.get_logger().info(f"Ejecutando trayectoria principal: {action_data}")
            self.execute_trajectory(action_data)
        elif action_type == 'gripper':
            self.get_logger().info(f"Ejecutando acción del gripper: {action_data}")
            self.execute_gripper_action(action_data)

    def execute_tapas_point_trajectory(self, traj_name):
        """Ejecutar la secuencia tapas_point."""
        self.get_logger().info(f"Ejecutando trayectoria de tapas: {traj_name}")
        trajectory_points = self.tapas_point_data[traj_name]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=self.joints,
            points=[
                JointTrajectoryPoint(
                    positions=pt['positions'],
                    velocities=pt['velocities'],
                    time_from_start=pt['time_from_start']
                ) for pt in trajectory_points
            ]
        )
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.tapas_point_goal_response_callback)

    def tapas_point_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("La trayectoria de 'tapas_point' fue rechazada.")
            self.shutdown()
            return

        self.get_logger().info("Trayectoria de 'tapas_point' aceptada. Esperando resultado...")
        goal_handle.get_result_async().add_done_callback(self.tapas_point_result_callback)

    def tapas_point_result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trayectoria de 'tapas_point' completada con éxito.")
            time.sleep(self.delay_between_trajectories)
            self.execute_next_action()
        else:
            self.get_logger().error(f"Error en la trayectoria de 'tapas_point': {result.error_code}")
            self.shutdown()
    
    def execute_start_point_trajectory(self, traj_name):
        """Ejecutar la secuencia start_point."""
        self.get_logger().info(f"Ejecutando trayectoria de referencia (start_point): {traj_name}")
        trajectory_points = self.start_point_data[traj_name]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=self.joints,
            points=[
                JointTrajectoryPoint(
                    positions=pt['positions'],
                    velocities=pt['velocities'],
                    time_from_start=pt['time_from_start']
                ) for pt in trajectory_points
            ]
        )
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.start_point_goal_response_callback)

    def start_point_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("La trayectoria de 'start_point' fue rechazada.")
            self.shutdown()
            return

        self.get_logger().info("Trayectoria de 'start_point' aceptada. Esperando resultado...")
        goal_handle.get_result_async().add_done_callback(self.start_point_result_callback)

    def start_point_result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trayectoria de 'start_point' completada con éxito.")
            time.sleep(self.delay_between_trajectories)
            self.execute_next_action()
        else:
            self.get_logger().error(f"Error en la trayectoria de 'start_point': {result.error_code}")
            self.shutdown()

    
    def execute_pickup_point_trajectory(self, traj_name):
        """Ejecutar la secuencia pickup_point solo si hay producto presente."""
        """ self.get_logger().info(f"Verificando presencia de producto antes de ejecutar: {traj_name}")
        while not self.product_present():
            self.get_logger().info("No hay producto presente. Esperando...")
            time.sleep(1)  # Espera activa de 1 segundo

        self.get_logger().info(f"Producto detectado. Ejecutando trayectoria fija: {traj_name}") """
        trajectory_points = self.pickup_point_data[traj_name]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=self.joints,
            points=[
                JointTrajectoryPoint(
                    positions=pt['positions'],
                    velocities=pt['velocities'],
                    time_from_start=pt['time_from_start']
                ) for pt in trajectory_points
            ]
        )
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.pickup_point_goal_response_callback)

    def pickup_point_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("La trayectoria de 'pickup_point' fue rechazada.")
            self.shutdown()
            return

        self.get_logger().info("Trayectoria de 'pickup_point' aceptada. Esperando resultado...")
        goal_handle.get_result_async().add_done_callback(self.pickup_point_result_callback)

    def pickup_point_result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trayectoria de 'pickup_point' completada con éxito.")
            time.sleep(self.delay_between_trajectories)
            self.execute_next_action()
        else:
            self.get_logger().error(f"Error en la trayectoria de 'pickup_point': {result.error_code}")
            self.shutdown()
    
    def execute_trajectory(self, traj_name):
          
        self.get_logger().info(f"Ejecutando trayectoria: {traj_name}")
        trajectory_points = self.trajectory_data[traj_name]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=self.joints,
            points=[
                JointTrajectoryPoint(
                    positions=pt['positions'],
                    velocities=pt['velocities'],
                    time_from_start=pt['time_from_start']
                ) for pt in trajectory_points
            ]
        )
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)

        self.action_client.send_goal_async(goal).add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("La trayectoria fue rechazada.")
            self.shutdown()
            return

        self.get_logger().info("Trayectoria aceptada. Esperando resultado...")
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trayectoria completada con éxito.")
            time.sleep(self.delay_between_trajectories)
            self.execute_next_action()
        else:
            self.get_logger().error(f"Error en la trayectoria: {result.error_code}")
            self.shutdown()

    def execute_gripper_action(self, action):
        self.get_logger().info(f"Ejecutando acción del gripper: {action}")

        cli = self.create_client(SetIO, '/io_and_status_controller/set_io')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("El servicio SetIO no está disponible.")
            self.shutdown()
            return

        req = SetIO.Request()
        req.fun = 1
        req.pin = 0
        req.state = 1.0 if action == "close" else 0.0

        cli.call_async(req).add_done_callback(self.gripper_response_callback)

    def gripper_response_callback(self, future):
        if future.result().success:
            self.get_logger().info("Acción del gripper completada.")
            self.execute_next_action()
        else:
            self.get_logger().error("Error al ejecutar la acción del gripper.")
            self.shutdown()

    def shutdown(self):
        self.get_logger().info("Finalizando nodo.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGripperController()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
