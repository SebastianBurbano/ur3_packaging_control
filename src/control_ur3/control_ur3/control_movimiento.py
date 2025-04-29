#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from ur_msgs.srv import SetIO
from std_msgs.msg import String
import numpy as np
import copy
import asyncio
try:
    from asyncua import Client
    OPCUA_AVAILABLE = True
except ImportError:
    OPCUA_AVAILABLE = False
from custom_msgs.msg import OrderStatus
from std_srvs.srv import Empty
import yaml

# ----------------------------
# Funciones auxiliares
# ----------------------------

def reorder_joint_data(joint_names_from_yaml, joint_data, expected_joint_order):
    """Reordena los datos de las juntas según el orden esperado."""
    joint_map = {name: i for i, name in enumerate(joint_names_from_yaml)}
    reordered_data = [0.0] * len(expected_joint_order)
    for i, joint_name in enumerate(expected_joint_order):
        if joint_name in joint_map:
            reordered_data[i] = joint_data[joint_map[joint_name]]
    return reordered_data


def load_trajectories_from_yaml(file_path, sequence_name, expected_joint_order):
    """
    Carga trayectorias desde un archivo YAML y reordena las posiciones y velocidades
    según el orden esperado de las juntas.
    """
    with open(file_path, "r") as file:
        data = yaml.safe_load(file)
    
    if sequence_name not in data:
        raise ValueError(f"La secuencia '{sequence_name}' no se encontró en el archivo YAML.")

    trajectories = {}
    for traj_name, points in data[sequence_name].items():
        trajectory = []
        joint_names_from_yaml = points[0]["joint_names"]
        for pt in points:
            trajectory.append({
                "positions": reorder_joint_data(joint_names_from_yaml, pt["position"], expected_joint_order),
                "velocities": reorder_joint_data(joint_names_from_yaml, pt["velocity"], expected_joint_order),
                "time_from_start": Duration(
                    sec=pt["timestamp"]["sec"], nanosec=pt["timestamp"]["nanosec"]
                ),
                "gripper_action": pt.get("gripper_action", "none"),
                "movement_type": pt.get("movement_type", "moveJ"),
                "num_steps": pt.get("num_steps", 5),
                "time_step": pt.get("time_step", 2.0)
            })
        trajectories[traj_name] = trajectory
    return trajectories

# ----------------------------
# Clase principal del nodo
# ----------------------------

class TrajectoryGripperController(Node):
    """
    Nodo de ROS2 para controlar trayectorias y acciones del gripper.
    Maneja la ejecución de trayectorias, verificación de producto mediante OPC-UA, 
    y procesamiento de pedidos recibidos.
    """
    def __init__(self):
        super().__init__('trajectory_gripper_controller')

        # Declarar y obtener parámetros
        self.declare_parameter('use_topic', True)
        self.use_topic = self.get_parameter('use_topic').value
        self.declare_parameter("only_sequence_points", False)
        self.only_sequence_points = self.get_parameter("only_sequence_points").value
        self.declare_parameter('opcua_url', 'opc.tcp://192.168.0.230:4840')
        self.declare_parameter('sensor_node_id', 'ns=4;i=2')
        self.declare_parameter("delay_between_trajectories", 0.3)
        self.delay_between_trajectories = self.get_parameter("delay_between_trajectories").value
        self.declare_parameter('controller_name', 'scaled_joint_trajectory_controller')
        self.declare_parameter('trajectory_file', 'trajectories.yaml')
        self.declare_parameter('sequence_name', 'empaque_4')
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
        
        self.joints = self.get_parameter('joints').value
        if not self.joints:
            raise ValueError('Debe especificar los nombres de las juntas.')
        self.trajectory_file = self.get_parameter('trajectory_file').value
        self.sequence_name = self.get_parameter('sequence_name').value

        # Configurar cliente de acción para FollowJointTrajectory
        controller_name = self.get_parameter('controller_name').value + '/follow_joint_trajectory'
        self.action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f'Esperando al servidor de acciones en {controller_name}...')
        self.action_client.wait_for_server()

        # Inicializar variables de OPC-UA
        self.opcua_enabled = False
        self.opcua_client = None
        self.loop = None

        # Variables para manejar acciones y pedidos
        self.last_executed_position = None
        self.action_queue = []
        self.current_action = None
        self.trajectories = {}
        self.is_executing = False
        self.is_paused = True
        self.tipo_pallet = []
        self.cantidad_pallet = []
        self.current_presentation = 0
        self.current_package_count = 0

        # Cargar trayectorias estáticas
        self.tapas_point_data = load_trajectories_from_yaml(self.trajectory_file, 'tapas_point', self.joints)
        self.pickup_point_data = load_trajectories_from_yaml(self.trajectory_file, 'pickup_point', self.joints)

        # Servicios para pausar, reiniciar y habilitar OPC-UA
        self.pause_service = self.create_service(Empty, '/pause_execution', self.pause_callback)
        self.reset_service = self.create_service(Empty, '/reset_execution', self.reset_callback)
        self.opcua_service = self.create_service(Empty, '/enable_opcua', self.enable_opcua_callback)

        # Publicador para comandos de URScript
        self.urscript_publisher = self.create_publisher(String, '/urscript_interface/script_command', 10)

        # Configurar suscripción o ejecución por parámetros según el modo
        if self.use_topic:
            self.subscription = self.create_subscription(
                OrderStatus, '/confirmed_order', self.order_callback, 10
            )
            self.get_logger().info("Modo TÓPICO: Esperando pedidos en /confirmed_order.")
        else:
            self.get_logger().info("Modo PARÁMETRO: Ejecutando trayectoria definida en parámetros.")
            self.trajectory_data = load_trajectories_from_yaml(self.trajectory_file, self.sequence_name, self.joints)
            # Asignar la trayectoria cargada al diccionario de trayectorias, con la clave que se usa en execute_trajectory
            self.trajectories[self.sequence_name] = self.trajectory_data
            self.build_parameter_action_queue()
            self.execute_next_action()



    # ----------------------------
    # Servicios y manejo de OPC-UA
    # ----------------------------

    
    def enable_opcua_callback(self, request, response):
        """Activa o desactiva OPC-UA según su estado actual."""
        if not OPCUA_AVAILABLE:
            self.get_logger().error("No se puede habilitar/deshabilitar OPC-UA porque asyncua no está instalado.")
            return response

        if self.opcua_enabled:
            # Si estaba activo, ahora lo desactivamos
            self.get_logger().info("Deshabilitando OPC-UA...")
            try:
                # Cerrar conexión
                if self.opcua_client:
                    self.loop.run_until_complete(self.opcua_client.disconnect())
                self.opcua_client = None
                self.opcua_enabled = False
                self.get_logger().info("OPC-UA fue deshabilitado.")
            except Exception as e:
                self.get_logger().error(f"Error al deshabilitar OPC-UA: {e}")
        else:
            # Si estaba inactivo, lo habilitamos
            self.get_logger().info("Habilitando OPC-UA...")
            self.opcua_enabled = True
            self.loop = asyncio.new_event_loop()
            self.opcua_client = Client(self.get_parameter('opcua_url').value)
            try:
                self.loop.run_until_complete(self.opcua_client.connect())
                self.sensor_node = self.opcua_client.get_node(self.get_parameter('sensor_node_id').value)
                self.get_logger().info("Conexión OPC-UA exitosa.")
            except Exception as e:
                self.get_logger().error(f"Error al conectar a OPC-UA: {e}")
                self.opcua_enabled = False
        
        return response

    def pause_callback(self, request, response):
        """Alterna el estado de pausa de la ejecución."""
        self.is_paused = not self.is_paused
        if self.is_paused:
            self.get_logger().info("🚨 EJECUCIÓN PAUSADA 🚨")
        else:
            self.get_logger().info("✅ EJECUCIÓN REANUDADA ✅")
            self.execute_next_action()
        return response

    def reset_callback(self, request, response):
        """Reinicia la cola de acciones y detiene la ejecución actual."""
        self.get_logger().info("⚠️ SECUENCIA REINICIADA ⚠️")
        self.action_queue.clear()
        self.is_executing = False
        self.is_paused = False
        return response

    # ----------------------------
    # Métodos de verificación OPC-UA
    # ----------------------------

    def product_present(self):
        """Verifica la presencia del producto mediante OPC-UA."""
        try:
            value = self.loop.run_until_complete(self.sensor_node.read_value())
            self.get_logger().info(f"Valor del sensor OPC-UA: {value} (Tipo: {type(value)})")
            if isinstance(value, bool):
                return value
            else:
                self.get_logger().warn(f"El valor del sensor no es booleano: {value}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error al leer el sensor OPC-UA: {e}")
            if "BadSessionIdInvalid" in str(e):
                self.reconnect_opcua()
            return False

    def reconnect_opcua(self):
        """Intenta reconectar el cliente OPC-UA en caso de error de sesión."""
        self.get_logger().info("Intentando reconectar OPC-UA...")
        try:
            self.loop.run_until_complete(self.opcua_client.disconnect())
        except Exception as e:
            self.get_logger().warn(f"No se pudo cerrar la conexión anterior: {e}")
        try:
            self.loop.run_until_complete(self.opcua_client.connect())
            self.sensor_node = self.opcua_client.get_node(self.get_parameter('sensor_node_id').value)
            self.get_logger().info("Reconexión OPC-UA exitosa.")
        except Exception as e:
            self.get_logger().error(f"Error al reconectar OPC-UA: {e}")

    # ----------------------------
    # Manejo de pedidos y construcción de la acción
    # ----------------------------

    def order_callback(self, msg):
        """
        Procesa el pedido recibido por el tópico /confirmed_order.
        Ignora nuevos pedidos si ya se está ejecutando uno.
        """
        if self.is_executing:
            self.get_logger().warn("Nuevo pedido recibido, pero el robot aún está ejecutando el actual. Ignorando este pedido.")
            return
        
        self.is_executing = True
        self.tipo_pallet = msg.tipo_pallet
        self.cantidad_pallet = msg.cantidad_pallet
        self.current_presentation = 0
        self.current_package_count = 0

        # Cargar trayectorias específicas para el pedido
        self.load_all_trajectories()
        self.build_topic_action_queue()
        self.execute_next_action()

    def load_all_trajectories(self):
        """Carga todas las trayectorias del archivo YAML necesarias para el pedido."""
        self.trajectories.clear()
        for tipo in set(self.tipo_pallet):
            sequence_name = f"empaque_{tipo}"
            self.trajectories[sequence_name] = load_trajectories_from_yaml(self.trajectory_file, sequence_name, self.joints)
        self.get_logger().info(f"Trayectorias cargadas: {list(self.trajectories.keys())}")

    def execute_next_action(self):
        """Ejecuta la siguiente acción en la cola si no está pausada."""
        if self.is_paused:
            self.get_logger().info("Ejecución pausada. Esperando reanudación...")
            return

        if not self.action_queue:
            self.get_logger().info("Se completaron todas las acciones del pedido.")
            self.is_executing = False
            return

        self.current_action = self.action_queue.pop(0)
        action_type, *action_data = self.current_action

        if action_type in ['tapas_point_trajectory', 'pickup_point_trajectory', 'gripper']:
            if len(action_data) == 1:
                self.get_logger().info(f"Ejecutando {action_type}: {action_data[0]}")
                if action_type == 'tapas_point_trajectory':
                    self.execute_tapas_point_trajectory(action_data[0])
                elif action_type == 'pickup_point_trajectory':
                    self.execute_pickup_point_trajectory(action_data[0])
                elif action_type == 'gripper':
                    self.execute_gripper_action(action_data[0])
            else:
                self.get_logger().error(f"Error: Parámetros incorrectos en {action_type} -> {action_data}")

        else:
            # Para trayectorias principales definidas por la secuencia
            if len(action_data) == 1:
                self.get_logger().info(f"Ejecutando trayectoria principal: {action_type} - {action_data[0]}")
                self.execute_trajectory(action_type, action_data[0])
            else:
                self.get_logger().error(f"Error: Parámetros incorrectos en {action_type} -> {action_data}")

    def build_topic_action_queue(self):
        """
        Construye la cola de acciones basada en el pedido recibido.
        Incluye trayectorias de tapas, pickup, y la secuencia principal de empaquetado.
        """
        self.action_queue.clear()
        for i, tipo in enumerate(self.tipo_pallet):
            for j in range(self.cantidad_pallet[i]):
                # Secuencias fijas de tapas y pickup
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
                
                sequence_name = f"empaque_{tipo}"
                if sequence_name not in self.trajectories or not isinstance(self.trajectories[sequence_name], dict):
                    self.get_logger().error(f"Error: '{sequence_name}' no está disponible o tiene formato incorrecto.")
                    return

                trajectory_names = list(self.trajectories[sequence_name].keys())
                trajectory_names.sort()  # Asegura orden traj0, traj1, traj2, ...

                if len(trajectory_names) % 2 != 0:
                    self.get_logger().error(f"La secuencia '{sequence_name}' debe tener un número par de trayectorias (pares elevación-descenso).")
                    return

                num_pairs = len(trajectory_names) // 2

                for idx in range(0, len(trajectory_names), 2):
                    traj_elevated = trajectory_names[idx]
                    traj_descended = trajectory_names[idx + 1]

                    # 1. Subir al punto elevado
                    self.action_queue.append((sequence_name, traj_elevated))

                    # 2. Bajar con interpolación (moveL)
                    self.action_queue.append((sequence_name, traj_descended))

                    # 3. Acción del gripper si aplica (del punto descendido)
                    for pt in self.trajectories[sequence_name][traj_descended]:
                        if pt['gripper_action'] != 'none':
                            self.action_queue.append(('gripper', pt['gripper_action']))
                    
                    # 4. Subida vertical nuevamente usando traj0 pero como moveL
                    # Creamos una copia del punto elevado pero con movement_type forzado a moveL
                    elevated_points = copy.deepcopy(self.trajectories[sequence_name][traj_elevated])
                    for pt in elevated_points:
                        pt["movement_type"] = "moveL"
                    self.trajectories[sequence_name][f"{traj_elevated}_ascenso"] = elevated_points
                    self.action_queue.append((sequence_name, f"{traj_elevated}_ascenso"))


                    # 5. Intercalar tapas y pickup si no es la última pareja
                    if idx < len(trajectory_names) - 2:
                        for tapas_traj_name, tapas_points in self.tapas_point_data.items():
                            self.action_queue.append(('tapas_point_trajectory', tapas_traj_name))
                            for pt in tapas_points:
                                if pt['gripper_action'] != 'none':
                                    self.action_queue.append(('gripper', pt['gripper_action']))

                        for pickup_traj_name, pickup_points in self.pickup_point_data.items():
                            self.action_queue.append(('pickup_point_trajectory', pickup_traj_name))
                            for pt in pickup_points:
                                if pt['gripper_action'] != 'none':
                                    self.action_queue.append(('gripper', pt['gripper_action']))
                                    

        self.get_logger().info(f"Cola de acciones generada con {len(self.action_queue)} acciones.")

    def build_parameter_action_queue(self):
        """
        Construye la cola de acciones cuando se usa el modo por parámetros,
        replicando la lógica del modo por tópico.
        Se asume que:
        - self.tapas_point_data y self.pickup_point_data ya fueron cargadas.
        - self.trajectory_data se cargó previamente usando:
            self.trajectory_data = load_trajectories_from_yaml(self.trajectory_file, self.sequence_name, self.joints)
        """
        self.action_queue.clear()

        if not self.only_sequence_points:
        
            # Secuencias fijas iniciales de tapas y pickup
            for traj_name, points in self.tapas_point_data.items():
                self.action_queue.append(('tapas_point_trajectory', traj_name))
                for pt in points:
                    if pt.get('gripper_action', 'none') != 'none':
                        self.action_queue.append(('gripper', pt['gripper_action']))
            for traj_name, points in self.pickup_point_data.items():
                self.action_queue.append(('pickup_point_trajectory', traj_name))
                for pt in points:
                    if pt.get('gripper_action', 'none') != 'none':
                        self.action_queue.append(('gripper', pt['gripper_action']))
        
        # Procesar la secuencia principal (se asume que la secuencia tiene un número par de trayectorias: pares elevación-descenso)
        trajectory_names = list(self.trajectory_data.keys())
        trajectory_names.sort()  # Se espera que los keys sean algo como traj0, traj1, ...

        if len(trajectory_names) % 2 != 0:
            self.get_logger().error(f"La secuencia '{self.sequence_name}' debe tener un número par de trayectorias (pares elevación-descenso).")
            return

        for idx in range(0, len(trajectory_names), 2):
            traj_elevated = trajectory_names[idx]
            traj_descended = trajectory_names[idx + 1]

            # 1. Añadir la trayectoria de subida al punto elevado
            self.action_queue.append((self.sequence_name, traj_elevated))

            # 2. Añadir la trayectoria de bajada con interpolación
            self.action_queue.append((self.sequence_name, traj_descended))

            # 3. Agregar las acciones de gripper definidas en la trayectoria de descenso
            for pt in self.trajectory_data[traj_descended]:
                if pt.get('gripper_action', 'none') != 'none':
                    self.action_queue.append(('gripper', pt['gripper_action']))

            # 4. Crear la "subida vertical" utilizando los mismos puntos del elevado pero forzando moveL
            elevated_points = copy.deepcopy(self.trajectory_data[traj_elevated])
            for pt in elevated_points:
                pt["movement_type"] = "moveL"
            nuevo_key = f"{traj_elevated}_ascenso"
            self.trajectory_data[nuevo_key] = elevated_points
            self.action_queue.append((self.sequence_name, nuevo_key))

            # 5. Intercalar secuencias fijas de tapas y pickup si no es la última pareja
            if (not self.only_sequence_points) and (idx < len(trajectory_names) - 2):
                for traj_name, points in self.tapas_point_data.items():
                    self.action_queue.append(('tapas_point_trajectory', traj_name))
                    for pt in points:
                        if pt.get('gripper_action', 'none') != 'none':
                            self.action_queue.append(('gripper', pt['gripper_action']))
                for traj_name, points in self.pickup_point_data.items():
                    self.action_queue.append(('pickup_point_trajectory', traj_name))
                    for pt in points:
                        if pt.get('gripper_action', 'none') != 'none':
                            self.action_queue.append(('gripper', pt['gripper_action']))

        self.get_logger().info(f"Cola de acciones generada con {len(self.action_queue)} acciones en modo parámetro.")

    
    # ----------------------------
    # Ejecución de trayectorias y acciones
    # ----------------------------
       
    def build_trajectory_points(self, raw_points):
        """
        Dado un conjunto de puntos de trayectoria con tipo de movimiento, 
        genera los JointTrajectoryPoints a enviar (con interpolación si es necesario).
        """
        trajectory = []

        for pt in raw_points:
            movement_type = pt["movement_type"]
            target_position = pt["positions"]
            velocities = pt["velocities"]
            time_stamp = pt["time_from_start"]

            if movement_type == "moveL":
                if self.last_executed_position is None:
                    self.get_logger().warn("No hay posición anterior para interpolar, usando moveJ por defecto.")
                    jt_point = JointTrajectoryPoint(
                        positions=target_position,
                        velocities=velocities,
                        time_from_start=time_stamp
                    )
                    trajectory.append(jt_point)
                else:
                    # Interpolar entre la última y el punto actual
                    num_steps = pt.get("num_steps", 5)
                    time_step = pt.get("time_step", 2.0)
                    interpolated = self.interpolate_points(self.last_executed_position, target_position, num_steps, time_step)
                    trajectory.extend(interpolated)
            else:
                # Movimiento normal
                jt_point = JointTrajectoryPoint(
                    positions=target_position,
                    velocities=velocities,
                    time_from_start=time_stamp
                )
                trajectory.append(jt_point)

            # Actualizamos la última posición para el próximo ciclo
            self.last_executed_position = target_position
            self.get_logger().info(f"Tipo de movimiento: {movement_type} - Generando {len(trajectory)} puntos")


        return trajectory
    
    def interpolate_points(self, start, end, num_steps=5, time_step=2.0):
        """
        Genera puntos intermedios entre dos configuraciones articulares.
        """
        points = []
        for i in range(1, num_steps + 1):
            alpha = i / num_steps
            interp_position = [(1 - alpha) * s + alpha * e for s, e in zip(start, end)]
            point = JointTrajectoryPoint()
            point.positions = interp_position
            point.velocities = [0.0] * len(interp_position)
            point.time_from_start = Duration(
                sec=int(i * time_step),
                nanosec=int((i * time_step % 1) * 1e9)
            )
            points.append(point)
        return points
    

    # ----------------------------
    # Ejecución de trayectorias y acciones
    # ----------------------------

    def execute_tapas_point_trajectory(self, traj_name):
        """Ejecuta la trayectoria 'tapas_point' tras verificar la presencia del producto."""
        if self.opcua_enabled:
            while not self.product_present():
                self.get_logger().info("No hay producto presente. Esperando...")
                time.sleep(1)
        else:
            self.get_logger().warn("OPC-UA deshabilitado. Ejecutando trayectoria sin verificación de sensor.")
        self.get_logger().info(f"Producto detectado. Ejecutando trayectoria de tapas: {traj_name}")
        trajectory_points = self.tapas_point_data[traj_name]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=self.joints,
            points=self.build_trajectory_points(trajectory_points)
        )
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._tapas_point_goal_response_callback)

    def _tapas_point_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("La trayectoria de 'tapas_point' fue rechazada.")
            self.shutdown()
            return
        self.get_logger().info("Trayectoria de 'tapas_point' aceptada. Esperando resultado...")
        goal_handle.get_result_async().add_done_callback(self._get_result_callback)

    def execute_pickup_point_trajectory(self, traj_name):
        """Ejecuta la trayectoria 'pickup_point' tras verificar la presencia del producto."""
        if self.opcua_enabled:
            while not self.product_present():
                self.get_logger().info("No hay producto presente. Esperando...")
                time.sleep(1)
        else:
            self.get_logger().warn("OPC-UA deshabilitado. Ejecutando trayectoria sin verificación de sensor.")
        self.get_logger().info(f"Producto detectado. Ejecutando trayectoria fija: {traj_name}")
        trajectory_points = self.pickup_point_data[traj_name]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=self.joints,
            points=self.build_trajectory_points(trajectory_points)          
        )
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._pickup_point_goal_response_callback)

    def _pickup_point_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("La trayectoria de 'pickup_point' fue rechazada.")
            self.shutdown()
            return
        self.get_logger().info("Trayectoria de 'pickup_point' aceptada. Esperando resultado...")
        goal_handle.get_result_async().add_done_callback(self._get_result_callback)

    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("La trayectoria fue rechazada.")
            self.shutdown()
            return
        self.get_logger().info("Trayectoria aceptada. Esperando resultado...")
        goal_handle.get_result_async().add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result  # Ahora future.result() es el resultado final
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trayectoria completada con éxito.")
            time.sleep(self.delay_between_trajectories)
            self.execute_next_action()
        else:
            self.get_logger().error(f"Error en la trayectoria: {result.error_code}")
            self.shutdown()
    from builtin_interfaces.msg import Duration

        
    def execute_trajectory(self, sequence_name, traj_name):
        """Ejecuta la trayectoria principal definida en la secuencia."""
        self.get_logger().info(f"Ejecutando trayectoria {traj_name} de {sequence_name}")
        trajectory_points = self.trajectories[sequence_name][traj_name]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=self.joints,
            points=self.build_trajectory_points(trajectory_points)
        )
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def execute_gripper_action(self, action):
        """Ejecuta la acción del gripper enviando el comando adecuado a través del servicio SetIO."""
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
        """Finaliza el nodo de ROS y cierra la ejecución."""
        self.get_logger().info("Finalizando nodo.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGripperController()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
