import sys
import os
import signal
import subprocess
import threading
import yaml
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QWidget, QLineEdit,
    QTabWidget, QComboBox, QCheckBox, QFormLayout, QMessageBox, QHBoxLayout, QGridLayout, QGroupBox,
    QInputDialog
)
from PyQt5.QtGui import QPalette, QColor, QFont
from PyQt5.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from custom_msgs.msg import OrderStatus
from std_srvs.srv import Trigger  # Importar el servicio Trigger
from std_srvs.srv import Empty

class RobotControlGUI(QMainWindow):
    """Interfaz gráfica de control del UR3, incluyendo el monitoreo de órdenes."""

    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node

        self.setWindowTitle("Control del Robot UR3")
        self.setGeometry(100, 100, 600, 500)
        self.apply_styles()

        # Widget principal con pestañas
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # Crear pestañas
        self.create_driver_tab()
        self.create_point_capture_tab()
        self.create_packaging_sequences_tab()
        self.create_order_tab()

        # Temporizador para actualizar la interfaz cada 500ms
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(500)  # Actualiza cada 500ms

        # Inicializar datos
        self.latest_msg = None

        # Proceso para el driver
        self.driver_process = None 
        
        # Flag para saber si estamos en pausa dentro de la pestaña "Orden de Pedido"
        self.seq_is_paused = False
    
    def closeEvent(self, event):
        """Al cerrar la GUI, terminamos todos los procesos lanzados."""
        procs = [
            getattr(self, 'driver_process', None),
            getattr(self, 'capture_process', None),
            getattr(self, 'sequence_process', None),
            getattr(self, 'monitor_process', None),
            getattr(self, 'trajectory_process', None),
        ]
        for proc in procs:
            if proc and proc.poll() is None:  # si sigue vivo
                try:
                    # matamos todo el grupo de procesos
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=5)
                except Exception:
                    pass
        # ahora sí cerramos la ventana
        super().closeEvent(event)

    def update_order_status(self, msg):
        """Callback para recibir mensajes y almacenarlos para actualización."""
        self.latest_msg = msg

    def update_gui(self):
        """Actualiza la GUI con los últimos datos recibidos."""
        if self.latest_msg:
            self.product_type_label.setText(f"Tipo de Producto: {self.latest_msg.tipo_producto}")
            self.presentation_label.setText(f"Presentación: {', '.join(map(str, self.latest_msg.presentacion))}")
            #self.quantity_label.setText(f"Cantidad por Presentación: {', '.join(map(str, self.latest_msg.cantidad_presentacion))}")
            self.order_quantity_label.setText(f"Cantidad de Pedido: {self.latest_msg.cantidad_pedido:.2f}")

            # Actualizar valores separados de cantidad por presentación
            cantidades = self.latest_msg.cantidad_presentacion  # Lista de valores
            self.quantity_1_label.setText(f"Presentación 1: {cantidades[0]}" if len(cantidades) > 0 else "Presentación 1: -")
            self.quantity_2_label.setText(f"Presentación 2: {cantidades[1]}" if len(cantidades) > 1 else "Presentación 2: -")
            self.quantity_3_label.setText(f"Presentación 3: {cantidades[2]}" if len(cantidades) > 2 else "Presentación 3: -")
        
        # Actualizar etiquetas de tipo de pallet y cantidad de pallet
            for i in range(3):
                self.pallet_type_labels[i].setText(f"Tipo de Pallet para Presentación {i+1}: {self.latest_msg.tipo_pallet[i]}" 
                                                if len(self.latest_msg.tipo_pallet) > i else f"Tipo de Pallet para Presentación {i+1}: -")
                
                self.pallet_quantity_labels[i].setText(f"Cantidad de Pallet para Presentación {i+1}: {self.latest_msg.cantidad_pallet[i]}" 
                                                    if len(self.latest_msg.cantidad_pallet) > i else f"Cantidad de Pallet para Presentación {i+1}: -")

            """  # 🏷️ Actualizar datos de pallets de forma separada
            pallet_types = self.latest_msg.tipo_pallet  # Lista de tipos de pallet
            self.pallet_1_type_label.setText(f"Tipo de Pallet para Presentación 1: {pallet_types[0]}" if len(pallet_types) > 0 else "Tipo de Pallet para Presentación 1: -")
            self.pallet_2_type_label.setText(f"Tipo de Pallet para Presentación 2: {pallet_types[1]}" if len(pallet_types) > 1 else "Tipo de Pallet para Presentación 2: -")
            self.pallet_3_type_label.setText(f"Tipo de Pallet para Presentación 3: {pallet_types[2]}" if len(pallet_types) > 2 else "Tipo de Pallet para Presentación 3: -")

            # Actualizar valores separados de cantidad de pallets
            pallets = self.latest_msg.cantidad_pallet  # Lista de valores
            self.pallet_1_label.setText(f"Pallet para Presentación 1: {pallets[0]}" if len(pallets) > 0 else "Pallet para Presentación 1: -")
            self.pallet_2_label.setText(f"Pallet para Presentación 2: {pallets[1]}" if len(pallets) > 1 else "Pallet para Presentación 2: -")
            self.pallet_3_label.setText(f"Pallet para Presentación 3: {pallets[2]}" if len(pallets) > 2 else "Pallet para Presentación 3: -") """


    def apply_styles(self):
        """Aplica el diseño visual general."""
        QApplication.setStyle("Fusion")  # 🔹 Cambio aquí (antes usaba self.app)

        # Paleta de colores personalizada
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(240, 240, 240))
        palette.setColor(QPalette.WindowText, Qt.black)
        palette.setColor(QPalette.Base, QColor(255, 255, 255))
        palette.setColor(QPalette.AlternateBase, QColor(220, 220, 220))
        palette.setColor(QPalette.Text, Qt.black)
        palette.setColor(QPalette.Button, QColor(200, 200, 200))
        palette.setColor(QPalette.ButtonText, Qt.black)
        palette.setColor(QPalette.Highlight, QColor(0, 122, 204))
        palette.setColor(QPalette.HighlightedText, Qt.white)
        
        QApplication.setPalette(palette)  # 🔹 Cambio aquí (antes usaba self.app)

        # Fuente global
        font = QFont("Arial", 10)
        QApplication.setFont(font)  # 🔹 Cambio aquí también

    
    def get_button_style(self):
        """Devuelve el estilo CSS para los botones."""
        return """
            QPushButton {
                background-color: #007ACC;
                color: white;
                border-radius: 8px;
                padding: 8px 16px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #005A9E;
            }
            QPushButton:pressed {
                background-color: #004785;
            }
        """
    

    def create_driver_tab(self):
        """Crea la pestaña para controlar el driver."""
        driver_tab = QWidget()
        layout = QVBoxLayout(driver_tab)

        # Etiqueta de estado
        self.driver_status_label = QLabel("Estado del Driver: No iniciado")
        self.driver_status_label.setAlignment(Qt.AlignCenter)
        self.driver_status_label.setStyleSheet("font-size: 16px; color: blue; font-weight: bold;")
        layout.addWidget(self.driver_status_label)
        
        # IP del robot
        ip_label = QLabel("Robot IP:")
        ip_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(ip_label)

        self.robot_ip_input = QLineEdit("192.168.56.101 ")
        self.robot_ip_input.setPlaceholderText("e.g. 192.168.56.101 ")
        self.robot_ip_input.setStyleSheet("padding: 4px;")
        layout.addWidget(self.robot_ip_input)

        # Lanzar RViz
        self.rviz_checkbox = QCheckBox("Launch RViz")
        self.rviz_checkbox.setChecked(True)
        layout.addWidget(self.rviz_checkbox)

        # Ruta kinematics_params_file 
        kin_label = QLabel("Kinematics Params File:")
        kin_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(kin_label)

        self.kinematics_input = QLineEdit(os.path.expanduser("~/my_robot_calibration.yaml"))
        self.kinematics_input.setPlaceholderText("/ruta/a/calibration.yaml")
        self.kinematics_input.setStyleSheet("padding: 4px;")
        layout.addWidget(self.kinematics_input)

        # Botón para iniciar el driver
        start_button = QPushButton("Lanzar Driver")
        start_button.setStyleSheet(self.get_button_style())
        start_button.clicked.connect(self.start_driver)
        layout.addWidget(start_button)

        # Botón para detener el driver
        stop_button = QPushButton("Detener Driver")
        stop_button.setStyleSheet(self.get_button_style())
        stop_button.clicked.connect(self.stop_driver)
        layout.addWidget(stop_button)

        # Botón para iniciar el programa en el robot
        run_program_button = QPushButton("Iniciar Programa en el Robot")
        run_program_button.setStyleSheet(self.get_button_style())
        run_program_button.clicked.connect(self.run_robot_program)
        layout.addWidget(run_program_button)

        layout.addStretch()
        self.tabs.addTab(driver_tab, "Control del Driver")

    def create_point_capture_tab(self):
        """Crea la pestaña para capturar puntos."""
        capture_tab = QWidget()
        layout = QVBoxLayout(capture_tab)

        # Selección de la secuencia
        sequence_label = QLabel("Seleccionar Secuencia:")
        sequence_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(sequence_label)

        self.point_capture_sequence_selector = QComboBox()


        # Cargar dinámicamente las secuencias del YAML
        self.point_capture_sequence_selector.addItems(self.load_sequences())
        self.point_capture_sequence_selector.setStyleSheet("padding: 4px;")
        layout.addWidget(self.point_capture_sequence_selector)

        # Botones para crear y eliminar secuencias
        button_layout = QHBoxLayout()
        create_seq_button = QPushButton("Crear Secuencia")
        create_seq_button.setStyleSheet(self.get_button_style())
        create_seq_button.clicked.connect(self.create_new_sequence)
        button_layout.addWidget(create_seq_button)
        
        delete_seq_button = QPushButton("Eliminar Secuencia")
        delete_seq_button.setStyleSheet(self.get_button_style())
        delete_seq_button.clicked.connect(self.delete_sequence)
        button_layout.addWidget(delete_seq_button)
        
        layout.addLayout(button_layout)

        # Botón centrado para Actualizar Secuencias
        update_seq_button = QPushButton("Actualizar Secuencias")
        update_seq_button.setStyleSheet(self.get_button_style())
        update_seq_button.clicked.connect(self.refresh_sequence_comboboxes)
        # Se agrega al layout centrado horizontalmente
        layout.addWidget(update_seq_button, alignment=Qt.AlignHCenter)

        # Acción del gripper
        gripper_label = QLabel("Seleccionar Acción del Gripper:")
        gripper_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(gripper_label)

        self.gripper_selector = QComboBox()
        self.gripper_selector.addItems(["none", "open", "close"])
        self.gripper_selector.setStyleSheet("padding: 4px;")
        layout.addWidget(self.gripper_selector)

        # Tipo de movimiento
        movement_label = QLabel("Tipo de Movimiento:")
        movement_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(movement_label)

        self.movement_type_selector = QComboBox()
        self.movement_type_selector.addItems(["moveJ", "moveL"])
        self.movement_type_selector.setStyleSheet("padding: 4px;")
        layout.addWidget(self.movement_type_selector)

        # Número de puntos para interpolación (solo si moveL)
        self.num_steps_label = QLabel("Número de Puntos Interpolados:")
        self.num_steps_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(self.num_steps_label)

        self.num_steps_input = QLineEdit("5")  # Valor predeterminado
        self.num_steps_input.setPlaceholderText("Ejemplo: 5, 10, 20...")
        self.num_steps_input.setStyleSheet("padding: 4px;")
        layout.addWidget(self.num_steps_input)

        # Tiempo entre puntos interpolados
        self.interp_time_label = QLabel("Tiempo entre Puntos Interpolados (segundos):")
        self.interp_time_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(self.interp_time_label)

        self.interp_time_input = QLineEdit("2.0")  # Valor predeterminado
        self.interp_time_input.setPlaceholderText("Ejemplo: 1.0, 1.5, 2.0...")
        self.interp_time_input.setStyleSheet("padding: 4px;")
        layout.addWidget(self.interp_time_input)
    
        # Entrada de usuario para el time_step
        time_step_label = QLabel("Tiempo entre Puntos (segundos):")
        time_step_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(time_step_label)

        self.time_step_input = QLineEdit("5.0")  # Valor predeterminado de 5.0 segundos
        self.time_step_input.setPlaceholderText("Ingrese el tiempo en segundos")
        self.time_step_input.setStyleSheet("padding: 4px;")
        layout.addWidget(self.time_step_input)

        # Botón para limpiar la secuencia
        clear_button = QPushButton("Limpiar Secuencia")
        clear_button.setStyleSheet(self.get_button_style())
        clear_button.clicked.connect(self.clear_sequence)
        layout.addWidget(clear_button)

        # Botón para capturar un punto
        capture_button = QPushButton("Capturar Punto")
        capture_button.setStyleSheet(self.get_button_style())
        capture_button.clicked.connect(self.capture_point)
        layout.addWidget(capture_button)    

        # Etiqueta de estado de la captura
        self.capture_status_label = QLabel("Estado: En espera")
        self.capture_status_label.setAlignment(Qt.AlignCenter)
        self.capture_status_label.setStyleSheet("font-size: 16px; color: blue; font-weight: bold;")
        layout.addWidget(self.capture_status_label)

        layout.addStretch()
        self.tabs.addTab(capture_tab, "Captura de Puntos")

         # Conectar el cambio de tipo de movimiento a la función que muestra/oculta las opciones
        self.movement_type_selector.currentIndexChanged.connect(self.toggle_interpolation_fields)

        # Ocultar inicialmente si NO es moveL
        self.toggle_interpolation_fields()
    
    def toggle_interpolation_fields(self):
        """Muestra u oculta los campos de interpolación dependiendo del tipo de movimiento."""
        current_movement = self.movement_type_selector.currentText()
        if current_movement == "moveL":
            self.num_steps_label.show()
            self.num_steps_input.show()
            self.interp_time_label.show()
            self.interp_time_input.show()
        else:
            self.num_steps_label.hide()
            self.num_steps_input.hide()
            self.interp_time_label.hide()
            self.interp_time_input.hide()
    
    
    def create_packaging_sequences_tab(self):
        """Crea la pestaña para manejar secuencias de empaquetado."""
        sequences_tab = QWidget()
        layout = QVBoxLayout(sequences_tab)

        # Formulario para los parámetros
        form_layout = QFormLayout()

        self.opcua_url_input = QLineEdit("opc.tcp://192.168.0.230:4840")
        form_layout.addRow("OPCUA URL:", self.opcua_url_input)

        self.sensor_node_id_input = QLineEdit("ns=4;i=2")
        form_layout.addRow("Sensor Node ID:", self.sensor_node_id_input)

        self.delay_input = QLineEdit("0.3")
        form_layout.addRow("Delay entre trayectorias:", self.delay_input)

        # Seleccionar Secuencia (e.g. empaque_4)
        self.packaging_sequence_selector = QComboBox()
        self.packaging_sequence_selector.addItems(["empaque_2", "empaque_4", "empaque_6"])
        form_layout.addRow("Seleccionar Secuencia:", self.packaging_sequence_selector)
        
        # Modo de ejecución
        self.execution_mode_selector = QComboBox()
        self.execution_mode_selector.addItems(["Completo", "Parcial"])
        form_layout.addRow("Modo de Ejecución:", self.execution_mode_selector)

        layout.addLayout(form_layout)

        # Etiqueta de estado de la secuencia
        self.sequence_status_label = QLabel("Estado: En espera")
        self.sequence_status_label.setAlignment(Qt.AlignCenter)
        self.sequence_status_label.setStyleSheet("font-size: 16px; color: blue; font-weight: bold;")
        layout.addWidget(self.sequence_status_label)
        
        # Bandera para saber si estamos en pausa o no
        self.packaging_paused = False

        # Botón para iniciar la secuencia
        start_button = QPushButton("Iniciar Secuencia")
        start_button.setStyleSheet(self.get_button_style())
        start_button.clicked.connect(self.start_sequence_wrapper)
        layout.addWidget(start_button)
        
        # Botón para detener la secuencia
        stop_button = QPushButton("Detener Nodo")
        stop_button.setStyleSheet(self.get_button_style())
        stop_button.clicked.connect(self.stop_sequence)
        layout.addWidget(stop_button)

        # --- Botón para activar OPC-UA ---
        enable_opcua_button = QPushButton("Activar OPC-UA")
        enable_opcua_button.setStyleSheet(self.get_button_style())
        enable_opcua_button.clicked.connect(self.enable_opcua_service)
        layout.addWidget(enable_opcua_button)
        
        # --- Botón para Pausa/Reanudar ---
        self.pause_resume_button = QPushButton("⏸ Pausa/Reanudar")
        self.pause_resume_button.setStyleSheet(self.get_button_style())
        self.pause_resume_button.clicked.connect(self.send_pause_request_secuence)
        layout.addWidget(self.pause_resume_button)

        layout.addStretch()
        self.tabs.addTab(sequences_tab, "Secuencias de Empaquetado")

    def create_order_tab(self):
        """Crea la pestaña para monitorear y gestionar órdenes de pedido."""
        order_tab = QWidget()
        layout = QGridLayout(order_tab)  # Usamos QGridLayout para una mejor organización

        # 🏷️ Etiqueta principal
        title_label = QLabel("📦 Orden de Pedido")
        title_label.setStyleSheet("font-size: 18px; font-weight: bold; color: navy;")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label, 0, 0, 1, 3)

        # 📊 Datos de la orden
        self.product_type_label = QLabel("Tipo de Producto: -")
        self.presentation_label = QLabel("Presentación: -")
        self.order_quantity_label = QLabel("Cantidad de Pedido: -")

        labels = [self.product_type_label, self.presentation_label, self.order_quantity_label]
        for label in labels:
            label.setStyleSheet("font-size: 14px; font-weight: bold;")
            label.setAlignment(Qt.AlignCenter)

        layout.addWidget(self.product_type_label, 1, 0, 1, 3)
        layout.addWidget(self.presentation_label, 2, 0, 1, 3)
        layout.addWidget(self.order_quantity_label, 3, 0, 1, 3)

        # 🔢 Sección para mostrar las cantidades por presentación
        quantity_group = QGroupBox("📊 Cantidad por Presentación")
        quantity_layout = QVBoxLayout()

        self.quantity_1_label = QLabel("Presentación 1: -")
        self.quantity_2_label = QLabel("Presentación 2: -")
        self.quantity_3_label = QLabel("Presentación 3: -")

        for label in [self.quantity_1_label, self.quantity_2_label, self.quantity_3_label]:
            label.setStyleSheet("font-size: 14px; font-weight: bold;")
            label.setAlignment(Qt.AlignCenter)
            quantity_layout.addWidget(label)

        quantity_group.setLayout(quantity_layout)
        layout.addWidget(quantity_group, 4, 0, 1, 3)


        # 🔢 Sección para mostrar el pedido de empaques
        package_group = QGroupBox("📊 Tipo de empaquetado")
        package_layout = QVBoxLayout()

        # Crear listas para almacenar etiquetas dinámicas
        self.pallet_type_labels = []
        self.pallet_quantity_labels = []

        for i in range(3):  # Tres presentaciones
            row_layout = QHBoxLayout()  # Para colocar elementos en una sola fila

            # Tipo de Pallet
            pallet_type_label = QLabel(f"Tipo de Pallet para Presentación {i+1}: -")
            pallet_type_label.setStyleSheet("font-size: 14px; font-weight: bold;")
            row_layout.addWidget(pallet_type_label)

            # Cantidad de Pallet
            pallet_quantity_label = QLabel(f"Cantidad de Pallet para Presentación {i+1}: -")
            pallet_quantity_label.setStyleSheet("font-size: 14px; font-weight: bold;")
            row_layout.addWidget(pallet_quantity_label)

            # Agregar referencias a listas
            self.pallet_type_labels.append(pallet_type_label)
            self.pallet_quantity_labels.append(pallet_quantity_label)

            # Agregar la fila completa al layout vertical
            package_layout.addLayout(row_layout)

        package_group.setLayout(package_layout)
        layout.addWidget(package_group, 5, 0, 1, 3)

        # 🔽 Botón para mostrar/ocultar la configuración OPC-UA
        self.toggle_opcua_button = QPushButton("🔽 Mostrar Configuración OPC-UA")
        self.toggle_opcua_button.setCheckable(True)  # Permitir estado activo/inactivo
        self.toggle_opcua_button.setStyleSheet("font-size: 12px; padding: 5px;")
        self.toggle_opcua_button.toggled.connect(self.toggle_opcua_visibility)
        layout.addWidget(self.toggle_opcua_button, 6, 0, 1, 3)

        # ⚙️ Configuración OPC-UA (Oculta por defecto)
        self.opcua_group = QGroupBox("⚙️ Configurar Conexión OPC-UA")
        opcua_layout = QGridLayout()

        self.opcua_url_input = QLineEdit("opc.tcp://192.168.0.230:4840")
        opcua_layout.addWidget(QLabel("📡 OPC-UA URL:"), 0, 0)
        opcua_layout.addWidget(self.opcua_url_input, 0, 1, 1, 2)

        self.ns_input = QLineEdit("4")
        opcua_layout.addWidget(QLabel("🔢 Espacio de nombres (ns):"), 1, 0)
        opcua_layout.addWidget(self.ns_input, 1, 1, 1, 2)

        # 🔢 Índices de Variables OPC-UA dentro del panel oculto
        self.tipo_producto_i_input = QLineEdit("4")
        opcua_layout.addWidget(QLabel("📦 Índice 'i' de Tipo de Producto:"), 2, 0)
        opcua_layout.addWidget(self.tipo_producto_i_input, 2, 1, 1, 2)

        self.presentacion_i_input = QLineEdit("6")
        opcua_layout.addWidget(QLabel("🎁 Índice 'i' de Presentación:"), 3, 0)
        opcua_layout.addWidget(self.presentacion_i_input, 3, 1, 1, 2)

        self.cantidad_presentacion_i_input = QLineEdit("10")
        opcua_layout.addWidget(QLabel("📊 Índice 'i' de Cantidad por Presentación:"), 4, 0)
        opcua_layout.addWidget(self.cantidad_presentacion_i_input, 4, 1, 1, 2)

        self.cantidad_pedido_i_input = QLineEdit("5")
        opcua_layout.addWidget(QLabel("📑 Índice 'i' de Cantidad de Pedido:"), 5, 0)
        opcua_layout.addWidget(self.cantidad_pedido_i_input, 5, 1, 1, 2)

        self.tipo_pallet_i_input = QLineEdit("7")
        opcua_layout.addWidget(QLabel("📑 Índice 'i' de Tipo de Pallet:"), 6, 0)
        opcua_layout.addWidget(self.tipo_pallet_i_input, 6, 1, 1, 2)

        self.cantidad_pallet_i_input = QLineEdit("8")
        opcua_layout.addWidget(QLabel("📑 Índice 'i' de Cantidad de Pallets:"), 7, 0)
        opcua_layout.addWidget(self.cantidad_pallet_i_input, 7, 1, 1, 2)

        self.opcua_group.setLayout(opcua_layout)
        layout.addWidget(self.opcua_group, 7, 0, 1, 3)
        self.opcua_group.setVisible(False)  # Ocultar al inicio

        # 🟢 Estado del monitoreo
        self.monitor_status_label = QLabel("Estado del Monitoreo: No iniciado")
        self.monitor_status_label.setStyleSheet("font-size: 16px; color: blue; font-weight: bold;")
        self.monitor_status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.monitor_status_label, 8, 0, 1, 3)

        # 🔘 Botones de monitoreo
        start_monitor_button = QPushButton("Iniciar Nodo de Monitoreo")
        stop_monitor_button = QPushButton("Detener Nodo de Monitoreo")

        start_monitor_button.setStyleSheet(self.get_button_style())
        stop_monitor_button.setStyleSheet(self.get_button_style())

        start_monitor_button.clicked.connect(self.start_order_monitor)
        stop_monitor_button.clicked.connect(self.stop_order_monitor)

        layout.addWidget(start_monitor_button, 9, 0)
        layout.addWidget(stop_monitor_button, 9, 1, 1, 2)


        #  Botón para mostrar/ocultar la configuración de empaque
        self.packaging_toggle_button = QPushButton("⚙️ Mostrar Configuración de Empaquetado")
        self.packaging_toggle_button.setCheckable(True)
        #self.packaging_toggle_button.setChecked(False)
        self.packaging_toggle_button.setStyleSheet("font-size: 12px; padding: 5px;")
        self.packaging_toggle_button.toggled.connect(self.toggle_packaging_visibility)
        layout.addWidget(self.packaging_toggle_button, 10, 0, 1, 3)


        # 📦 Grupo para Configuración del Nodo de Trayectorias
        self.trajectory_node_group = QGroupBox("📦 Configuración de Empaque")
        self.trajectory_node_group_layout = QVBoxLayout()

        # Contenedor para empaques
        self.trajectory_node_content = QWidget()
        trajectory_node_content_layout = QVBoxLayout(self.trajectory_node_group)

        # 🔘 Layout horizontal para los botones de pausa y reiniciar
        node_button_layout = QHBoxLayout()

        # Estado del empaquetado
        self.package_status_label= QLabel("Estado del Robot: Esperando activacion del nodo")
        self.package_status_label.setStyleSheet("font-size: 16px; color: blue; font-weight: bold;")
        self.package_status_label.setAlignment(Qt.AlignCenter)
        trajectory_node_content_layout.addWidget(self.package_status_label)

        # 🔘 Botones para iniciar/detener el nodo de trayectorias
        start_trajectory_button = QPushButton("Iniciar Nodo de Trayectorias")
        start_trajectory_button.setStyleSheet(self.get_button_style())
        start_trajectory_button.clicked.connect(self.start_trajectory_node)
        node_button_layout.addWidget(start_trajectory_button)

        stop_trajectory_button = QPushButton("Detener Nodo de Trayectorias")
        stop_trajectory_button.setStyleSheet(self.get_button_style())
        stop_trajectory_button.clicked.connect(self.stop_trajectory_node)
        node_button_layout.addWidget(stop_trajectory_button)

        # ⏸ Botón de Pausa/Reanudar
        pause_resume_button = QPushButton("⏸ Pausa/Reanudar")
        pause_resume_button.setStyleSheet(self.get_button_style())
        pause_resume_button.clicked.connect(self.send_pause_request)
        node_button_layout.addWidget(pause_resume_button)

        # 🔄 Botón de Reiniciar
        reset_button = QPushButton("🔄 Reiniciar")
        reset_button.setStyleSheet(self.get_button_style())
        reset_button.clicked.connect(self.send_reset_request)
        node_button_layout.addWidget(reset_button)

        # Nuevo botón para habilitar OPC-UA, centrado
        enable_opcua_button = QPushButton("Activar OPC-UA")
        enable_opcua_button.setStyleSheet(self.get_button_style())
        enable_opcua_button.clicked.connect(self.enable_opcua_service)
        node_button_layout.addWidget(enable_opcua_button, alignment=Qt.AlignHCenter)


        # Agregar el layout de botones al contenido principal
        trajectory_node_content_layout.addLayout(node_button_layout)

        self.trajectory_node_content.setLayout(trajectory_node_content_layout)
        self.trajectory_node_group_layout.addWidget(self.trajectory_node_content)

        # Inicialmente ocultar el contenido
        self.trajectory_node_group.setVisible(False)

        self.trajectory_node_group.setLayout(self.trajectory_node_group_layout)
        layout.addWidget(self.trajectory_node_group, 11, 0, 1, 3)


        # 📦 Grupo para Configuración de Empaque Automático
        self.auto_packaging_group = QGroupBox("📦 Configuración de Empaque Automático")
        self.auto_packaging_group_layout = QVBoxLayout()

        # Contenedor para empaques
        self.auto_packaging_content = QWidget()
        auto_packaging_content_layout = QVBoxLayout(self.auto_packaging_content)

        # 🔘 Layout horizontal para los botones de pausa y reiniciar
        button_layout = QHBoxLayout()


        # 🔘 Botón para iniciar el pedido automatico
        auto_start_button = QPushButton("📝 Iniciar Pedido Automático")
        auto_start_button.setStyleSheet(self.get_button_style())
        auto_start_button.clicked.connect(self.start_auto_order)
        button_layout.addWidget(auto_start_button)

        # Agregar el layout de botones al contenido principal
        auto_packaging_content_layout.addLayout(button_layout)

        self.auto_packaging_content.setLayout(auto_packaging_content_layout)
        self.auto_packaging_group_layout.addWidget(self.auto_packaging_content)

        # Inicialmente ocultar el contenido
        self.auto_packaging_group.setVisible(False)

        self.auto_packaging_group.setLayout(self.auto_packaging_group_layout)
        layout.addWidget(self.auto_packaging_group, 12, 0, 1, 3)

        # 📦 Grupo para Configuración de Empaque Manual (ahora ocultable)
        self.manual_packaging_group = QGroupBox("📦 Configuración de Empaque Manual")
        self.manual_packaging_group_layout = QVBoxLayout()

        # Contenedor para empaques
        self.manual_packaging_content = QWidget()
        manual_packaging_content_layout = QVBoxLayout(self.manual_packaging_content)

        self.presentation_widgets = []
        self.packaging_selectors = []
        self.package_count_inputs = []

        for i in range(3):  # Asumimos 3 presentaciones
            row_layout = QHBoxLayout()

            # Etiqueta de presentación
            presentation_label = QLabel(f"Presentación {i+1}: -")
            presentation_label.setStyleSheet("font-size: 14px; font-weight: bold;")
            row_layout.addWidget(presentation_label)

            # Selección de empaque
            packaging_selector = QComboBox()
            packaging_selector.addItems(["2", "4", "6"])
            row_layout.addWidget(QLabel("Empaque:"))
            row_layout.addWidget(packaging_selector)

            # Entrada de cantidad
            package_count_input = QLineEdit()
            row_layout.addWidget(QLabel("Cantidad:"))
            row_layout.addWidget(package_count_input)

            # Guardar referencias
            self.presentation_widgets.append(presentation_label)
            self.packaging_selectors.append(packaging_selector)
            self.package_count_inputs.append(package_count_input)

            # Agregar al layout
            row_widget = QWidget()
            row_widget.setLayout(row_layout)
            manual_packaging_content_layout.addWidget(row_widget)
        
        # 📊 Mensaje de factibilidad
        self.feasibility_label = QLabel("Factibilidad: En espera")
        self.feasibility_label.setStyleSheet("font-size: 14px; color: blue;")
        self.feasibility_label.setAlignment(Qt.AlignCenter)
        manual_packaging_content_layout.addWidget(self.feasibility_label)

        # ✅ Botón para verificar el pedido
        verify_button = QPushButton("Verificar Pedido")
        verify_button.setStyleSheet(self.get_button_style())
        verify_button.clicked.connect(self.verify_order)
        manual_packaging_content_layout.addWidget(verify_button)

        # 🟢 Botón para realizar el pedido
        submit_button = QPushButton("Realizar Pedido Manual")
        submit_button.setStyleSheet(self.get_button_style())
        submit_button.clicked.connect(self.start_manual_order)
        manual_packaging_content_layout.addWidget(submit_button)

        self.manual_packaging_content.setLayout(manual_packaging_content_layout)
        self.manual_packaging_group_layout.addWidget(self.manual_packaging_content)

        # Inicialmente ocultar el contenido
        self.manual_packaging_group.setVisible(False)

        self.manual_packaging_group.setLayout(self.manual_packaging_group_layout)
        layout.addWidget(self.manual_packaging_group, 13, 0, 1, 3)

        self.tabs.addTab(order_tab, "Orden de Pedido")
    
    def toggle_opcua_visibility(self, checked):
        """Muestra u oculta la configuración OPC-UA."""
        self.opcua_group.setVisible(checked)
        if checked:
            self.toggle_opcua_button.setText("🔼 Ocultar Configuración OPC-UA")
        else:
            self.toggle_opcua_button.setText("🔽 Mostrar Configuración OPC-UA")
    
    # Función para ocultar/mostrar
    def toggle_packaging_visibility(self,checked):
        self.manual_packaging_group.setVisible(checked)
        self.auto_packaging_group.setVisible(checked)
        self.trajectory_node_group.setVisible(checked)
        if checked:
            self.packaging_toggle_button.setText("⚙️ Ocultar Configuración de Empaquetado Manual")
        else:
            self.packaging_toggle_button.setText("⚙️ Mostrar Configuración de Empaquetado Manual")

        
    def start_driver(self):
        """Lanza el driver del robot usando subprocess, con IP, RViz y kinematics file."""
        if self.driver_process is not None:
            QMessageBox.warning(self, "Advertencia", "El driver ya está en ejecución.")
            return

        ip = self.robot_ip_input.text().strip()
        if not ip:
            QMessageBox.warning(self, "Error", "Debes ingresar la IP del robot.")
            return

        launch_rviz = "true" if self.rviz_checkbox.isChecked() else "false"
        kin_path = self.kinematics_input.text().strip()

        # Construcción básica del comando
        command = [
            "ros2", "launch", "ur_robot_driver", "ur_control.launch.py",
            "ur_type:=ur3",
            f"robot_ip:={ip}",
            f"launch_rviz:={launch_rviz}"
        ]
        # Sólo añadimos kinematics_params_file si el usuario ingresó algo
        if kin_path:
            command.append(f"kinematics_params_file:={kin_path}")

        try:
            self.driver_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.driver_status_label.setText("Estado del Driver: Iniciado")
            self.driver_status_label.setStyleSheet("font-size: 16px; color: green;")
            QMessageBox.information(self, "Éxito", "Driver del Robot iniciado correctamente.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error al iniciar el driver: {e}")

    def stop_driver(self):
        """Detener el driver del robot si está corriendo."""
        if self.driver_process:
            try:
                os.killpg(os.getpgid(self.driver_process.pid), signal.SIGINT)
                self.driver_process.wait()
                self.driver_process = None
                self.driver_status_label.setText("Estado del Driver: Detenido")
                self.driver_status_label.setStyleSheet("font-size: 16px; color: red;")
                QMessageBox.information(self, "Éxito", "Driver del Robot detenido correctamente.")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error al detener el driver: {e}")
        else:
            QMessageBox.warning(self, "Advertencia", "No hay un driver en ejecución.")
    
    def run_robot_program(self):
        """Ejecuta el servicio para iniciar el programa en el robot."""
        self.node.get_logger().info("Enviando solicitud para iniciar el programa en el robot...") 
        # Crear cliente del servicio
        client = self.node.create_client(Trigger, "/dashboard_client/play")

        if not client.wait_for_service(timeout_sec=3.0):
            QMessageBox.warning(self, "Error", " No se encontró el servicio /dashboard_client/play")
            return

        # Crear solicitud vacía
        request = Trigger.Request()

        # Llamar al servicio de manera asíncrona
        future = client.call_async(request)
        future.add_done_callback(self.handle_robot_program_response)

    def handle_robot_program_response(self, future):
        """Maneja la respuesta del servicio después de iniciar el programa en el robot."""
        try:
            response = future.result()
            if response.success:
                QMessageBox.information(self, "Éxito", "Programa en el robot iniciado correctamente.")
            else:
                QMessageBox.critical(self, "Error", f"Error al iniciar el programa: {response.message}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Excepción al llamar al servicio: {e}")

    def load_sequences(self):
        """Lee el archivo YAML que contiene las secuencias y devuelve la lista de nombres de secuencias."""
        sequences = []
        file_path = "trajectories.yaml"
        if os.path.exists(file_path):
            with open(file_path, "r") as f:
                data = yaml.safe_load(f)
                if data:
                    sequences = list(data.keys())
        return sequences

    def refresh_sequence_comboboxes(self):
        """Actualiza los combobox de secuencias en las pestañas relevantes."""
        seqs = self.load_sequences()
        # Actualiza la pestaña de captura de puntos
        self.point_capture_sequence_selector.clear()
        self.point_capture_sequence_selector.addItems(seqs)
        # Actualiza la pestaña de secuencias de empaquetado
        self.packaging_sequence_selector.clear()
        self.packaging_sequence_selector.addItems(seqs)
        # Si tienes algún combobox en la pestaña de orden de pedido, actualízalo también
        # self.order_sequence_selector.clear()
        # self.order_sequence_selector.addItems(seqs)
    
    def create_new_sequence(self):
        """Solicita al usuario el nombre de una nueva secuencia y la añade al archivo YAML."""
        seq_name, ok = QInputDialog.getText(self, "Nueva Secuencia", "Ingrese el nombre de la nueva secuencia:")
        if ok and seq_name:
            file_path = "trajectories.yaml"
            # Leer datos existentes
            if os.path.exists(file_path):
                with open(file_path, "r") as f:
                    data = yaml.safe_load(f) or {}
            else:
                data = {}
            if seq_name in data:
                QMessageBox.warning(self, "Advertencia", "Ya existe una secuencia con ese nombre.")
                return
            # Agregar la nueva secuencia (puedes definir la estructura inicial; por ejemplo, vacío o con un placeholder)
            data[seq_name] = {}  # O [] si la estructura de la secuencia es una lista de trayectorias
            with open(file_path, "w") as f:
                yaml.dump(data, f)
            self.refresh_sequence_comboboxes()
            QMessageBox.information(self, "Éxito", f"Secuencia '{seq_name}' creada correctamente.")

    def delete_sequence(self):
        """Elimina la secuencia seleccionada tras confirmar con el usuario."""
        seq_name = self.point_capture_sequence_selector.currentText()
        if not seq_name:
            QMessageBox.warning(self, "Advertencia", "No hay ninguna secuencia seleccionada.")
            return
        reply = QMessageBox.question(
            self,
            "Confirmar eliminación",
            f"¿Está seguro de eliminar la secuencia '{seq_name}'?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            file_path = "trajectories.yaml"
            if os.path.exists(file_path):
                with open(file_path, "r") as f:
                    data = yaml.safe_load(f) or {}
                if seq_name in data:
                    del data[seq_name]
                    with open(file_path, "w") as f:
                        yaml.dump(data, f)
                    self.refresh_sequence_comboboxes()
                    QMessageBox.information(self, "Éxito", f"Secuencia '{seq_name}' eliminada correctamente.")
                else:
                    QMessageBox.warning(self, "Advertencia", "La secuencia no se encontró en el archivo.")


    def clear_sequence(self):
        """Muestra un mensaje de confirmación antes de limpiar la secuencia seleccionada."""
        sequence = self.point_capture_sequence_selector.currentText()

        # Crear cuadro de diálogo de confirmación
        confirm_dialog = QMessageBox()
        confirm_dialog.setIcon(QMessageBox.Warning)
        confirm_dialog.setWindowTitle("Confirmación")
        confirm_dialog.setText(f"⚠️ ¿Estás seguro de que deseas borrar la secuencia '{sequence}'?")
        confirm_dialog.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        confirm_dialog.setDefaultButton(QMessageBox.No)

        # Mostrar el cuadro de diálogo y capturar la respuesta
        response = confirm_dialog.exec_()

        # Si el usuario elige "No", cancelar la acción
        if response == QMessageBox.No:
            return

        # Si el usuario elige "Sí", proceder con la limpieza
        command = [
            "ros2", "run", "control_ur3", "captura_varios",
            f"--ros-args", "-p", f"sequence_name:={sequence}", "-p", "clear_sequence:=True"
        ]

        try:
            subprocess.run(command, check=True)
            QMessageBox.information(self, "Éxito", f"Secuencia '{sequence}' limpiada correctamente.")
        except subprocess.CalledProcessError as e:
            QMessageBox.critical(self, "Error", f" Error al limpiar la secuencia: {e}")


    def capture_point(self):
        """Lanza el nodo para capturar un punto."""
        sequence = self.point_capture_sequence_selector.currentText()
        gripper_action = self.gripper_selector.currentText()
        movement_type = self.movement_type_selector.currentText()
        time_step = self.time_step_input.text()

        # Validar que el tiempo ingresado tenga un punto decimal
        if "." not in time_step:
            QMessageBox.warning(self, "Error", "Ingrese un número con decimales (ejemplo: 2.0, 3.5, etc.).")
            return

        # Convertir a float después de validar el formato
        try:
            time_step = float(time_step)
        except ValueError:
            QMessageBox.warning(self, "Error", "Ingrese un número válido para el tiempo entre puntos.")
            return      


        command = [
            "ros2", "run", "control_ur3", "captura_varios",
            f"--ros-args", 
            "-p", f"sequence_name:={sequence}", 
            "-p", f"gripper_action:={gripper_action}", 
            "-p", f"time_step:={time_step}",
            "-p", f"movement_type:={movement_type}"         
        ]
        # Si el tipo de movimiento es "moveL", se leen y agregan los parámetros de interpolación
        if movement_type == "moveL":
            num_steps_text = self.num_steps_input.text()
            interp_time_text = self.interp_time_input.text()
            try:
                num_steps_value = int(num_steps_text)
                interp_time_value = float(interp_time_text)
            except ValueError:
                QMessageBox.warning(self, "Error", "Ingrese valores válidos: número de puntos interpolados (entero) y tiempo entre puntos interpolados (decimal).")
                return
            # Agregar los nuevos parámetros a la lista del comando  
            command.extend([
                "-p", f"num_steps:={num_steps_value}",
                "-p", f"interp_p_time:={interp_time_value}"
            ])
        try:
            self.capture_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.capture_status_label.setText("Capturando punto...")
            self.capture_status_label.setStyleSheet("font-size: 16px; color: orange;")
            self.capture_process.wait()
            self.capture_status_label.setText(f"Punto capturado en {sequence}.")
            self.capture_status_label.setStyleSheet("font-size: 16px; color: green;")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error al capturar el punto: {e}")
    
    
    def start_sequence_wrapper(self):
        mode = self.execution_mode_selector.currentText()  # “Completo” o “Parcial”
        if mode == "Completo":
            self.start_sequence_full()
        else:
            self.start_sequence_partial()

    def start_sequence_full(self):
        """
        Lanza la secuencia de empaquetado "completa" con tapas, pickup, etc.
        """
        opcua_url = self.opcua_url_input.text()
        sensor_node_id = self.sensor_node_id_input.text()
        delay = self.delay_input.text()
        sequence_name = self.packaging_sequence_selector.currentText()

        command = [
            "ros2", "run", "control_ur3", "control_movimiento",
            "--ros-args",
            "-p", f"use_topic:=False",
            "-p", f"opcua_url:={opcua_url}",
            "-p", f"sensor_node_id:={sensor_node_id}",
            "-p", f"delay_between_trajectories:={delay}",
            "-p", f"sequence_name:={sequence_name}"
        ]
        try:
            self.sequence_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.sequence_status_label.setText(f"Nodo de control de movimiento iniciado con la trayectoria en pausa. Secuencia {sequence_name} (Completa) en espera.")
            self.sequence_status_label.setStyleSheet("font-size: 16px; color: yellow; font-weight: bold;")
            self.packaging_paused = False
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error al iniciar la secuencia completa: {e}")
    
    def start_sequence_partial(self):
        """
        Lanza la secuencia de empaquetado "parcial": solo las trayectorias definidas
        en el YAML para la secuencia (sin tapas ni pickup).
        """
        opcua_url = self.opcua_url_input.text()
        sensor_node_id = self.sensor_node_id_input.text()
        delay = self.delay_input.text()
        sequence_name = self.packaging_sequence_selector.currentText()

        command = [
            "ros2", "run", "control_ur3", "control_movimiento",
            "--ros-args",
            "-p", f"use_topic:=False",
            "-p", f"opcua_url:={opcua_url}",
            "-p", f"sensor_node_id:={sensor_node_id}",
            "-p", f"delay_between_trajectories:={delay}",
            "-p", f"sequence_name:={sequence_name}",
            "-p", "only_sequence_points:=True"
        ]

        try:
            self.sequence_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.sequence_status_label.setText(f"Nodo de control de movimiento iniciado con la trayectoria en pausa. Secuencia {sequence_name} (Parcial) en espera.")
            self.sequence_status_label.setStyleSheet("font-size: 16px; color: yellow; font-weight: bold;")
            self.packaging_paused = False
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error al iniciar la secuencia parcial: {e}")


    def stop_sequence(self):
        """Detiene la secuencia de empaquetado si está corriendo."""
        if self.sequence_process:
            os.killpg(os.getpgid(self.sequence_process.pid), signal.SIGINT)
            self.sequence_status_label.setText("Nodo de control de movimiento cerrado.")
            self.sequence_status_label.setStyleSheet("font-size: 16px; color: red; font-weight: bold;")
            self.sequence_process = None
            self.packaging_paused = True

    
    def start_order_monitor(self):
        """Inicia el nodo de monitoreo con los valores proporcionados por el usuario en la GUI."""
        if not hasattr(self, "monitor_process") or self.monitor_process is None:
            opcua_url = self.opcua_url_input.text().strip()
            ns_value = self.ns_input.text().strip()
            tipo_producto_i = self.tipo_producto_i_input.text().strip()
            presentacion_i = self.presentacion_i_input.text().strip()
            cantidad_presentacion_i = self.cantidad_presentacion_i_input.text().strip()
            cantidad_pedido_i = self.cantidad_pedido_i_input.text().strip()
            tipo_pallet_i = self.tipo_pallet_i_input.text().strip()
            cantidad_pallet_i = self.cantidad_pallet_i_input.text().strip()

            # Validar que todos los valores sean números
            if not ns_value.isdigit() or not tipo_producto_i.isdigit() or not presentacion_i.isdigit() \
            or not cantidad_presentacion_i.isdigit() or not cantidad_pedido_i.isdigit():
                QMessageBox.warning(self, "Advertencia", "Todos los valores deben ser números enteros.")
                return

            try:
                command = [
                    "ros2", "run", "control_ur3", "order_monitor",
                    "--ros-args",
                    "-p", f"opcua_url:={opcua_url}",
                    "-p", f"ns:={ns_value}",
                    "-p", f"tipo_producto_i:={tipo_producto_i}",
                    "-p", f"presentacion_i:={presentacion_i}",
                    "-p", f"cantidad_presentacion_i:={cantidad_presentacion_i}",
                    "-p", f"cantidad_pedido_i:={cantidad_pedido_i}",
                    "-p", f"tipo_pallet_i:={tipo_pallet_i}",
                    "-p", f"cantidad_pallet_i:={cantidad_pallet_i}"
                ]

                self.monitor_process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                self.monitor_status_label.setText("Estado del Monitoreo: Iniciado")
                self.monitor_status_label.setStyleSheet("font-size: 16px; color: green;")
                QMessageBox.information(self, "Éxito", "Nodo de monitoreo iniciado correctamente.")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error al iniciar el nodo de monitoreo: {e}")
        else:
            QMessageBox.warning(self, "Advertencia", "El nodo de monitoreo ya está en ejecución.")


    def stop_order_monitor(self):
        """Detiene el nodo de monitoreo."""
        if hasattr(self, "monitor_process") and self.monitor_process is not None:
            try:
                os.killpg(os.getpgid(self.monitor_process.pid), signal.SIGINT)
                self.monitor_process.wait()
                self.monitor_process = None
                self.monitor_status_label.setText("Estado del Monitoreo: Detenido")
                self.monitor_status_label.setStyleSheet("font-size: 16px; color: red;")
                QMessageBox.information(self, "Éxito", "Nodo de monitoreo detenido correctamente.")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error al detener el nodo de monitoreo: {e}")
        else:
            QMessageBox.warning(self, "Advertencia", "No hay un nodo de monitoreo en ejecución.")

    def start_trajectory_node(self):
        """Inicia el nodo de trayectorias con los valores proporcionados por el usuario en la GUI."""
        if not hasattr(self, "trajectory_process") or self.trajectory_process is None:
            opcua_url = self.opcua_url_input.text()
            sensor_node_id = self.sensor_node_id_input.text()
            delay = self.delay_input.text()
            
            try:
                command = [
                    "ros2", "run", "control_ur3", "control_movimiento",
                    "--ros-args",
                    "-p", f"opcua_url:={opcua_url}",
                    "-p", f"sensor_node_id:={sensor_node_id}",
                    "-p", f"delay_between_trajectories:={delay}",
                    "-p", "use_topic:=True"
                ]

                self.trajectory_process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                self.package_status_label.setText("Nodo de Trayectorias Activado, Esperando Pedido...")
                self.package_status_label.setStyleSheet("font-size: 16px; color: green;")

                QMessageBox.information(self, "Éxito", "Nodo de trayectorias iniciado correctamente.")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error al iniciar el nodo de trayectorias: {e}")
        else:
            QMessageBox.warning(self, "Advertencia", "El nodo de trayectorias ya está en ejecución.")

    def stop_trajectory_node(self):
        """Detiene el nodo de trayectorias."""
        if hasattr(self, "trajectory_process") and self.trajectory_process is not None:
            try:
                os.killpg(os.getpgid(self.trajectory_process.pid), signal.SIGINT)
                self.trajectory_process.wait()
                self.trajectory_process = None
                self.package_status_label.setText("Nodo de Trayectorias Desactivado.")
                self.package_status_label.setStyleSheet("font-size: 16px; color: red;")
                QMessageBox.information(self, "Éxito", "Nodo de trayectorias detenido correctamente.")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error al detener el nodo de trayectorias: {e}")
        else:
            QMessageBox.warning(self, "Advertencia", "No hay un nodo de trayectorias en ejecución.")
    
    def enable_opcua_service(self):
        """Invoca el servicio /enable_opcua para habilitar OPC-UA."""
        client = self.node.create_client(Empty, '/enable_opcua')
        if not client.wait_for_service(timeout_sec=3.0):
            QMessageBox.warning(self, "Servicio OPC-UA", "No se encontró el servicio /enable_opcua")
            return
        req = Empty.Request()
        future = client.call_async(req)
        future.add_done_callback(self.handle_opcua_response)

    def handle_opcua_response(self, future):
        """Callback para manejar la respuesta del servicio OPC-UA."""
        try:
            response = future.result()
            QMessageBox.information(self, "Servicio OPC-UA", "OPC-UA habilitado correctamente.")
        except Exception as e:
            QMessageBox.critical(self, "Servicio OPC-UA", f"Error al habilitar OPC-UA: {e}")

    def send_pause_request_secuence(self):
            """Envía una solicitud para pausar o reanudar la ejecución."""
            try:
                node = rclpy.create_node('pause_request_node')
                client = node.create_client(Empty, '/pause_execution')
                
                if not client.wait_for_service(timeout_sec=2.0):
                    QMessageBox.warning(self, "Error", "El servicio /pause_execution no está disponible.")
                    return
                
                request = Empty.Request()
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future)
                
                if future.result() is not None:
                    # Alternar estado de la ejecución
                    self.packaging_paused = not self.packaging_paused
                    
                    if self.packaging_paused:
                        self.sequence_status_label.setText("Se reanudó la ejecución de la trayectoria")
                        self.sequence_status_label.setStyleSheet("font-size: 16px; color: green; font-weight: bold;")
                        QMessageBox.information(self, "Éxito", "Se ha reanudado la ejecución de la trayectoria.")
                    else:
                        self.sequence_status_label.setText("Ejecución de la trayectoria pausada")
                        self.sequence_status_label.setStyleSheet("font-size: 16px; color: yellow; font-weight: bold;")
                        QMessageBox.information(self, "Éxito", "Se ha pausado la ejecución de la trayectoria.")
                else:
                    QMessageBox.warning(self, "Error", "No se pudo enviar la solicitud.")
                
                node.destroy_node()
            
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error al enviar la solicitud: {e}")

    def send_pause_request(self):
            """Envía una solicitud para pausar o reanudar la ejecución."""
            try:
                node = rclpy.create_node('pause_request_node')
                client = node.create_client(Empty, '/pause_execution')
                
                if not client.wait_for_service(timeout_sec=2.0):
                    QMessageBox.warning(self, "Error", "El servicio /pause_execution no está disponible.")
                    return
                
                request = Empty.Request()
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future)
                
                if future.result() is not None:
                    # Alternar estado de la ejecución
                    self.seq_is_paused = not self.seq_is_paused

                    if self.seq_is_paused:
                        self.package_status_label.setText("Se reanudó la ejecución de la trayectoria")
                        self.package_status_label.setStyleSheet("font-size: 16px; color: green; font-weight: bold;")
                        QMessageBox.information(self, "Éxito", "Se ha reanudado la ejecución de la trayectoria.")
                    else:
                        self.package_status_label.setText("Ejecución de la trayectoria pausada")
                        self.package_status_label.setStyleSheet("font-size: 16px; color: yellow; font-weight: bold;")
                        QMessageBox.information(self, "Éxito", "Se ha pausado la ejecución de la trayectoria.")
                else:
                    QMessageBox.warning(self, "Error", "No se pudo enviar la solicitud.")
                
                node.destroy_node()
            
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error al enviar la solicitud: {e}")

    def send_reset_request(self):
        """Envía una solicitud para reiniciar la ejecución."""
        try:
            node = rclpy.create_node('reset_request_node')
            client = node.create_client(Empty, '/reset_execution')
            
            if not client.wait_for_service(timeout_sec=2.0):
                QMessageBox.warning(self, "Error", "El servicio /reset_execution no está disponible.")
                return
            
            request = Empty.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            
            if future.result() is not None:
                # Actualizar el estado en la GUI
                self.package_status_label.setText("Se reinició la secuencia correctamente")
                self.package_status_label.setStyleSheet("font-size: 16px; color: blue; font-weight: bold;")

                QMessageBox.information(self, "Éxito", "Se reinició la secuencia correctamente.")
            else:
                QMessageBox.warning(self, "Error", "No se pudo enviar la solicitud.")
            
            node.destroy_node()
        
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error al enviar la solicitud: {e}")

    
    def start_auto_order(self):
        """Inicia la ejecución manual del pedido utilizando los datos de la GUI."""
        try:
            # Obtener los valores desde la GUI
            tipo_producto = int(self.product_type_label.text().split(": ")[1])
            cantidad_pedido = float(self.order_quantity_label.text().split(": ")[1])

            # Obtener listas de presentaciones, cantidad por presentación, tipo de pallet y cantidad de pallet
            cantidad_labels = [self.quantity_1_label, self.quantity_2_label, self.quantity_3_label]
            pallet_type_labels = [self.pallet_type_labels[0], self.pallet_type_labels[1], self.pallet_type_labels[2]]
            pallet_quantity_labels = [self.pallet_quantity_labels[0], self.pallet_quantity_labels[1], self.pallet_quantity_labels[2]]

            presentacion = []
            cantidad_presentacion = []
            tipo_pallet = []
            cantidad_pallet = []

            for i in range(3):
                cantidad_text = cantidad_labels[i].text().split(": ")[1]
                pallet_type_text = pallet_type_labels[i].text().split(": ")[1]
                pallet_quantity_text = pallet_quantity_labels[i].text().split(": ")[1]

                if cantidad_text != "-":
                    presentacion.append(i + 1)  # Presentación 1, 2, 3
                    cantidad_presentacion.append(int(cantidad_text))  # Cantidad disponible
                    tipo_pallet.append(int(pallet_type_text))  # Tipo de pallet recibido del PLC
                    cantidad_pallet.append(int(pallet_quantity_text))  # Cantidad de pallets recibidos

            # Construir el mensaje de confirmación
            confirm_message = (
                f"🔹 Pedido a enviar:\n"
                f"📦 Tipo de Producto: {tipo_producto}\n"
                f"📦 Presentaciones: {presentacion}\n"
                f"📊 Cantidades por Presentación: {cantidad_presentacion}\n"
                f"🎁 Tipo de Pallet: {tipo_pallet}\n"
                f"📦 Cantidad de Pallets: {cantidad_pallet}\n"
                f"🔢 Total de unidades: {cantidad_pedido}\n\n"
                "⚠️ ¿Estás seguro de que deseas enviarlo?"
            )

            # Mostrar cuadro de confirmación
            reply = QMessageBox.question(self, "Confirmación de Pedido", confirm_message,
                                        QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

            if reply == QMessageBox.No:
                return  # Si el usuario cancela, no se envía el pedido
            
            # Construir el comando para publicar en el tópico
            command = [
                "ros2", "topic", "pub", '/confirmed_order', "custom_msgs/msg/OrderStatus",
                f"{{tipo_producto: {tipo_producto}, presentacion: {presentacion}, cantidad_presentacion: {cantidad_presentacion}, "
                f"cantidad_pedido: {cantidad_pedido}, tipo_pallet: {tipo_pallet}, cantidad_pallet: {cantidad_pallet}}}",
                "--once"
            ]

            subprocess.run(command, check=True)

            QMessageBox.information(self, "Éxito", f"Pedido publicado con {cantidad_pedido} unidades en {len(presentacion)} presentaciones.")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error al iniciar el pedido: {e}")

    
    def start_manual_order(self):
        """Inicia la ejecución del pedido de empaquetado publicando en el tópico /order_status."""
        try:
            # Obtener los valores ingresados en la GUI
            tipo_producto = int(self.product_type_label.text().split(": ")[1])
            cantidad_pedido = float(self.order_quantity_label.text().split(": ")[1])

            # Obtener listas de presentaciones, cantidad por presentación, tipo de pallet y cantidad de pallet
            cantidad_labels = [self.quantity_1_label, self.quantity_2_label, self.quantity_3_label]
            packaging_selectors = [self.packaging_selectors[0], self.packaging_selectors[1], self.packaging_selectors[2]]
            package_count_inputs = [self.package_count_inputs[0], self.package_count_inputs[1], self.package_count_inputs[2]]

            presentacion = []
            cantidad_presentacion = []
            tipo_pallet = []
            cantidad_pallet = []

            for i in range(3):
                cantidad_text = cantidad_labels[i].text().split(": ")[1]
                if cantidad_text != "-":
                    presentacion.append(i + 1)  # Presentación 1, 2, 3
                    cantidad_presentacion.append(int(cantidad_text))  # Cantidad de unidades disponibles
                    tipo_pallet.append(int(packaging_selectors[i].currentText()))  # Tipo de pallet seleccionado
                    cantidad_pallet.append(int(package_count_inputs[i].text()) if package_count_inputs[i].text() else 0)  # Cantidad de pallets

            # Construir el mensaje de confirmación
            confirm_message = (
                f"🔹 Pedido a enviar:\n"
                f"📦 Tipo de Producto: {tipo_producto}\n"
                f"📦 Presentaciones: {presentacion}\n"
                f"📊 Cantidades por Presentación: {cantidad_presentacion}\n"
                f"🎁 Tipo de Pallet: {tipo_pallet}\n"
                f"📦 Cantidad de Pallets: {cantidad_pallet}\n"
                f"🔢 Total de unidades: {cantidad_pedido}\n\n"
                "⚠️ ¿Estás seguro de que deseas enviarlo?"
            )

            # Mostrar cuadro de confirmación
            reply = QMessageBox.question(self, "Confirmación de Pedido", confirm_message,
                                        QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

            if reply == QMessageBox.No:
                return  # Si el usuario cancela, no se envía el pedido
            
            # Construir el comando para publicar en el tópico
            command = [
                "ros2", "topic", "pub", '/confirmed_order', "custom_msgs/msg/OrderStatus",
                f"{{tipo_producto: {tipo_producto}, presentacion: {presentacion}, cantidad_presentacion: {cantidad_presentacion}, "
                f"cantidad_pedido: {cantidad_pedido}, tipo_pallet: {tipo_pallet}, cantidad_pallet: {cantidad_pallet}}}",
                "--once"
            ]

            subprocess.run(command, check=True)

            QMessageBox.information(self, "Éxito", f"Pedido publicado con {cantidad_pedido} unidades en {len(presentacion)} presentaciones.")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error al iniciar el pedido: {e}")


    def verify_order(self):
        """Valida el pedido para cada presentación de forma independiente y muestra la factibilidad."""
        try:
            # Variables para la factibilidad general
            factibility_messages = []
            all_valid = True  # Para determinar si todas las presentaciones son válidas

            # Lista de etiquetas correspondientes a las cantidades por presentación
            quantity_labels = [self.quantity_1_label, self.quantity_2_label, self.quantity_3_label]

            # Iterar sobre cada presentación
            for i in range(3):
                package_size = int(self.packaging_selectors[i].currentText())  # Tipo de empaque (4, 6, 12)
                num_packages = int(self.package_count_inputs[i].text())  # Número de empaques a realizar

                # Obtener la cantidad disponible de la presentación
                total_units_available_text = quantity_labels[i].text().split(": ")[1]
                total_units_available = int(total_units_available_text) if total_units_available_text.isdigit() else 0

                # Calcular empaques completos posibles y unidades sobrantes
                total_units_required = package_size * num_packages  # Total necesario para la presentación
                full_packages = total_units_available // package_size
                leftover_units = total_units_available % package_size

                # Verificar si el pedido es válido
                if total_units_required <= total_units_available:
                    factibility_messages.append(
                        f"✅ Presentación {i+1}: Pedido válido. Se pueden realizar {num_packages} empaques completos."
                    )
                else:
                    all_valid = False  # Marcar que al menos una presentación no es válida
                    factibility_messages.append(
                        f"❌ Presentación {i+1}: Pedido no válido. "
                        f"Solo se pueden realizar {full_packages} empaques completos. "
                        f"Unidades sobrantes: {leftover_units}."
                    )

            # Actualizar la etiqueta de factibilidad en la GUI
            self.feasibility_label.setText("\n".join(factibility_messages))
            self.feasibility_label.setStyleSheet("font-size: 14px; color: green;" if all_valid else "font-size: 14px; color: red;")

        except Exception as e:
            self.feasibility_label.setText("❌ Error en los datos ingresados.")
            self.feasibility_label.setStyleSheet("font-size: 14px; color: red;")

    def run(self):
        """Ejecuta la GUI y mantiene ROS2 corriendo."""
        self.window.show()
        try:
            self.app.exec_()
        except KeyboardInterrupt:
            self.get_logger().info("Finalizando GUI.")
            self.shutdown()

    def shutdown(self):
        """Finaliza el nodo ROS2 y cierra la GUI."""
        if self.driver_process:
            self.stop_driver()
        self.get_logger().info("Cerrando nodo GUI.")
        rclpy.shutdown()
    
class OrderStatusNode(Node):
    """Nodo ROS2 que se suscribe al tópico /order_status y actualiza la GUI."""

    def __init__(self, gui):
        super().__init__('order_status_gui')
        self.gui = gui

        self.subscription = self.create_subscription(
            OrderStatus,
            '/order_status',
            self.order_status_callback,
            10
        )
        self.get_logger().info("Nodo suscrito a /order_status.")

    def order_status_callback(self, msg):
        """Callback cuando se recibe un mensaje de /order_status."""
        #self.get_logger().info(f"Mensaje recibido: {msg}")
        self.gui.update_order_status(msg)


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    node = OrderStatusNode(None)
    gui = RobotControlGUI(node)
    node.gui = gui

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    gui.show()
    sys.exit(app.exec_())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()