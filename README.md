
# UR3 Packaging Control – Sistema de Paletizado Automatizado

Este repositorio contiene el desarrollo de un sistema automatizado de paletizado de refrescos, utilizando un brazo robótico colaborativo **UR3** controlado mediante **ROS 2**. El sistema fue implementado como parte de una pasantía académica en el **Centro de Electricidad y Automatización Industrial (CEAI)** del **SENA**, Regional Valle del Cauca.

---

## 📄 Descripción del Proyecto

El proyecto busca optimizar el proceso de paletizado final en una planta de embotellado didáctica, fortaleciendo la formación práctica de aprendices en robótica industrial y automatización, enmarcado en los principios de la **Industria 4.0**.

Se diseñó una arquitectura basada en **ROS 2**, utilizando nodos modulares para:
- Control de movimiento del UR3.
- Captura de trayectorias personalizadas.
- Comunicación industrial mediante **OPC UA**.
- Interfaz gráfica de usuario (**GUI**) para supervisión y control del sistema.

---

## 🛠️ Tecnologías Utilizadas

- **UR3** – Universal Robots
- **ROS 2** (Foxy / Humble)
- **Python** – Programación de nodos ROS2 y GUI
- **Tkinter** – Interfaz gráfica
- **asyncua** – Cliente OPC UA en Python
- **Modbus TCP** (opcional como alternativa de comunicación)
- **YAML** – Configuración de trayectorias

---

## 📂 Estructura del Repositorio

```bash
ur3_packaging_control/
│
├── src/                          # Código fuente de nodos ROS2 y scripts de control
│   ├── capture_points/           # Nodo para captura de puntos articulares
│   ├── control_movement/         # Nodo principal de ejecución de trayectorias
│   ├── gui_interface/            # GUI para operador (Tkinter)
│   ├── opcua_client/             # Comunicación con PLC
│   └── common_msgs/              # Mensajes y servicios personalizados
│
├── config/                       # Archivos de configuración YAML de trayectorias
│
├── launch/                       # Archivos de lanzamiento de nodos ROS2
│
├── README.md                     # Documentación del proyecto
```

---

## 🚀 Instalación y Ejecución

### 1. Clonar el repositorio:
```bash
git clone https://github.com/SebastianBurbano/ur3_packaging_control.git
```

### 2. Compilar el paquete:
```bash
cd ur3_packaging_control
colcon build --packages-select control_ur3
source install/setup.bash
```

### 3. Lanzar los nodos principales:
```bash
ros2 launch control_ur3 full_system.launch.py
```
*(Asegúrate de tener configurada la red Ethernet con IPs fijas para comunicarte con el UR3 y el PLC)*

---

## 🧠 Funcionalidades principales

- **Control de UR3**: Ejecución de trayectorias pregrabadas mediante `scaled_joint_trajectory_controller`.
- **Captura de Trayectorias**: Nodo de captura de puntos manuales, interpolación y creación de trayectorias personalizadas.
- **Comunicación PLC-ROS2**: Recepción de órdenes de empaquetado en tiempo real vía **OPC UA**.
- **Interfaz gráfica (GUI)**:
  - Gestión de pedidos.
  - Activación/desactivación de nodos de control.
  - Captura de nuevas trayectorias.
  - Supervisión del estado de pedidos.

---

## 📊 Resultados y Validación

La integración fue validada en un entorno real de laboratorio, logrando:

- **100% de éxito** en ejecución de trayectorias bajo condiciones controladas.
- Precisión de posicionamiento aceptable (variaciones menores a 3-5 mm en condiciones ideales).
- Interfaz gráfica intuitiva para gestión de pedidos y control del sistema.
- Comunicación estable mediante **OPC-UA** sobre red Ethernet dedicada.

🎥 **Videos demostrativos**: Ver [Anexo_A](link_al_video)  
📸 **Capturas de interfaz y pruebas**: Se encuentran en la carpeta de anexos o en la sección de resultados del proyecto.

---

## 🔎 Imágenes del Proyecto

**Diagrama general del sistema:**

![Diagrama General](link_a_imagen_diagrama)

**Interfaz gráfica:**

![GUI Principal](link_a_imagen_gui)

---

## 📚 Documentación de Apoyo

- **Documento final de pasantía**: [Ver aquí](link_al_pdf_documento_final)
- **Lista de nodos y tópicos ROS 2**: [Ver arquitectura del sistema](link_a_documento_o_archivo)

---

## 🔑 Palabras Clave

`ROS 2` | `UR3` | `Robótica colaborativa` | `Automatización industrial` | `OPC-UA` | `Empaquetado automático` | `Manufactura inteligente`

---

## 👨‍💻 Autor

**Juan Sebastián Burbano Urbano**  
Pasantía organizacional para optar al título de Ingeniero Mecatrónico  
**Universidad Autónoma de Occidente**, 2025

---

# 📌 Notas

Este proyecto moderniza el entorno de formación del CEAI y sirve como base replicable para futuras aplicaciones reales de paletizado y automatización industrial.
