from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import subprocess

def generate_launch_description():
    position_goals = PathJoinSubstitution(
        [FindPackageShare("control_ur3"), "config", "go_home.yaml"]
    )

    # Iniciar el publicador
    publisher_node = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_joint_trajectory_controller",
        name="publisher_scaled_joint_trajectory_controller",
        parameters=[position_goals],
        output="screen",
    )
    # Lanzar el nodo publicador como un proceso separado para obtener su PID
    #process = subprocess.Popen(['ros2', 'launch', 'control_ur3', 'go_home.launch.py'])  # Reemplaza 'launch_file.py' con el nombre del launch


    # Nodo de monitoreo que verifica si se ha alcanzado la posición
    monitor_node = Node(
        package="control_ur3",
        executable="monitor_position",
        name="monitor_position",
        output="screen",
    )


    # Registrar el PID del proceso del publicador en el nodo de monitoreo
    #monitor_node.set_publisher_process_pid(process.pid)


    return LaunchDescription([
        publisher_node,
        monitor_node
    ])