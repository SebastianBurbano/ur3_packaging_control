#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionClient

#from std_msgs.msg import String
from std_srvs.srv import Trigger

from ur_msgs.srv import SetIO

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

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


class PruebaClient(rclpy.node.Node):

    def __init__(self):
        super().__init__("empaque_prueba")
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

        self.declare_parameter("trajectory_file", "trajectories.yaml")  # Archivo YAML
        self.declare_parameter("sequence_name", "empaque_4")  # Secuencia activa
        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value
        trajectory_file = self.get_parameter("trajectory_file").value
        sequence_name = self.get_parameter("sequence_name").value

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        # Cargar trayectorias desde el archivo YAML
        self.trajectories = load_trajectories_from_yaml(trajectory_file, sequence_name, self.joints)
        self.parse_trajectories()
        self.i = 0
        self._send_goal_future = None
        self._get_result_future = None
        self.execute_next_trajectory()

    def parse_trajectories(self):
        self.goals = {}

        for traj_name, points in self.trajectories.items():
            goal = JointTrajectory()
            goal.joint_names = self.joints
            for pt in points:
                point = JointTrajectoryPoint()
                point.positions = pt["positions"]
                point.velocities = pt["velocities"]
                point.time_from_start = pt["time_from_start"]
                goal.points.append(point)

            self.goals[traj_name] = goal

    def execute_next_trajectory(self):
        if self.i >= len(self.goals):
            self.get_logger().info("Done with all trajectories")
            raise SystemExit
        traj_name = list(self.goals)[self.i]
        self.i = self.i + 1
        if traj_name:
            self.execute_trajectory(traj_name)

    def execute_trajectory(self, traj_name):
        self.get_logger().info(f"Executing trajectory {traj_name}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.goals[traj_name]

        goal.goal_time_tolerance = Duration(sec=5, nanosec=0)
    
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, traj_name))

    def execute_gripper_action(self, action):
        action = action.strip().lower()
        state = 1.0 if action == "open" else 0.0

        self.get_logger().info(f"Executing gripper action: {action}")

        # Publicar el comando para cambiar el estado del gripper
        cli = self.create_client(SetIO, '/io_and_status_controller/set_io')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetIO service not available!")
            return

        req = SetIO.Request()
        req.fun = 1  # Función para setear digital output
        req.pin = 0  # Pin del gripper
        req.state = state

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info(f"Gripper action '{action}' executed successfully.")
            self.resend_robot_program()
        else:
            self.get_logger().error(f"Failed to execute gripper action '{action}'.")
    
    def resend_robot_program(self):
        """Reenvía el programa del robot para asegurar que el controlador esté en un estado limpio."""
        self.get_logger().info("Re-sending robot program to reset controller state.")

        cli = self.create_client(Trigger, '/io_and_status_controller/resend_robot_program')

        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Resend program service not available!")
            return

        req = Trigger.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.done() and future.result().success:
            self.get_logger().info("Robot program resent successfully.")
            time.sleep(2)  # Dar tiempo para que el controlador se estabilice
        else:
            self.get_logger().error("Failed to resend robot program.")

        
    
    def goal_response_callback(self, future, traj_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            raise RuntimeError("Goal rejected :(")

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda future: self.get_result_callback(future, traj_name))


    def get_result_callback(self, future, traj_name):
        result = future.result().result
        self.get_logger().info(f"Done with result: {self.error_code_to_str(result.error_code)}")
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            # Verificar si la trayectoria actual tiene una acción de gripper
            
           
            points = self.trajectories[traj_name]
            for pt in points:
                # Verificar si hay acción del gripper
                if "gripper_action" in pt and pt["gripper_action"] != "none":
                    self.execute_gripper_action(pt["gripper_action"]) 

            time.sleep(2)  # Dar un pequeño retraso
            self.execute_next_trajectory()
        else:
            raise RuntimeError("Executing trajectory failed")
    
    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"


def main(args=None):
    rclpy.init(args=args)

    jtc_client = PruebaClient()
    try:
        rclpy.spin(jtc_client)
    except SystemExit:
        rclpy.logging.get_logger("empaque_prueba").info("Done")

    rclpy.shutdown()


if __name__ == "__main__":
    main()