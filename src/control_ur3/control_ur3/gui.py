#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tkinter as tk
import subprocess
import signal
import os


class Gui(Node):  # MODIFICAR NOMBRE SI ES NECESARIO
    def __init__(self):
        super().__init__("gui_control_ur3")

        self.crearVentana()

    def empaquetar_4_unidades(self):
        comando4 = 'ros2 run control_ur3 jtc_client'
        self.get_logger().info("Lanzando empaquetado de 4 unidades")

        self.resultado = subprocess.Popen([comando4], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid)

    def empaquetar_6_unidades(self):
        comando6 = 'ros2 launch control_ur3 empaquetado_6_unidades.launch.py'
        self.get_logger().info("Lanzando empaquetado de 6 unidades")

        self.resultado = subprocess.Popen([comando6], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid)

    def empaquetar_12_unidades(self):
        comando12 = 'ros2 launch control_ur3 empaquetado_12_unidades.launch.py'
        self.get_logger().info("Lanzando empaquetado de 12 unidades")

        self.resultado = subprocess.Popen([comando12], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid)
    def go_home(self):
        comando6 = 'ros2 run control_ur3 go_home'
        self.get_logger().info("Lanzando go home")
        self.resultado = subprocess.Popen([comando6], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid)

    

    def salir(self):
        os.killpg(os.getpgid(self.resultado.pid), signal.SIGINT)

    def crearVentana(self):
        ventana = tk.Tk()
        ventana.title("Control de Empaquetado UR3")

        # Texto superior
        etiqueta_superior = tk.Label(ventana, text="Control de Empaquetado - UR3")
        etiqueta_superior.pack(pady=10)

        ventana.geometry("600x400")

        canvas = tk.Canvas(ventana, height=400, width=600)
        canvas.pack()

        btn_empaquetar_4 = tk.Button(canvas, text="Empaquetar 4 Unidades", command=self.empaquetar_4_unidades, height=5, width=20)
        btn_empaquetar_6 = tk.Button(canvas, text="Empaquetar 6 Unidades", command=self.empaquetar_6_unidades, height=5, width=20)
        btn_empaquetar_12 = tk.Button(canvas, text="Empaquetar 12 Unidades", command=self.empaquetar_12_unidades, height=5, width=20)
        btn_go_home = tk.Button(canvas, text="Go_home", command=self.go_home, height=5, width=20)
        btn_salir = tk.Button(canvas, text="Terminar proceso", command=self.salir, height=5, width=20)

        canvas.create_window(150, 100, window=btn_empaquetar_4)
        canvas.create_window(450, 100, window=btn_empaquetar_6)
        #canvas.create_window(300, 250, window=btn_empaquetar_12)
        canvas.create_window(300, 250, window=btn_go_home)
        canvas.create_window(300, 350, window=btn_salir)

        ventana.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = Gui()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
