import rclpy
from rclpy.node import Node
from custom_msgs.msg import OrderStatus
from asyncua import Client, ua
import asyncio


class OrderMonitor(Node):
    def __init__(self):
        super().__init__('order_monitor')

        # Declarar parámetros de OPC-UA en ROS2
        self.declare_parameter('opcua_url', 'opc.tcp://192.168.0.230:4840')  
        self.declare_parameter('ns', 4)  # Espacio de nombres OPC-UA (ns)
        self.declare_parameter('tipo_producto_i', 4)
        self.declare_parameter('presentacion_i', 6)
        self.declare_parameter('cantidad_presentacion_i', 10)
        self.declare_parameter('cantidad_pedido_i', 5)
        self.declare_parameter('tipo_pallet_i', 14)
        self.declare_parameter('cantidad_pallet_i', 18)

        # Obtener valores de parámetros
        self.opcua_url = self.get_parameter('opcua_url').value
        ns = self.get_parameter('ns').value
        tipo_producto_i = self.get_parameter('tipo_producto_i').value
        presentacion_i = self.get_parameter('presentacion_i').value
        cantidad_presentacion_i = self.get_parameter('cantidad_presentacion_i').value
        cantidad_pedido_i = self.get_parameter('cantidad_pedido_i').value
        tipo_pallet_i = self.get_parameter('tipo_pallet_i').value
        cantidad_pallet_i = self.get_parameter('cantidad_pallet_i').value

        # Crear cliente OPC-UA
        self.opcua_url = self.get_parameter('opcua_url').value 
        self.nodes = {
            'tipo_producto': f"ns={ns};i={tipo_producto_i}",
            'presentacion': f"ns={ns};i={presentacion_i}",
            'cantidad_presentacion': f"ns={ns};i={cantidad_presentacion_i}",
            'cantidad_pedido': f"ns={ns};i={cantidad_pedido_i}",
            'tipo_pallet': f"ns={ns};i={tipo_pallet_i}",
            'cantidad_pallet': f"ns={ns};i={cantidad_pallet_i}"
        }
        self.loop = asyncio.new_event_loop()
        self.opcua_client = Client(self.opcua_url)
        self.publisher = self.create_publisher(OrderStatus, '/order_status', 10)
        self.timer = self.create_timer(1.0, self.read_and_publish_data)
        
        self.get_logger().info("Conectando al servidor OPC-UA...")
        self.loop.run_until_complete(self.connect_opcua())

    async def connect_opcua(self):
        try:
            await self.opcua_client.connect()
            self.get_logger().info("Conexión OPC-UA exitosa.")
        except Exception as e:
            self.get_logger().error(f"Error al conectar al servidor OPC-UA: {e}")
            rclpy.shutdown()

    def read_and_publish_data(self):
        """Lee datos del servidor OPC-UA y los publica."""
        try:
            # Leer valores del servidor OPC-UA
            tipo_producto = self.loop.run_until_complete(self.read_value(self.nodes['tipo_producto']))
            presentacion = self.loop.run_until_complete(self.read_value(self.nodes['presentacion']))
            cantidad_presentacion = self.loop.run_until_complete(self.read_value(self.nodes['cantidad_presentacion']))
            cantidad_pedido = self.loop.run_until_complete(self.read_value(self.nodes['cantidad_pedido']))
            tipo_pallet = self.loop.run_until_complete(self.read_value(self.nodes['tipo_pallet'])) 
            cantidad_pallet = self.loop.run_until_complete(self.read_value(self.nodes['cantidad_pallet'])) 


            # Asegurarse de que 'presentacion' y 'cantidad_presentacion' sean listas de enteros
            if not isinstance(presentacion, list):
                presentacion = [int(presentacion)]
            else:
                presentacion = [int(x) for x in presentacion]

            if not isinstance(cantidad_presentacion, list):
                cantidad_presentacion = [int(cantidad_presentacion)]
            else:
                cantidad_presentacion = [int(x) for x in cantidad_presentacion]

            if not isinstance(tipo_pallet, list):
                tipo_pallet = [int( tipo_pallet)]
            else:
                tipo_pallet = [int(x) for x in tipo_pallet]
            
            if not isinstance(cantidad_pallet, list):
                cantidad_pallet = [int(cantidad_pallet)]
            else:
                cantidad_pallet = [int(x) for x in cantidad_pallet]

            # Crear y publicar mensaje
            msg = OrderStatus()
            msg.tipo_producto = int(tipo_producto) if isinstance(tipo_producto, (int, float)) else 0
            msg.presentacion = presentacion  # Lista de enteros
            msg.cantidad_presentacion = cantidad_presentacion  # Lista de enteros
            msg.cantidad_pedido = float(cantidad_pedido) if isinstance(cantidad_pedido, (int, float)) else 0.0
            msg.tipo_pallet = tipo_pallet
            msg.cantidad_pallet = cantidad_pallet
            
            self.publisher.publish(msg)
            self.get_logger().info(f"Publicado: {msg}")

        except Exception as e:
            self.get_logger().error(f"Error al leer o publicar datos: {e}")

    async def read_value(self, node_id):
        """Lee un valor de un nodo OPC-UA."""
        try:
            node = self.opcua_client.get_node(node_id)
            value = await node.read_value()
            return value
        except Exception as e:
            self.get_logger().error(f"Error al leer nodo {node_id}: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = OrderMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
