import asyncio
from asyncua import Client

async def main():
    # Dirección del servidor OPC-UA del PLC
    opcua_url = "opc.tcp://192.168.0.230:4840"  # Cambia a la dirección de tu PLC

    # Nodo que quieres leer (Node ID)
    node_id = "ns=4;i=3"  # Cambia al Node ID de tu sensor

    # Conectar al servidor OPC-UA
    async with Client(opcua_url) as client:
        print("Conexión establecida con el servidor OPC-UA.")

        # Leer el valor del nodo
        node = client.get_node(node_id)
        value = await node.read_value()

        print(f"Valor leído del nodo {node_id}: {value}")

# Ejecutar el programa
if __name__ == "__main__":
    asyncio.run(main())