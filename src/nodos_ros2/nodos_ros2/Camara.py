import cv2
import socket
import struct
import pickle
from picamera2 import Picamera2

# # Configuracion de la camara
# picam2 = Picamera2()

# # Aqui esta la configuracion correcta, utilizamos un metodo de configuracion, no llamamos al objeto
# config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
# picam2.configure(config)
# picam2.start()

# # Configuracion de la red
# server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server_socket.bind(('0.0.0.0', 8000))  # Escucha en todas las interfaces en el puerto 8000
# server_socket.listen(0)
# print("Esperando conexion...")
# connection, client_address = server_socket.accept()
# print(f"Conectado a: {client_address}")

# try:
#     while True:
#         # Captura un frame de la c�mara
#         frame = picam2.capture_array()

#         # Serializa el frame
#         data = pickle.dumps(frame)
#         # Envia el tamano del frame primero
#         message_size = struct.pack("L", len(data))  # Para Python 2, usa "Q"
#         connection.sendall(message_size + data)
# finally:
#     connection.close()
#     server_socket.close()


# Configuracion de la camara
picam2 = Picamera2()

# Configuracion correcta de la cámara
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
picam2.configure(config)
picam2.start()

# Configuracion de la red
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 8000))  # Escucha en todas las interfaces en el puerto 8000
server_socket.listen(0)
print("Esperando conexion...")

try:
    while True:
        print("Esperando nuevo cliente...")
        connection, client_address = server_socket.accept()
        print(f"Conectado a: {client_address}")

        try:
            while True:
                # Captura un frame de la cámara
                frame = picam2.capture_array()

                # Serializa el frame
                data = pickle.dumps(frame)
                # Envia el tamaño del frame primero
                message_size = struct.pack("L", len(data))  # Para Python 2, usa "Q"
                connection.sendall(message_size + data)
        except Exception as e:
            print(f"Error durante la transmisión: {e}")
        finally:
            print(f"Conexión cerrada con: {client_address}")
            connection.close()
finally:
    server_socket.close()

