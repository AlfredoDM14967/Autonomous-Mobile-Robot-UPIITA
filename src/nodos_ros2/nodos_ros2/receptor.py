import cv2
import socket
import struct
import pickle

# Configuracion de la red
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.1.78', 8000))  # Cambia por la IP de tu Raspberry Pi

data = b""
payload_size = struct.calcsize("L")  # Para Python 2, usa "Q"

while True:
    # Recibe los datos del socket
    while len(data) < payload_size:
        data += client_socket.recv(4096)

    # Obtiene el tamano del frame
    packed_size = data[:payload_size]
    data = data[payload_size:]
    frame_size = struct.unpack("L", packed_size)[0]  # Para Python 2, usa "Q"

    # Recibe el frame completo
    while len(data) < frame_size:
        data += client_socket.recv(4096)

    # Deserializa el frame
    frame_data = data[:frame_size]
    data = data[frame_size:]
    frame = pickle.loads(frame_data)

    # Muestra el frame
    cv2.imshow("Video", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

client_socket.close()
cv2.destroyAllWindows()
