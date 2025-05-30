from picamera2 import Picamera2
from time import sleep

# Inicializar la cámara
picam2 = Picamera2()

# Configurar la cámara para capturar imágenes
picam2.configure(picam2.create_still_configuration())

# Iniciar la cámara
picam2.start()

# Esperar para estabilizar
sleep(2)

# Capturar una imagen
output_path = "imagen_capturada.jpg"
picam2.capture_file(output_path)

print(f"Imagen capturada y guardada en {output_path}")

# Detener la cámara
picam2.stop()
