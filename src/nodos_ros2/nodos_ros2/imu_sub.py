import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import MagneticField
from std_msgs.msg import String
import os

class MagnetoPlot(Node):
    def __init__(self):
        super().__init__('MagnetoPlot')
        self.sub_ = self.create_subscription(MagneticField, "/mag/out", self.listener_callback, qos_profile=qos_profile_sensor_data)
        # self.ard_sub_ = self.create_subscription(String, "Comandos", self.ardu_callback, 10)
        self.mx = []
        self.my = []

          # Ruta del archivo
        self.file_path = "/ros_ws/TT/MagnetometroHMC3.txt"
        # self.file_path_2 = "/ros_ws/TT/V.txt"

        # Crear directorio si no existe
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)

        with open(self.file_path, 'w') as file:
            file.write("MagX\tMagY\n")  # Encabezados
        
        # os.makedirs(os.path.dirname(self.file_path_2), exist_ok=True)

        # with open(self.file_path, 'w') as file:
        #     file.write("Voltaje\n")  # Encabezados

    def listener_callback(self, msg):
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        self.mx.append(mx)
        self.my.append(my)

        # Guardar los datos en el archivo
        with open(self.file_path, 'a') as file:
            file.write(f"{mx}\t{my}\n")

    # def ardu_callback(self, msg):
    #     voltage = msg.data

    #     # Guardar los datos en el archivo
    #     with open(self.file_path_2, 'a') as file:
    #         file.write(f"{voltage}\n")

def main():
    rclpy.init()
    mag_plot = MagnetoPlot()
    rclpy.spin(mag_plot)
    mag_plot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
