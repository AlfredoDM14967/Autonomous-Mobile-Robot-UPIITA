import smbus
import time

# Direcciones I2C
MPU9250_ADDR = 0x69
AK8963_ADDR = 0x0C

# Registros importantes
INT_PIN_CFG = 0x37
AK8963_CNTL = 0x0A
AK8963_XOUT_L = 0x03
AK8963_ST1 = 0x02
AK8963_ST2 = 0x09

# Configuración del modo continuo
AK8963_CONTINUOUS_MODE = 0x16

# Crear una instancia del bus I2C
bus = smbus.SMBus(1)

def write_register(device_addr, reg_addr, data):
    """Escribe un valor en un registro I2C."""
    bus.write_byte_data(device_addr, reg_addr, data)

def read_register(device_addr, reg_addr, length):
    """Lee un conjunto de bytes desde un registro I2C."""
    return bus.read_i2c_block_data(device_addr, reg_addr, length)

def configure_magnetometer():
    """Configura el magnetómetro AK8963 en modo continuo."""
    write_register(AK8963_ADDR, AK8963_CNTL, 0x00)  # Apagar el magnetómetro
    time.sleep(0.01)
    write_register(AK8963_ADDR, AK8963_CNTL, AK8963_CONTINUOUS_MODE)  # Modo continuo 2
    time.sleep(0.01)

def verify_magnetometer_mode():
    """Verifica que el magnetómetro esté en el modo continuo correcto."""
    cntl = read_register(AK8963_ADDR, AK8963_CNTL, 1)[0]
    if cntl != AK8963_CONTINUOUS_MODE:
        print(f"Advertencia: El modo del magnetómetro no es el esperado (CNTL={cntl:#02x}).")
    else:
        print("El magnetómetro está configurado correctamente.")

def read_magnetometer():
    """Lee los datos del magnetómetro y devuelve las componentes X, Y, Z."""
    data = read_register(AK8963_ADDR, AK8963_ST1, 8)  # Leer registros de estado y datos
    st1 = data[0]  # Registro de estado 1
    if not (st1 & 0x01):  # Verifica si los datos están listos
        return None, None, None  # Datos no válidos

    mag_x = int16(data[2] << 8 | data[1])
    mag_y = int16(data[4] << 8 | data[3])
    mag_z = int16(data[6] << 8 | data[5])

    st2 = data[7]  # Registro de estado 2
    if st2 & 0x08:  # Verifica si hay desbordamiento
        print("Error: Datos inválidos (overflow)")
        return None, None, None

    return mag_x, mag_y, mag_z

def int16(value):
    """Convierte un valor de 16 bits a un entero con signo."""
    if value > 32767:
        value -= 65536
    return value

def setup():
    """Configura el magnetómetro para la lectura."""
    write_register(MPU9250_ADDR, INT_PIN_CFG, 0x02)  # Habilitar bypass I2C
    time.sleep(0.01)
    configure_magnetometer()
    verify_magnetometer_mode()
    print("Magnetómetro activado y configurado en modo continuo.")

def loop():
    """Bucle principal que lee e imprime los datos del magnetómetro."""
    try:
        while True:
            mag_x, mag_y, mag_z = read_magnetometer()
            if mag_x is not None:
                print(f"MagX: {mag_x}, MagY: {mag_y}, MagZ: {mag_z}")
            else:
                print("Esperando datos del magnetómetro...")

            time.sleep(0.1)  # Pausa de 100 ms
    except KeyboardInterrupt:
        print("Finalizando lectura.")

if __name__ == "__main__":
    setup()
    loop()


# import smbus
# import time

# # Direcciones I2C
# MPU6050_ADDR = 0x68  # Dirección de la IMU
# HMC5883L_ADDR = 0x1E # Dirección del magnetómetro (interno)

# # Inicializar el bus I2C
# bus = smbus.SMBus(1)

# def init_mpu6050():
#     """
#     Configura el MPU6050 para habilitar el acceso al magnetómetro.
#     """
#     # Despertar el MPU6050 (saca del modo de suspensión)
#     bus.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)

#     # Habilitar el bypass I2C para acceder al magnetómetro directamente
#     bus.write_byte_data(MPU6050_ADDR, 0x37, 0x02)
#     print("MPU6050 configurado en modo bypass I2C.")

# def init_hmc5883l():
#     """
#     Configura el HMC5883L para modo continuo.
#     """
#     # Registro A: 8 promedios, 15 Hz, modo normal
#     bus.write_byte_data(HMC5883L_ADDR, 0x00, 0x70)
#     # Registro B: Ganancia de ±1.3 gauss
#     bus.write_byte_data(HMC5883L_ADDR, 0x01, 0xA0)
#     # Registro de modo: Modo continuo
#     bus.write_byte_data(HMC5883L_ADDR, 0x02, 0x00)

#     print("HMC5883L configurado correctamente.")

# def read_raw_data(register):
#     """
#     Lee dos bytes de datos desde un registro del HMC5883L y los convierte en un entero con signo.
#     """
#     high = bus.read_byte_data(HMC5883L_ADDR, register)    # Byte alto
#     low = bus.read_byte_data(HMC5883L_ADDR, register + 1) # Byte bajo
#     value = (high << 8) | low                             # Combina alto y bajo

#     # Convierte a entero con signo si es necesario
#     if value > 32767:
#         value -= 65536
#     return value

# # Configurar el MPU6050 y el HMC5883L
# init_mpu6050()
# init_hmc5883l()

# print("Leyendo valores crudos del HMC5883L...")
# while True:
#     try:
#         # Leer datos de los ejes X, Y y Z
#         mag_x = read_raw_data(0x03)
#         mag_z = read_raw_data(0x05)
#         mag_y = read_raw_data(0x07)

#         # Imprimir los valores crudos
#         print(f"Magnetómetro: X={mag_x}, Y={mag_y}, Z={mag_z}")
#         time.sleep(1)
#     except Exception as e:
#         print(f"Error al leer datos: {e}")
#         break


# from gpiozero import LED
# from time import sleep

# led = LED(17)  # Usa el pin GPIO 17

# while True:
#     led.on()
#     print("LED encendido")
#     # sleep(1)
#     # led.off()
#     # print("LED apagado")
#     # sleep(1)

# # a = True
# # b = not(a)
# # print(b)