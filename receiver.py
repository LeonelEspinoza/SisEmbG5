import serial
from struct import pack, unpack

# Se configura el puerto y el BAUD_Rate
PORT = 'COM3'  # Esto depende del sistema operativo
BAUD_RATE = 115200  # Debe coincidir con la configuracion de la ESP32

# Se abre la conexion serial
ser = serial.Serial(PORT, BAUD_RATE, timeout = 1)

# Funciones
def send_message(message):
    """ Funcion para enviar un mensaje a la ESP32 """
    ser.write(message)

def receive_response():
    """ Funcion para recibir un mensaje de la ESP32 """
    response = ser.readline()
    return response

def receive_data():
    """ Funcion que recibe tres floats (fff) de la ESP32 
    y los imprime en consola """
    data = receive_response()
    print(data)
    data = unpack("fff", data)
    print(f'Received: {data}')
    return data

def send_end_message():
    """ Funcion para enviar un mensaje de finalizacion a la ESP32 """
    end_message = pack('4s', 'END\0'.encode())
    ser.write(end_message)

# Se envia el mensaje de inicio de comunicacion
message = pack('6s','BEGIN\0'.encode())
send_message(message)

# Se lee data por la conexion serial
counter = 0
while True:
    if ser.in_waiting > 0:
        try:
            message = receive_data()
        except:
            print('Error en leer mensaje')
            continue
        else: 
            counter += 1
            print(counter)
        finally:
            if counter == 10:
                print('Lecturas listas!')
                break


# Se envia el mensaje de termino de comunicacion
send_end_message()

ser.close()