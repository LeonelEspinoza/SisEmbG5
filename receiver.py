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

def receive_data(msg_size):
    """ Funcion que recibe tres floats (fff) de la ESP32 
    y los imprime en consola """
    #data = receive_response()
    #print(f"<receive_data> msg_size: {msg_size}")
    data = ser.read(4*msg_size)
    #data = ser.readline()
    #print(data)
    #print(f"<receive_data> calling unpack with {msg_size}f")
    data = unpack(f"{msg_size}f", data)
    #print(f'Received: {data}')
    return data

def send_end_message():
    """ Funcion para enviar un mensaje de finalizacion a la ESP32 """
    end_message = pack('4s', 'END\0'.encode())
    ser.write(end_message)

def make_f(size):
    strf = "f" * size
    return strf


wSize = 0


tmp = 0
while True:
    if ser.in_waiting > 0:
        try:
            response1 = receive_response()
            print(response1)
            response1 = unpack("s", response1)
        except:
            continue
        finally:
            if str(response1).rfind('OK setup') == -1:
                continue
            print("sending BEGIN")
            break
    
    tmp+=1
    if tmp > 100000:
        tmp =0
        print("intenta apretando el boton de la ESP")

# Se envia el mensaje de inicio de comunicacion
message = pack('6s','BEGIN\0'.encode())
send_message(message)

#Try to recive window size
while True:
    if ser.in_waiting > 0:
        try:
            #wSize_bytes = receive_response()
            wSize_bytes = ser.read(4)
            print(f"wSize_bytes: {wSize_bytes}")
            wSize_bytes = unpack("i", wSize_bytes)
            wSize = int.from_bytes(wSize_bytes)
            print(wSize)
        except:
            continue
        finally:
            print(wSize_bytes)
            print('Set Window Size to ' + str(wSize))
            if wSize < 1:
                print("wSize < 1")
                continue
            message = pack('3s','OK\0'.encode())
            send_message(message)
            break

while(True):
    while(True):
        print('''Choose one of the following:
            [1] Get Data Window
            [2] Change Window Size
            [3] Finish Conection
            ''')
        select = input("Your selection: ")

        if(select == "1" or select == "2" or select == "3"):
            break
        else:
            print("invalid selecion")

    send_message(pack('2s',select.encode()))

    if select == "1":
        #Set msg_size
        msg_size = (wSize * 4) + 4 
        meassure_data=[0]
        # Try to Recive sensor data
        while True:
            if ser.in_waiting > 0:
                try:
                    meassure_data = receive_data(msg_size)
                except:
                    continue
                finally:
                    print('Data recived \n')
                    break

        temp_array = [0 for i in range(wSize)]
        press_array = [0 for i in range(wSize)]
        hum_array = [0 for i in range(wSize)]
        CO_array = [0 for i in range(wSize)]
        i=0
        while(i<wSize):
            temp_array[i]   =meassure_data[i]
            press_array[i]  =meassure_data[i+wSize*1]
            hum_array[i]    =meassure_data[i+wSize*2]
            CO_array[i]     =meassure_data[i+wSize*3]
            i+=1
        
        temp_RMS = meassure_data[wSize*4+0]
        press_RMS = meassure_data[wSize*4+1]
        hum_RMS = meassure_data[wSize*4+2]
        CO_RMS = meassure_data[wSize*4+3]
        
        print(f"Temperature data: {temp_array}")
        print(f"Temperature RMS: {temp_RMS} \n")

        print(f"Preasure data: {press_array}")
        print(f"Preasure RMS: {press_RMS} \n")
        
        print(f"Humidity data: {hum_array}")
        print(f"Humidity RMS: {hum_RMS} \n")

        print(f"CO data: {CO_array}")
        print(f"CO RMS: {CO_RMS} \n")
 
        msg_size = 20
        while True:
            if ser.in_waiting > 0:
                try:
                    message_peaks = receive_data(msg_size)
                except:
                    continue
                finally:
                    print('Peaks recived \n')
                    break

        print(f'Peaks temp: {message_peaks[0:5]} ')
        print(f'Peaks press: {message_peaks[5:10]} ')
        print(f'Peaks hum: {message_peaks[10:15]} ')
        print(f'Peaks co: {message_peaks[15:20]} \n')

        msg_size = (wSize * 2 * 4)
        while True:
            if ser.in_waiting > 0:
                try:
                    message_FFT = receive_data(msg_size)
                except:
                    continue
                finally:
                    print('FFT recived \n')
                    break
                    
        print(f'FFT temp: {message_FFT[0:wSize*2]} ')
        print(f'FFT press: {message_FFT[wSize*2:wSize*4]} ')
        print(f'FFT hum: {message_FFT[wSize*4:wSize*6]} ')
        print(f'FFT CO: {message_FFT[wSize*6:wSize*8]} \n')
        

    if select == "2":
        
        while(True):
            windows_size = input("Enter window size (1 - 200):")
            try:
                windows_size = min(200,max(1,int(windows_size)))
            except:
                print("invalid input")
                continue
            else:
                break

        wSize = windows_size
        
        if windows_size < 10:
            windows_size = "00" + str(windows_size)
        elif windows_size < 100:
            windows_size = "0" + str(windows_size)
        else:
            windows_size = str(windows_size)
        #manda tamaño de ventana
        send_message(pack('3s',windows_size.encode()))
        

    if select == "3":
        break

ser.close()