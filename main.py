import pyautogui
import time
import eel
import socket

from calibration_manager import run_calibration
from json_manager import load_profiles, save_profiles

# Costanti
WINDOW_SIZE = 9
TIME_INTERVAL = 0.3
FLEX_TIME_INTERVAL = 1
GYRO_Y_MIN_THRESHOLD = -15000
GYRO_Y_MAX_THRESHOLD = 15000
screenWidth, screenHeight = pyautogui.size()
centerX = screenWidth / 2
centerY = screenHeight / 2

# Variabili globali
sock = None 
last_pressed = 0
start_right_window = False
start_left_window = False
swipe_window = []
invalid_gesture = False
# Dizionario per storare dati relativi alllo swipe da dx a sx (DX)
calibration_dx = {
    'accel_x_min': -10000, 'accel_x_max': 10000, 'accel_x_min_mean': None, 'accel_x_max_mean': None,
    'accel_y_min': None, 'accel_y_max': None, 'accel_y_min_mean': -17000, 'accel_y_max_mean': -12000,
    'accel_z_min': None, 'accel_z_max': None, 'accel_z_min_mean': None, 'accel_z_max_mean': None,
    'gyro_x_min': -22000, 'gyro_x_max': 18000, 'gyro_x_min_mean': None, 'gyro_x_max_mean': None,
    'gyro_y_min': None, 'gyro_y_max': None, 'gyro_y_min_mean': -17000, 'gyro_y_max_mean': -9000,
    'gyro_z_min': -10000, 'gyro_z_max': 10000, 'gyro_z_min_mean': None, 'gyro_z_max_mean': None
}
# Dizionario per storare dati relativi alllo swipe da sx a dx (SX)
calibration_sx = {
    'accel_x_min': -10000, 'accel_x_max': 10000, 'accel_x_min_mean': None, 'accel_x_max_mean': None,
    'accel_y_min': None, 'accel_y_max': None, 'accel_y_min_mean': -17000, 'accel_y_max_mean': -12000,
    'accel_z_min': None, 'accel_z_max': None, 'accel_z_min_mean': None, 'accel_z_max_mean': None,
    'gyro_x_min': -22000, 'gyro_x_max': 18000, 'gyro_x_min_mean': None, 'gyro_x_max_mean': None,
    'gyro_y_min': None, 'gyro_y_max': None, 'gyro_y_min_mean': 15000, 'gyro_y_max_mean': 7000,
    'gyro_z_min': -10000, 'gyro_z_max': 10000, 'gyro_z_min_mean': None, 'gyro_z_max_mean': None
}
scroll_up = False


def send_vibration_command(command_char):
    global sock
    ARDUINO_IP = "172.20.10.4"  
    ARDUINO_PORT = 8889  # port for sending commands to Arduino
    try:
        if sock:
            sock.sendto(command_char.encode('utf-8'), (ARDUINO_IP, ARDUINO_PORT))
            print(f"Comando '{command_char}' inviato ad Arduino a {ARDUINO_IP}:{ARDUINO_PORT}.")
        else:
            print("Socket UDP non inizializzato. Impossibile inviare comando.")
    except Exception as e:
        print(f"Errore nell'invio del comando UDP: {e}")


def swipe(stream, calibration_data):
    global last_pressed, start_right_window, start_left_window, swipe_window, invalid_gesture
    if len(swipe_window) < WINDOW_SIZE:
        swipe_window.append(stream)
    elif len(swipe_window) == WINDOW_SIZE:
        pos_accelx = False
        sum_accy = 0
        mean_accy = 0
        neg_accelz = False
        sum_gyroy = 0
        mean_gyroy = 0
        # Controllo ogni riga della finestra e vedo se tutte sono sotto la soglia o sforano
        for row in swipe_window:
            print(row)

            # Inserisci qui i check (acc/gyro)

            # ACCEL_X
            if (int(row[0]) < calibration_data['accel_x_min'] or int(row[0]) > calibration_data['accel_x_max']) and (invalid_gesture == False): # se una riga sfora, il gesto diventa invalido
                invalid_gesture = True    
                print("INVALID ACCEL X")
            if (int(row[0]) > -1000):
                pos_accelx = True

            # ACCEL_Y: se l'accelerazione della mano verso alto/basso è eccessiva, gesto invalido
            # procedo facendo la somma di ogni valore per poi calcolarne la media e fare dopo i check (così non è sensibile agli outliers)
            try: 
                sum_accy += int(row[1])
            except Exception as e:
                print(f"Errore nella somma accel_Y: {e}")
            # ACCEL_Z
            if (int(row[2]) < 0):
                neg_accelz = True

            # GYRO_X: rotazione oraria/antioraria
            if (int(row[3]) < calibration_data['gyro_x_min'] or int(row[3]) > calibration_data['gyro_x_max']) and (invalid_gesture == False): # se una riga sfora, il gesto diventa invalido
                invalid_gesture = True    
                print("INVALID GYRO X")

            # GYRO_Y: se la velocità di sliding è eccessivamente alta o bassa, gesto invalido
            # procedo facendo la somma dei valori e poi dei check sulla media, limito gyro_y 
            if calibration_data == calibration_dx:
                if int(row[4]) < 0:
                    try: 
                        sum_gyroy += int(row[4])
                    except Exception as e:
                        print(f"Errore nella somma gyro_Y 1: {e}")
            elif calibration_data == calibration_sx:
                if int(row[4]) > 0:
                    try: 
                        sum_gyroy += int(row[4])
                    except Exception as e:
                        print(f"Errore nella somma gyro_Y 2: {e}")

            # GYRO_Z: rotazione verso l'alto/basso
            if (int(row[5]) < calibration_data['gyro_z_min']  or int(row[5]) > calibration_data['gyro_z_max']) and (invalid_gesture == False): # se una riga sfora, il gesto diventa invalido
                invalid_gesture = True
                print("INVALID GYRO Z")

            # FLEX
            # if (int(row[6]) > 600) and (invalid_gesture == False): # se una riga sfora, il gesto diventa invalido
            #     invalid_gesture = True    
            #     print("INVALID FLEX")


        # CHECK ACCEL_X
        if not pos_accelx:
            invalid_gesture = True
            print("INVALID ACCEL X")

        # CHECK ACCEL_Y
        mean_accy = sum_accy / WINDOW_SIZE
        try: 
            min_mean = (calibration_data['accel_y_min'] + calibration_data['accel_y_min_mean']) / 2
            max_mean = (calibration_data['accel_y_max'] + (calibration_data['accel_y_max_mean'] * 1.6)) / 2
        except Exception as e:
            print(f"Errore calcolo medie accel Y: {e}")
        if (mean_accy < min_mean or mean_accy > max_mean): # di solito oscilla intorno ai -15.000. [-17.000, -12.000]
            invalid_gesture = True
            print("INVALID ACCEL Y")
        # print(min_mean)
        # print(max_mean)
        # print(mean_accy)

        # CHECK ACCEL_Z
        if not neg_accelz:
            invalid_gesture = True
            print("INVALID ACCEL Z")


        # CHECK GYRO_Y
        mean_gyroy = sum_gyroy / WINDOW_SIZE
        # print(mean_gyroy)
        if calibration_data == calibration_dx:
            if (mean_gyroy < calibration_data['gyro_y_min_mean'] or mean_gyroy > calibration_data['gyro_y_max_mean']): # di solito oscilla intorno ai -12.000
                invalid_gesture = True
                print("INVALID GYRO Y: dx")
        elif calibration_data == calibration_sx:
            if (mean_gyroy > calibration_sx['gyro_y_min_mean'] or mean_gyroy < calibration_sx['gyro_y_max_mean']): # di solito oscilla intorno ai -12.000
                invalid_gesture = True
                print("INVALID GYRO Y: sx")



        if (invalid_gesture == False): # infine se il gesto è valido, procedo con l'attivazione
            if calibration_data == calibration_dx:
                pyautogui.press('right')
                send_vibration_command('S')
                last_pressed = time.time()
                print("SPAZZATA A DX")
                # eel.updateCalibrationStatus("SPAZZATA A DX")() # Call JS function
                # eel.sleep(0.1) # Give JS a moment to update
            elif calibration_data == calibration_sx:
                pyautogui.press('left')
                send_vibration_command('S')
                last_pressed = time.time()
                print("SPAZZATA A SX")
                # eel.updateCalibrationStatus("SPAZZATA A SX")() # Call JS function
                # eel.sleep(0.1) # Give JS a moment to update

        start_right_window = False
        start_left_window = False
        swipe_window = []
        invalid_gesture = False


# --- Start ---
def start(choice, profile_name):
    global sock, calibration_dx, calibration_sx

    # UDP setup
    UDP_IP = "0.0.0.0"       # Listen on all available interfaces
    UDP_PORT = 8888          # Port to listen on for incoming data from Arduino

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        # Set a timeout for the socket to prevent blocking indefinitely
        sock.settimeout(1) 
        print(f"In ascolto su {UDP_IP}:{UDP_PORT}...")
    except socket.error as e:
        print(f"Errore nell'apertura del socket UDP: {e}")
        exit()


    if choice == 1:
        new_calibration_data_dx, new_calibration_data_sx = run_calibration(sock, profile_name)
        calibration_dx = new_calibration_data_dx
        calibration_sx = new_calibration_data_sx
        save_profiles(profile_name, calibration_dx, calibration_sx)
    elif choice == 2:
        profiles = load_profiles()
        calibration_dx = profiles[profile_name]['dx']
        calibration_sx = profiles[profile_name]['sx']
        print(f"\nSelezionato il profilo '{profile_name}'.")
        print(f"Con valori di calibrazione swipe a dx: '{calibration_dx}'\n E valori di calibrazione swipe a sx: '{calibration_sx}'")




# --- Update ---
def update():
    global sock, last_pressed, start_right_window, start_left_window, swipe_window, scroll_up
    while True:
        try:
            data, addr = sock.recvfrom(1024)  # Buffer size
            line = data.decode('utf-8').rstrip()
            stream = [n for n in line.split(',')]
            # Check necessario per accedere a tutti i 7 valori dei sensori per non avere errori
            if len(stream) == 7:
                # Formattazione dei valori di accelerometro (x, y, z), giroscopio (x, y, z) e flex sensor su terminale
                length = 8
                formatted_line = ""
                for element in stream:
                    formatted_line += f"{element:<{length}}"
                print(formatted_line)

                current_time = time.time()

                accel_x = int(stream[0])
                accel_y = int(stream[1])
                accel_z = int(stream[2])
                # gyro_x = int(stream[3])
                gyro_y = int(stream[4])
                # gyro_z = int(stream[5])
                flex = int(stream[6])


                # CHECK SLIDING DA DX A SX
                # Se c'è una velocità di rotazione plausibile per attivare lo sliding e il tempo ha superato i limiti corretti,
                # allora registro una finestra di 10 valori da esaminare
                if (gyro_y < GYRO_Y_MIN_THRESHOLD) and (current_time - last_pressed > TIME_INTERVAL):
                    if not start_right_window:
                        start_right_window = True
                if start_right_window and (not start_left_window):
                    current_time = last_pressed = time.time()
                    swipe(stream, calibration_dx)
                # CHECK SLIDING DA SX A DX (SX)
                elif (gyro_y > GYRO_Y_MAX_THRESHOLD) and (current_time - last_pressed > TIME_INTERVAL):
                    if not start_left_window:
                        start_left_window = True
                if (not start_right_window) and start_left_window:
                    current_time = last_pressed = time.time()
                    swipe(stream, calibration_sx)
                # Check play/pause video (BARRA SPAZIATRICE)
                elif (flex) > 900 and (current_time - last_pressed > FLEX_TIME_INTERVAL):
                    current_time = last_pressed = time.time()
                    # pyautogui.press('space') # Implementazione play/pause con barra spaziatrice, ma non funziona per video nelle presentazioni
                    current_x, current_y = pyautogui.position()
                    if (current_x == centerX) and (current_y == centerY):
                        pyautogui.moveTo(centerX+5, centerY+5, duration=0.2)
                    else:
                        pyautogui.moveTo(centerX, centerY, duration=0.2)
                    pyautogui.click()
                    send_vibration_command('P')
                # Check sliding up/down (scroll)
                elif (-4000 < accel_y < 4000) and (8000 < accel_z < 17000) and (flex) < 800 and (current_time - last_pressed > 0.1):
                    if (accel_x < -10000):
                        pyautogui.scroll(150)
                        send_vibration_command('U')
                        current_time = last_pressed = time.time()
                    elif (accel_x > 10000):
                        pyautogui.scroll(-150)
                        send_vibration_command('U')
                        current_time = last_pressed = time.time()
        except socket.timeout:
            # No data received within the timeout period, just continue
            pass
        except Exception as e:
            print(f"Errore nella ricezione o elaborazione dati UDP: {e}")
                



                    

def main(choice, profile_name):
    start(choice, profile_name)
    try:
        while True:
            update()
    except KeyboardInterrupt:
        print("\nProgramma terminato.")
    finally:
        if sock: # Close the UDP socket if it was opened
            sock.close()
            print("Socket UDP chiuso.")