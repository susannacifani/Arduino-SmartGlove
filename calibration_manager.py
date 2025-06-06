import time
import eel

from json_manager import save_profiles

# Costanti
WINDOW_SIZE = 9
TIME_INTERVAL = 0.3
GYRO_Y_MIN_THRESHOLD = -15000
GYRO_Y_MAX_THRESHOLD = 15000

# Variabili globali
calibrated_dx = False
calibrated_sx = False
last_pressed = 0
start_calibration_window = False
calibration_window = []
num_swipes = 0
calibration_dx = {
    'accel_x_min': -10000, 'accel_x_max': 10000, 'accel_x_min_mean': None, 'accel_x_max_mean': None,
    'accel_y_min': None, 'accel_y_max': None, 'accel_y_min_mean': -17000, 'accel_y_max_mean': -12000,
    'accel_z_min': None, 'accel_z_max': None, 'accel_z_min_mean': None, 'accel_z_max_mean': None,
    'gyro_x_min': -22000, 'gyro_x_max': 18000, 'gyro_x_min_mean': None, 'gyro_x_max_mean': None,
    'gyro_y_min': None, 'gyro_y_max': None, 'gyro_y_min_mean': -17000, 'gyro_y_max_mean': -9000,
    'gyro_z_min': -10000, 'gyro_z_max': 10000, 'gyro_z_min_mean': None, 'gyro_z_max_mean': None
}
calibration_sx = {
    'accel_x_min': -10000, 'accel_x_max': 10000, 'accel_x_min_mean': None, 'accel_x_max_mean': None,
    'accel_y_min': None, 'accel_y_max': None, 'accel_y_min_mean': -17000, 'accel_y_max_mean': -12000,
    'accel_z_min': None, 'accel_z_max': None, 'accel_z_min_mean': None, 'accel_z_max_mean': None,
    'gyro_x_min': -22000, 'gyro_x_max': 18000, 'gyro_x_min_mean': None, 'gyro_x_max_mean': None,
    'gyro_y_min': None, 'gyro_y_max': None, 'gyro_y_min_mean': 15000, 'gyro_y_max_mean': 7000,
    'gyro_z_min': -10000, 'gyro_z_max': 10000, 'gyro_z_min_mean': None, 'gyro_z_max_mean': None
}


def calibration_swipe(stream, calibration_data):
    global num_swipes, last_pressed, start_calibration_window, calibration_window
    invalid_swipe = False
    if len(calibration_window) < WINDOW_SIZE:
        calibration_window.append(stream)
    elif len(calibration_window) == WINDOW_SIZE:
        sum_accel_x = 0
        sum_accel_y = 0
        sum_accel_z = 0
        # sum_gyro_x = 0
        # sum_gyro_y = 0
        sum_gyro_z = 0
        for row in calibration_window:
            print(row)

            # Check su gyro_x per ripetere
            if calibration_data['gyro_x_min'] is None or int(row[3]) < calibration_data['gyro_x_min'] or calibration_data['gyro_x_max'] is None or int(row[3]) > calibration_data['gyro_x_max']:
                invalid_swipe = True 
            if not invalid_swipe:
                # ACCEL: min, max
                if calibration_data['accel_x_min'] is None or int(row[0]) < calibration_data['accel_x_min']:
                    calibration_data['accel_x_min'] = int(row[0])
                if calibration_data['accel_x_max'] is None or int(row[0]) > calibration_data['accel_x_max']:
                    calibration_data['accel_x_max'] = int(row[0])
                if calibration_data['accel_y_min'] is None or int(row[1]) < calibration_data['accel_y_min']:
                    calibration_data['accel_y_min'] = int(row[1])
                if calibration_data['accel_y_max'] is None or int(row[1]) > calibration_data['accel_y_max']:
                    calibration_data['accel_y_max'] = int(row[1])
                if calibration_data['accel_z_min'] is None or int(row[2]) < calibration_data['accel_z_min']:
                    calibration_data['accel_z_min'] = int(row[2])
                if calibration_data['accel_z_max'] is None or int(row[2]) > calibration_data['accel_z_max']:
                    calibration_data['accel_z_max'] = int(row[2])
                # GYRO: min, max
                if calibration_data['gyro_z_min'] is None or int(row[5]) < calibration_data['gyro_z_min']:
                    calibration_data['gyro_z_min'] = int(row[5])
                if calibration_data['gyro_z_max'] is None or int(row[5]) > calibration_data['gyro_z_max']:
                    calibration_data['gyro_z_max'] = int(row[5])
                # ACCEL, GYRO: sums
                sum_accel_x += int(row[0])
                sum_accel_y += int(row[1])
                sum_accel_z += int(row[2])
                sum_gyro_z += int(row[5])

        if not invalid_swipe:
            # ACCEL: means
            mean_accel_x = sum_accel_x / WINDOW_SIZE
            if calibration_data['accel_x_min_mean'] is None or mean_accel_x < calibration_data['accel_x_min_mean']:
                calibration_data['accel_x_min_mean'] = round(mean_accel_x)
            if calibration_data['accel_x_max_mean'] is None or mean_accel_x > calibration_data['accel_x_max_mean']:
                calibration_data['accel_x_max_mean'] = round(mean_accel_x)
            mean_accel_y = sum_accel_y / WINDOW_SIZE
            if calibration_data['accel_y_min_mean'] is None or mean_accel_y < calibration_data['accel_y_min_mean']:
                calibration_data['accel_y_min_mean'] = round(mean_accel_y)
            if calibration_data['accel_y_max_mean'] is None or mean_accel_y > calibration_data['accel_y_max_mean']:
                calibration_data['accel_y_max_mean'] = round(mean_accel_y)
            mean_accel_z = sum_accel_z / WINDOW_SIZE
            if calibration_data['accel_z_min_mean'] is None or mean_accel_z < calibration_data['accel_z_min_mean']:
                calibration_data['accel_z_min_mean'] = round(mean_accel_z)
            if calibration_data['accel_z_max_mean'] is None or mean_accel_z > calibration_data['accel_z_max_mean']:
                calibration_data['accel_z_max_mean'] = round(mean_accel_z)
            # GYRO: means
            mean_gyro_z = sum_gyro_z / WINDOW_SIZE
            if calibration_data['gyro_z_min_mean'] is None or mean_gyro_z < calibration_data['gyro_z_min_mean']:
                calibration_data['gyro_z_min_mean'] = round(mean_gyro_z)
            if calibration_data['gyro_z_max_mean'] is None or mean_gyro_z > calibration_data['gyro_z_max_mean']:
                calibration_data['gyro_z_max_mean'] = round(mean_gyro_z)

            num_swipes = num_swipes + 1
            remaining = 5 - num_swipes
            if remaining != 0:
                eel.updateCalibrationStatus(f"{remaining} swipe rimanenti.")()
                eel.sleep(0.1)

        elif invalid_swipe:
            eel.updateCalibrationStatus("Swipe non valido, ripeti.")()
            eel.sleep(0.1)
            invalid_swipe = False

        last_pressed = time.time()
        start_calibration_window = False
        calibration_window = []

# --- Funzione per la fase di calibrazione ---
def run_calibration(sock, profile_name):
    global num_swipes, last_pressed, start_calibration_window, calibration_dx, calibration_sx, calibrated_dx, calibrated_sx
    num_swipes = 0
    eel.updateCalibrationStatus("Fai 5 swipe da destra a sinistra.")()
    eel.sleep(0.1)
    while True:
        if num_swipes == 5:
            if not calibrated_dx:
                calibrated_dx = True
                num_swipes = 0
                # print(f"Valori di calibrazione swipe a dx: '{calibration_dx}'")
                eel.updateCalibrationStatus("Fai 5 swipe da sinistra a destra.")()
                eel.sleep(0.1)
            elif calibrated_dx and not calibrated_sx:
                calibrated_sx = True
                # print(f"Valori di calibrazione swipe a sx: '{calibration_sx}'")
                save_profiles(profile_name)
                eel.updateCalibrationStatus(f"Calibrazione completata per il profilo '{profile_name}'!")()
                eel.sleep(0.1)
                break

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
            gyro_y = int(stream[4])

            if not calibrated_dx:
                if (gyro_y < GYRO_Y_MIN_THRESHOLD) and (current_time - last_pressed > TIME_INTERVAL):
                    if not start_calibration_window:
                        start_calibration_window = True
                if start_calibration_window:
                    calibration_swipe(stream, calibration_dx)
            if calibrated_dx and not calibrated_sx:
                if (gyro_y > GYRO_Y_MAX_THRESHOLD) and (current_time - last_pressed > TIME_INTERVAL):
                    if not start_calibration_window:
                        start_calibration_window = True
                if start_calibration_window:
                    calibration_swipe(stream, calibration_sx)

    return calibration_dx, calibration_sx
                
                