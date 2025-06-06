import json
import uuid  # Libreria per generare UUID (identificatori univoci universali)


# Variabili globali
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


def load_profiles(filename="profiles.json"):
    """Carica i profili di calibrazione salvati da un file."""
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
            return data
    except FileNotFoundError:
        print(f"Errore: il file '{filename}' non è stato trovato.")
        return {}
    except json.JSONDecodeError:
        print(f"Errore: il file '{filename}' contiene JSON non valido.")
        return {}


def save_profiles(profile_name, filename="profiles.json"):
    """Salva i profili di calibrazione in un file."""
    if profile_name == "":
        profile_name = f"calibrazione_{uuid.uuid4().hex[:8]}" # Genera un nome casuale
    data = load_profiles()
    calibration_data = {'dx': calibration_dx, 'sx': calibration_sx}

    data[profile_name] = calibration_data

    with open(filename, 'w') as f:
        json.dump(data, f, indent=4)
        print(f"Profilo salvato con nome: {profile_name}")
        print(f"Con valori di calibrazione swipe a dx: '{calibration_dx}'\n E valori di calibrazione swipe a sx: '{calibration_sx}'")


def delete_profiles(profile_name, filename="profiles.json"):
    """Cancella un profilo di calibrazione dal file JSON."""
    data = load_profiles(filename)
    if profile_name in data:
        del data[profile_name]
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)