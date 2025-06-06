import eel

from json_manager import load_profiles, delete_profiles
from main import main


# Inizializza Eel e punta alla cartella 'web' dove si trovano i file HTML, CSS e JS.
eel.init('web')


@eel.expose
def crea_nuovo_profilo(profile_name):
    print(f"Richiesta creazione nuovo profilo: {profile_name}")
    try:
        profiles = load_profiles()
        if profile_name in profiles:
            print(f"Profile '{profile_name}' already exists.")
            return "existing"
        else:
            print(f"Avvio la calibrazione per il profilo: {profile_name}. Segui le istruzioni nella console Python.")
            main(1, profile_name)
            print(f"Calibrazione per il profilo '{profile_name}' completata (o tentata).")
            return "created"
    except Exception as e:
        print(f"Error during profile creation or calibration: {e}")
        return "error"


@eel.expose
def get_profile_names():
    """Returns a list of names (keys) for all available profiles."""
    profiles = load_profiles()
    return list(profiles.keys())

@eel.expose
def carica_profilo_selezionato(profile_name): 
    """
    Function to load a specific profile by its name.
    """
    print(f"Request to load profile: {profile_name}")
    try:
        profiles = load_profiles()
        if profile_name in profiles:
            main(2, profile_name)
        else:
            print(f"Profile '{profile_name}' not found.")
            return None
    except Exception as e:
        print(f"Error loading profile '{profile_name}': {e}")
        return None



@eel.expose
def elimina_profilo(profile_name):
    """
    Python function to delete a specific profile.
    Deletes the corresponding key from the JSON file.
    """
    print(f"Delete profile request: {profile_name}")
    try:
        delete_profiles(profile_name)
        print(f"Profile '{profile_name}' deleted successfully.")
        return True
    except Exception as e:
        print(f"Error during deletion of profile '{profile_name}': {e}")
        return False


# Avvia l'applicazione Eel
eel.start('index.html', mode='chrome') 