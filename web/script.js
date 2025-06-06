// Funzione per mostrare messaggi all'utente
// function showMessage(msg, type = 'info') { // duration in ms, default 5 sec
//     const messageElement = document.getElementById('message');
//     messageElement.innerText = msg;
//     if (type === 'error') {
//         messageElement.style.color = '#e74c3c';
//     } else if (type === 'success') {
//         messageElement.style.color = '#27ae60';
//     } else { // info
//         messageElement.style.color = '#555';

//     }
    
//     messageElement.style.display = 'block'; // <--- RENDE IL MESSAGGIO VISIBILE
// }



function showMessage(msg, type = 'info') {
    const messageElement = document.getElementById('message');
    // Rimuovi tutte le classi di tipo precedentemente aggiunte
    messageElement.classList.remove('info', 'error', 'success');
    messageElement.innerText = msg;
    // Aggiungi la classe corretta in base al tipo
    if (type === 'error') {
        messageElement.classList.add('error');
    } else if (type === 'success') {
        messageElement.classList.add('success');
    } else { // info
        messageElement.classList.add('info'); // Aggiungiamo sempre una classe per coerenza
    }
    // Rende il messaggio visibile
    messageElement.style.display = 'block';
}

// Nuova funzione per nascondere il messaggio
function hideMessage() {
    const messageElement = document.getElementById('message');
    messageElement.innerText = ''; // Pulisci il testo
    messageElement.style.display = 'none'; // <--- NASCONDE IL MESSAGGIO
    messageElement.style.borderColor = 'transparent'; // Rimuovi il bordo quando nascosto
    messageElement.style.backgroundColor = 'transparent'; // Rimuovi lo sfondo quando nascosto
}


function hideElement(elementId) {
    const element = document.getElementById(elementId);
    if (element) {
        element.style.display = 'none';
    }
}

async function nuovoProfilo() {
    const newProfileNameInput = document.getElementById('newProfileName');
    const profileName = newProfileNameInput.value.trim();

    if (!profileName) {
        showMessage("Please enter a name for the new profile.", 'error');
        return;
    }

    // Nasconde il bottone "Back" una volta che la creazione del profilo inizia
    hideElement('backToHomeBtn');

    showMessage(`Creating new profile: ${profileName}. Please wait, calibration is starting in the Python console...`, 'info');
    try {
        const success = await eel.crea_nuovo_profilo(profileName)();
        if (success === "created") {
            showMessage(`Profile '${profileName}' created and calibrated successfully!`, 'success');
            newProfileNameInput.value = ''; // Clear input
            // Dopo aver creato un nuovo profilo, ripopola il dropdown per includerlo
            populateProfileDropdown();
        } else if (success === "existing") {
            showMessage(`Profile '${profileName}' already exists.`, 'error');
        } else {
            showMessage("Error during profile creation.", 'error');
        }
    } catch (error) {
        console.error("Error calling eel.crea_nuovo_profilo:", error);
        showMessage("Communication error with the backend.", 'error');
    }
}

async function populateProfileDropdown() {
    const profileSelect = document.getElementById('profileSelect');
    profileSelect.innerHTML = ''; // Pulisci tutte le opzioni esistenti

    // Crea e aggiungi l'opzione placeholder SEMPRE per prima
    const placeholderOption = document.createElement('option');
    placeholderOption.value = ""; // Valore vuoto o un identificatore specifico
    placeholderOption.innerText = "-- Select a profile --";
    placeholderOption.disabled = true; // Non selezionabile una volta cambiata la selezione
    placeholderOption.selected = true; // È l'opzione predefinita visibile all'avvio
    profileSelect.appendChild(placeholderOption);

    try {
        const profileNames = await eel.get_profile_names()(); // Chiamata alla funzione Python
        if (profileNames && profileNames.length > 0) {
            profileNames.forEach(name => {
                const option = document.createElement('option');
                option.value = name;
                option.innerText = name;
                profileSelect.appendChild(option);
            });
            profileSelect.disabled = false; // Abilita il dropdown se ci sono profili
            // IMPORTANTE: NON IMPOSTARE profileSelect.value = profileNames[0]; QUI
            // Lascia che l'opzione placeholder rimanga selezionata per default
        } else {
            // Se non ci sono profili, assicurati che il placeholder sia l'unica opzione e sia selezionata
            // e disabilita il dropdown per evitare selezioni indesiderate.
            profileSelect.disabled = true; // Disabilita il dropdown
            placeholderOption.innerText = "No profile available"; // Cambia testo del placeholder
            // placeholderOption.value rimane "", disabled e selected.
            showMessage("No profiles found.", 'error');
        }
    } catch (error) {
        console.error("Error retrieving profile names:", error);
        showMessage("Error loading profiles into the dropdown menu.", 'error');
        profileSelect.disabled = true; // Disabilita in caso di errore
        placeholderOption.innerText = "Loading error"; // Cambia testo del placeholder
        // placeholderOption.value rimane "", disabled e selected.
    }
}

async function loadSelectedProfile() {
    const profileSelect = document.getElementById('profileSelect');
    const selectedProfileName = profileSelect.value;
    const selectButton = document.querySelector('.profile-selection button'); // Seleziona il bottone "Select"
    const deleteButton = document.querySelector('.button-container button[onclick="deleteProfile()"]'); // Assicurati di selezionare il bottone "Delete" se esiste

    // Nascondi eventuali messaggi precedenti
    // hideMessage();

    // Validazione iniziale del placeholder (importante!)
    if (!selectedProfileName || selectedProfileName === "-- Select a profile --" || selectedProfileName === "No profile available" || selectedProfileName === "Loading error") {
        showMessage("Please select a valid profile to load.", 'error');
        return;
    }

    // A questo punto, la selezione è valida. Disabilita gli elementi.
    profileSelect.disabled = true; // Disabilita il menu a discesa
    selectButton.disabled = true;  // Disabilita il pulsante "Select"
    if (deleteButton) { // Disabilita il pulsante "Delete" se esiste
        deleteButton.disabled = true;
    }
    hideElement('backToHomeBtn'); // Questo nasconde il bottone "Back to Home", se lo desideri

    showMessage(`Profile '${selectedProfileName}' loaded successfully!`, 'info');
    try {
        const profileData = await eel.carica_profilo_selezionato(selectedProfileName)();
        if (profileData) {
            const parsedData = JSON.parse(profileData);
            if (parsedData.status === "success") {
                showMessage(`Profile '${selectedProfileName}' loaded successfully!`, 'success');
                console.log("Loaded profile data:", parsedData);
            } else {
                showMessage(`No data found for profile '${selectedProfileName}' or an error occurred during loading.`, 'error');
                // In caso di errore nel caricamento, potresti voler riabilitare per permettere una nuova scelta
                profileSelect.disabled = false;
                selectButton.disabled = false;
                if (deleteButton) { deleteButton.disabled = false; }
            }
        } else {
            showMessage(`No data found for profile '${selectedProfileName}' or an error occurred during loading.`, 'error');
            // In caso di errore, riabilita
            profileSelect.disabled = false;
            selectButton.disabled = false;
            if (deleteButton) { deleteButton.disabled = false; }
        }
    } catch (error) {
        console.error(`Error calling eel.carica_profilo_selezionato for '${selectedProfileName}':`, error);
        showMessage("Communication error with the backend while loading the profile.", 'error');
        // In caso di errore di comunicazione, riabilita
        profileSelect.disabled = false;
        selectButton.disabled = false;
        if (deleteButton) { deleteButton.disabled = false; }
    }
}



async function deleteProfile() {
    const profileSelect = document.getElementById('profileSelect');
    const selectedProfileName = profileSelect.value;

    if (!selectedProfileName || selectedProfileName === "-- Select a profile --" || selectedProfileName === "No profile available" || selectedProfileName === "Loading error") {
        showMessage("Please select a profile to delete.", 'error');
        return;
    }

    const confirmDelete = confirm(`Are you sure you want to delete profile '${selectedProfileName}'?`);
    if (!confirmDelete) {
        showMessage("Deletion cancelled.", 'info');
        return;
    }

    showMessage(`Deleting profile '${selectedProfileName}'...`, 'info');
    try {
        const success = await eel.elimina_profilo(selectedProfileName)();
        if (success) {
            showMessage(`Profile '${selectedProfileName}' deleted successfully!`, 'success');
            // Dopo aver eliminato, ripopola il dropdown
            populateProfileDropdown();
        } else {
            showMessage(`Error deleting profile '${selectedProfileName}'.`, 'error');
        }
    } catch (error) {
        console.error("Errore durante la chiamata a eel.elimina_profilo:", error);
        showMessage("Errore di comunicazione con il backend.", 'error');
    }
}