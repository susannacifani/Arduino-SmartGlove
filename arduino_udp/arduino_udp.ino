#include <Wire.h>
#include <MPU6050.h>
#include <WiFiS3.h>   // For Arduino Uno R4 WiFi
#include <WiFiUdp.h>  // For UDP communication

MPU6050 mpu;
int flexPin = A0;
const int vibrationMotorPin = 4;

// Replace with your network credentials
char ssid[] = "iPhone di Susanna";    // Your network SSID (name)
char pass[] = "bbno7788"; // Your network password (if any)

int status = WL_IDLE_STATUS;

WiFiUDP Udp;

// IP address of the computer running the Python script
// IMPORTANT: Replace with your PC's actual IP address
const IPAddress pythonServerIP(172, 20, 10, 2);
const unsigned int pythonServerPort = 8888; // Port Python is listening on for sensor data

// Port Arduino is listening on for commands from the Python script
const unsigned int arduinoListenPort = 8889;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  pinMode(vibrationMotorPin, OUTPUT);

  // MPU6050 connection check
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connesso correttamente!");
  } else {
    Serial.println("Errore nel collegamento al MPU6050!");
    // Consider adding a delay or halt here if MPU6050 is critical
  }

  // Attempt to connect to WiFi network
  Serial.println("Tentativo di connessione al WiFi...");
  while (status != WL_CONNECTED) {
    Serial.print("Connessione a SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(5000); // Wait 10 seconds for connection
  }

  Serial.println("Connesso al WiFi!");
  printWifiStatus(); // Print WiFi status and IP address

  Udp.begin(arduinoListenPort); // Start UDP listener for commands from Python
  Serial.print("In ascolto su UDP porta: ");
  Serial.println(arduinoListenPort);
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  int flexValue = analogRead(flexPin);

  // Send the data as a CSV string via UDP
  String dataString = String(ax) + "," + String(ay) + "," + String(az) + "," +
                      String(gx) + "," + String(gy) + "," + String(gz) + "," +
                      String(flexValue);

  Udp.beginPacket(pythonServerIP, pythonServerPort);
  Udp.print(dataString);
  Udp.endPacket();

  // Listen for incoming UDP packets from Python (for vibration commands)
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255]; // buffer to hold incoming packet
    Udp.read(packetBuffer, packetSize);
    packetBuffer[packetSize] = 0; // Null-terminate the string

    Serial.print("Comando ricevuto: ");
    Serial.println(packetBuffer);

    // Process the command for vibration
    if (packetBuffer[0] == 'S') { // Swipe command
      digitalWrite(vibrationMotorPin, HIGH);
      delay(100); // Short vibration for swipe
      digitalWrite(vibrationMotorPin, LOW);
    } else if (packetBuffer[0] == 'P') { // Play/Pause command
      digitalWrite(vibrationMotorPin, HIGH);
      delay(100); // Shorter vibration for play/pause
      digitalWrite(vibrationMotorPin, LOW);
    } else if (packetBuffer[0] == 'U') { // Scroll command
      digitalWrite(vibrationMotorPin, HIGH);
      delay(100); // Very short vibration for scroll
      digitalWrite(vibrationMotorPin, LOW);
    }
  }

}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("Indirizzo IP dell'Arduino: ");
  Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print("Potenza segnale (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}