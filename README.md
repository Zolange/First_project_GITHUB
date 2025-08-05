#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ==== Wi-Fi Credentials ====
const char* ssid = "KALANSO_Telefilani";
const char* password = "M@l!BeDjo09";

// ==== MQTT Broker Info ====
const char* mqtt_broker = "a2249c968f424394925bcdd597fff0c7.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "Sylla";
const char* mqtt_password = "Alert_o001";
const char* topic_publish_ir = "esp32/ir_sensor";

// ==== SIM900 (via UART1) ====
HardwareSerial sim900(1);  // UART1

// ==== Constantes / Broches ====
const int anemometerPin = 5;
const int ONE_WIRE_BUS = 4;
const int triggerPin = 26;
const int echoPin = 12;

// ==== Capteurs ====
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ==== WiFi / MQTT ====
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// ==== Variables globales ====
volatile int windCount = 0;
unsigned long lastWindMillis = 0;
unsigned long previousTime = 0;
long windSpeed = 0;
const float windFactor = 2.4;

// ==== Interruption Anémomètre ====
void countWind() {
  windCount++;
}

// ==== Connexion MQTT ====
void setupMQTT() {
  mqttClient.setServer(mqtt_broker, mqtt_port);
}

void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
    Serial.println("Reconnecting to MQTT Broker...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT Broker.");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" -> retry in 5 sec");
      delay(5000);
    }
  }
}

// ==== Setup ====
void setup() {
  Serial.begin(115200);
  sim900.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.println("SIM900 initialisé.");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connecté.");

  wifiClient.setInsecure(); // ⚠️ Insecure mode (pas pour production)

  sensors.begin();
  Serial.println("DS18B20 prêt.");

  setupMQTT();

  pinMode(anemometerPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(anemometerPin), countWind, FALLING);


  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

// ==== Loop ====
void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  unsigned long now = millis();
  if (now - previousTime > 10000) {  // Toutes les 10 secondes
    previousTime = now;

    

    // === Capteur ultrason ===
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    long distance = (duration * 0.0343) / 2;
    distance = distance/100;
    Serial.print("Niveau d'eau : ");
    Serial.println(distance);

    // === Vitesse du vent ===
    if (now - lastWindMillis >= 10000) {
      lastWindMillis = now;
      windSpeed = (windCount * windFactor) / 1000.0 * 3600.0;
      windCount = 0;
      Serial.print("Vitesse du vent : ");
      Serial.print(windSpeed);
      Serial.println(" km/h");
    }


    // === Température ===
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    Serial.print("Température : ");
    Serial.print(tempC);
    Serial.println(" °C");

    // === JSON à envoyer ===
    StaticJsonDocument<300> doc;
    doc["device_id"] = "ESP32_2";
    doc["timestamp"] = now / 1000.0;
    doc["Temperature"] = tempC;
    doc["niveau_eau"] = distance;
    doc["vitesse_du_vent"] = windSpeed;


    char jsonBuffer[300];
    serializeJson(doc, jsonBuffer);

    // === Topic avec timestamp ===
    unsigned long ts = now / 1000;
    char topic_with_timestamp[50];
    snprintf(topic_with_timestamp, sizeof(topic_with_timestamp), "%s/%lu", topic_publish_ir, ts);

    if (mqttClient.connected()) {
      mqttClient.publish(topic_with_timestamp, jsonBuffer);
      Serial.print("Données envoyées sur ");
      Serial.print(topic_with_timestamp);
      Serial.print(" : ");
      Serial.println(jsonBuffer);
    }

    delay(100);
  }
}
