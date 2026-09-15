#include <Arduino.h>
#include <WiFi.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <XPowersLib.h>

#include "WeatherSensorCfg.h"
#include "WeatherSensor.h"
#include "WeatherUtils.h"
#include "secrets.h"

// Hardware and network objects
XPowersAXP2101 PMU_2101;
XPowersAXP192 PMU_192;
WeatherSensor weatherSensor;
WiFiClient net;
MQTTClient mqttClient(1024);

const char* MQTT_BASE_TOPIC = "homeassistant/sensor/bresser_weatherstation_9in1";

unsigned long lastDiscoveryTime = 0;
const unsigned long DISCOVERY_INTERVAL = 300000; // Refresh discovery every 5 minutes

// --- 1. AXP2101 / AXP192 Power Management: Enable power for SX1262 LoRa ---
bool initPMU() {
    Wire.begin(21, 22);
    
    // Try AXP2101 (T-Beam V1.2)
    if (PMU_2101.begin(Wire, AXP2101_SLAVE_ADDRESS, 21, 22)) {
        log_i("AXP2101 PMU detected.");
        PMU_2101.setALDO2Voltage(3300);
        PMU_2101.enableALDO2();
        log_i("ALDO2 (SX1262 LoRa 3.3V) enabled.");
        return true;
    }
    
    // Fallback: AXP192 (T-Beam V1.0 / V1.1)
    if (PMU_192.begin(Wire, AXP192_SLAVE_ADDRESS, 21, 22)) {
        log_i("AXP192 PMU detected.");
        PMU_192.setLDO2Voltage(3300);
        PMU_192.enableLDO2();
        log_i("LDO2 (LoRa 3.3V) enabled.");
        return true;
    }

    log_e("No AXP PMU detected!");
    return false;
}

// --- 2. Home Assistant MQTT Auto-Discovery ---
void publishDiscoverySensor(const char* id, const char* name, const char* dev_cla, 
                            const char* unit, const char* val_field, const char* state_class = nullptr) {
    JsonDocument doc;
    char unique_id[64];
    snprintf(unique_id, sizeof(unique_id), "bresser_9in1_%s", id);

    char state_topic[128];
    snprintf(state_topic, sizeof(state_topic), "%s/state", MQTT_BASE_TOPIC);

    doc["name"] = name;
    doc["unique_id"] = unique_id;
    doc["state_topic"] = state_topic;
    
    char val_tmpl[64];
    snprintf(val_tmpl, sizeof(val_tmpl), "{{ value_json.%s }}", val_field);
    doc["value_template"] = val_tmpl;

    if (dev_cla) doc["device_class"] = dev_cla;
    if (unit) doc["unit_of_measurement"] = unit;
    if (state_class) doc["state_class"] = state_class;

    // Device block: Groups all sensors under one device in Home Assistant
    JsonObject dev = doc["device"].to<JsonObject>();
    JsonArray id_arr = dev["identifiers"].to<JsonArray>();
    id_arr.add("bresser_9in1_7803520");
    dev["name"] = "Bresser Weather Station";
    dev["model"] = "7803520 9-in-1 Solar";
    dev["manufacturer"] = "Bresser";

    char discTopic[128];
    snprintf(discTopic, sizeof(discTopic), "%s/%s/config", MQTT_BASE_TOPIC, id);

    String payload;
    serializeJson(doc, payload);
    bool ok = mqttClient.publish(discTopic, payload, true, 0); // Retained
    if (ok) {
        Serial.printf("[MQTT] Discovery OK: %s\n", discTopic);
    } else {
        Serial.printf("[MQTT] Discovery FAILED: %s (Error: %d)\n", discTopic, mqttClient.lastError());
    }
    delay(15);
    mqttClient.loop();
}

void sendHomeAssistantDiscovery() {
    log_i("Publishing Home Assistant MQTT discovery configurations...");
    publishDiscoverySensor("temperature", "Temperature", "temperature", "°C", "temperature", "measurement");
    publishDiscoverySensor("humidity", "Humidity", "humidity", "%", "humidity", "measurement");
    publishDiscoverySensor("wind_speed", "Wind Speed", "wind_speed", "m/s", "wind_speed", "measurement");
    publishDiscoverySensor("wind_gust", "Wind Gust", "wind_speed", "m/s", "wind_gust", "measurement");
    publishDiscoverySensor("wind_direction", "Wind Direction", "wind_direction", "°", "wind_direction");
    publishDiscoverySensor("rain", "Total Rain", "precipitation", "mm", "rain", "total_increasing");
    publishDiscoverySensor("light_lux", "Illuminance", "illuminance", "lx", "light_lux", "measurement");
    publishDiscoverySensor("uv", "UV Index", nullptr, nullptr, "uv", "measurement");
    publishDiscoverySensor("dewpoint", "Dew Point", "temperature", "°C", "dewpoint", "measurement");
    publishDiscoverySensor("battery", "Battery", "battery", "%", "battery");
    publishDiscoverySensor("rssi", "Signal Strength", "signal_strength", "dBm", "rssi", "measurement");
}

// --- 3. WiFi and MQTT Connection Handling ---
void connectWiFiAndMQTT() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Connecting to WiFi...");
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        int retries = 0;
        while (WiFi.status() != WL_CONNECTED && retries < 20) {
            delay(500);
            Serial.print(".");
            retries++;
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
        } else {
            Serial.println("\nWiFi connection failed, will retry later.");
            return;
        }
    }

    if (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT broker (" + String(MQTT_HOST) + ")...");
        if (mqttClient.connect("T-Beam-Bresser-Gateway", MQTT_USER, MQTT_PASS)) {
            Serial.println("\nMQTT connected!");
            sendHomeAssistantDiscovery();
            lastDiscoveryTime = millis();
        } else {
            Serial.println("\nMQTT connection failed.");
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== Bresser 9-in-1 Weatherstation Receiver Started ===");

    // 1. Initialize PMU (Enable power supply rail for SX1262 LoRa module)
    if (!initPMU()) {
        Serial.println("Warning: PMU initialization failed!");
    }

    // 2. Initialize RadioLib / SX1262 transceiver
    // Uses T-Beam pins defined in WeatherSensorCfg.h: CS 18, IRQ 33, GPIO 32, RST 23
    int state = weatherSensor.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Failed to initialize SX1262 transceiver! Error code: %d\n", state);
        while (true) { delay(1000); }
    }
    Serial.println("SX1262 successfully initialized on 868.3 MHz.");

    // 3. Prepare MQTT client
    mqttClient.begin(MQTT_HOST, MQTT_PORT, net);
    connectWiFiAndMQTT();
}

void loop() {
    // Check WiFi and MQTT connections
    if (WiFi.status() != WL_CONNECTED || !mqttClient.connected()) {
        static unsigned long lastReconnectAttempt = 0;
        if (millis() - lastReconnectAttempt > 5000) {
            lastReconnectAttempt = millis();
            connectWiFiAndMQTT();
        }
    }
    mqttClient.loop();

    // Periodically refresh discovery
    if (millis() - lastDiscoveryTime > DISCOVERY_INTERVAL) {
        if (mqttClient.connected()) {
            sendHomeAssistantDiscovery();
        }
        lastDiscoveryTime = millis();
    }

    // 4. Receive and decode weather sensor radio packets (100 ms timeout for reactive loop)
    int decode_status = weatherSensor.getData(100, DATA_COMPLETE);

    if (decode_status == DECODE_OK) {
        for (size_t i = 0; i < weatherSensor.sensor.size(); i++) {
            if (!weatherSensor.sensor[i].valid) continue;

            auto &s = weatherSensor.sensor[i];
            Serial.printf("Received sensor [0x%08X]. RSSI: %.1f dBm\n", s.sensor_id, s.rssi);

            JsonDocument stateDoc;
            stateDoc["sensor_id"] = s.sensor_id;
            stateDoc["rssi"] = s.rssi;
            stateDoc["battery"] = s.battery_ok ? 100 : 10;

            if (s.w.temp_ok) stateDoc["temperature"] = s.w.temp_c;
            if (s.w.humidity_ok) stateDoc["humidity"] = s.w.humidity;
            if (s.w.wind_ok) {
                stateDoc["wind_speed"] = s.w.wind_avg_meter_sec;
                stateDoc["wind_gust"] = s.w.wind_gust_meter_sec;
                stateDoc["wind_direction"] = s.w.wind_direction_deg;
            }
            if (s.w.rain_ok) stateDoc["rain"] = s.w.rain_mm;
            if (s.w.light_ok) stateDoc["light_lux"] = s.w.light_lux;
            if (s.w.uv_ok) stateDoc["uv"] = s.w.uv;

            // Calculated values (Dew Point)
            if (s.w.temp_ok && s.w.humidity_ok) {
                stateDoc["dewpoint"] = calcdewpoint(s.w.temp_c, s.w.humidity);
            }

            if (mqttClient.connected()) {
                String jsonPayload;
                serializeJson(stateDoc, jsonPayload);
                char stateTopic[128];
                snprintf(stateTopic, sizeof(stateTopic), "%s/state", MQTT_BASE_TOPIC);
                bool ok = mqttClient.publish(stateTopic, jsonPayload, false, 0);
                if (ok) {
                    Serial.printf("[MQTT] Published to %s: %s\n", stateTopic, jsonPayload.c_str());
                } else {
                    Serial.printf("[MQTT] Publish FAILED to %s (Error: %d)\n", stateTopic, mqttClient.lastError());
                }
            } else {
                Serial.println("[MQTT] Not connected! Cannot publish state.");
            }
        }
    }
}
