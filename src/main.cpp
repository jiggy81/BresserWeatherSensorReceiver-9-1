#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
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
const char* MQTT_RECEIVER_BASE_TOPIC = "homeassistant/sensor/bresser_receiver";

unsigned long lastDiscoveryTime = 0;
const unsigned long DISCOVERY_INTERVAL = 300000; // Refresh discovery every 5 minutes

enum PmuType { PMU_TYPE_NONE, PMU_TYPE_AXP192, PMU_TYPE_AXP2101 };
PmuType detectedPMU = PMU_TYPE_NONE;

// --- 1. AXP2101 / AXP192 Power Management: Enable power & charging ---
bool initPMU() {
    Wire.begin(21, 22);
    
    // Try AXP2101 (T-Beam V1.2)
    if (PMU_2101.begin(Wire, AXP2101_SLAVE_ADDRESS, 21, 22)) {
        log_i("AXP2101 PMU detected.");
        PMU_2101.setALDO2Voltage(3300);
        PMU_2101.enableALDO2();
        log_i("ALDO2 enabled.");

        // SX1262 LoRa transceiver power rail on LilyGO T-Beam V1.2 is ALDO3
        PMU_2101.setALDO3Voltage(3300);
        PMU_2101.enableALDO3();
        log_i("ALDO3 (SX1262 LoRa 3.3V) enabled.");

        // 1. Akku-Ladefunktion aktivieren (18650 Li-Ion: 4.2V, 500mA Ladestrom)
        PMU_2101.enableCellbatteryCharge();
        PMU_2101.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
        PMU_2101.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);

        // 2. Hardware-Lade-LED am Board aktivieren (leuchtet beim Laden, erlischt wenn voll)
        PMU_2101.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

        detectedPMU = PMU_TYPE_AXP2101;
        return true;
    }
    
    // Fallback: AXP192 (T-Beam V1.0 / V1.1)
    if (PMU_192.begin(Wire, AXP192_SLAVE_ADDRESS, 21, 22)) {
        log_i("AXP192 PMU detected.");
        PMU_192.setLDO2Voltage(3300);
        PMU_192.enableLDO2();
        log_i("LDO2 (LoRa 3.3V) enabled.");

        PMU_192.enableCharge();
        PMU_192.setChargeTargetVoltage(XPOWERS_AXP192_CHG_VOL_4V2);
        PMU_192.setChargerConstantCurr(XPOWERS_AXP192_CHG_CUR_780MA);
        PMU_192.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

        detectedPMU = PMU_TYPE_AXP192;
        return true;
    }

    log_e("No AXP PMU detected!");
    return false;
}

int getReceiverBatteryPercent() {
    if (detectedPMU == PMU_TYPE_AXP2101) {
        return PMU_2101.getBatteryPercent();
    } else if (detectedPMU == PMU_TYPE_AXP192) {
        return PMU_192.getBatteryPercent();
    } else {
        uint32_t mv = analogReadMilliVolts(35);
        float vbat = (mv * 2.0f) / 1000.0f;
        if (vbat <= 3.3f) return 0;
        if (vbat >= 4.2f) return 100;
        return (int)((vbat - 3.3f) / (4.2f - 3.3f) * 100.0f);
    }
}

float getReceiverBatteryVoltage() {
    if (detectedPMU == PMU_TYPE_AXP2101) {
        return PMU_2101.getBattVoltage() / 1000.0f;
    } else if (detectedPMU == PMU_TYPE_AXP192) {
        return PMU_192.getBattVoltage() / 1000.0f;
    } else {
        uint32_t mv = analogReadMilliVolts(35);
        return (mv * 2.0f) / 1000.0f;
    }
}

bool isReceiverCharging() {
    if (detectedPMU == PMU_TYPE_AXP2101) {
        return PMU_2101.isCharging();
    } else if (detectedPMU == PMU_TYPE_AXP192) {
        return PMU_192.isCharging();
    }
    return false;
}

bool isReceiverUsbPower() {
    if (detectedPMU == PMU_TYPE_AXP2101) {
        return PMU_2101.isVbusIn();
    } else if (detectedPMU == PMU_TYPE_AXP192) {
        return PMU_192.isVbusIn();
    }
    return false;
}

const char* getReceiverChargerStatus() {
    if (detectedPMU == PMU_TYPE_AXP2101) {
        if (!PMU_2101.isBatteryConnect()) return "No Battery";
        xpowers_chg_status_t st = PMU_2101.getChargerStatus();
        switch (st) {
            case XPOWERS_AXP2101_CHG_TRI_STATE: return "Tri-Charge";
            case XPOWERS_AXP2101_CHG_PRE_STATE: return "Pre-Charge";
            case XPOWERS_AXP2101_CHG_CC_STATE:  return "Constant Current";
            case XPOWERS_AXP2101_CHG_CV_STATE:  return "Constant Voltage";
            case XPOWERS_AXP2101_CHG_DONE_STATE: return "Fully Charged";
            case XPOWERS_AXP2101_CHG_STOP_STATE:
            default:
                return PMU_2101.isVbusIn() ? "Standby" : "Discharging";
        }
    }
    return "Unknown";
}

// --- 2. Home Assistant MQTT Auto-Discovery ---
void publishWeatherDiscoverySensor(const char* id, const char* name, const char* dev_cla, 
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

    JsonObject dev = doc["device"].to<JsonObject>();
    JsonArray id_arr = dev["identifiers"].to<JsonArray>();
    id_arr.add("bresser_9in1_7803520");
    dev["name"] = "Wetterstation";
    dev["model"] = "7803520 9-in-1 Solar";
    dev["manufacturer"] = "Bresser";

    char discTopic[128];
    snprintf(discTopic, sizeof(discTopic), "%s/%s/config", MQTT_BASE_TOPIC, id);

    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(discTopic, payload, true, 0); // Retained
    delay(15);
    mqttClient.loop();
}

void publishReceiverDiscoverySensor(const char* id, const char* name, const char* dev_cla,
                                    const char* unit, const char* val_field, const char* state_class = nullptr,
                                    const char* icon = nullptr) {
    JsonDocument doc;
    char unique_id[64];
    snprintf(unique_id, sizeof(unique_id), "bresser_receiver_%s", id);

    char state_topic[128];
    snprintf(state_topic, sizeof(state_topic), "%s/state", MQTT_RECEIVER_BASE_TOPIC);

    doc["name"] = name;
    doc["unique_id"] = unique_id;
    doc["state_topic"] = state_topic;
    
    char val_tmpl[64];
    snprintf(val_tmpl, sizeof(val_tmpl), "{{ value_json.%s }}", val_field);
    doc["value_template"] = val_tmpl;

    if (dev_cla) doc["device_class"] = dev_cla;
    if (unit) doc["unit_of_measurement"] = unit;
    if (state_class) doc["state_class"] = state_class;
    if (icon) doc["icon"] = icon;

    JsonObject dev = doc["device"].to<JsonObject>();
    JsonArray id_arr = dev["identifiers"].to<JsonArray>();
    id_arr.add("bresser_receiver_ttgo");
    dev["name"] = "Wetterstation Receiver";
    dev["model"] = "LilyGO T-Beam Receiver";
    dev["manufacturer"] = "LilyGO";

    char discTopic[128];
    snprintf(discTopic, sizeof(discTopic), "homeassistant/sensor/bresser_receiver/%s/config", id);

    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(discTopic, payload, true, 0); // Retained
    delay(15);
    mqttClient.loop();
}

void publishReceiverDiscoveryBinarySensor(const char* id, const char* name, const char* dev_cla,
                                          const char* val_field) {
    JsonDocument doc;
    char unique_id[64];
    snprintf(unique_id, sizeof(unique_id), "bresser_receiver_%s", id);

    char state_topic[128];
    snprintf(state_topic, sizeof(state_topic), "%s/state", MQTT_RECEIVER_BASE_TOPIC);

    doc["name"] = name;
    doc["unique_id"] = unique_id;
    doc["state_topic"] = state_topic;
    
    char val_tmpl[64];
    snprintf(val_tmpl, sizeof(val_tmpl), "{{ 'ON' if value_json.%s else 'OFF' }}", val_field);
    doc["value_template"] = val_tmpl;

    if (dev_cla) doc["device_class"] = dev_cla;

    JsonObject dev = doc["device"].to<JsonObject>();
    JsonArray id_arr = dev["identifiers"].to<JsonArray>();
    id_arr.add("bresser_receiver_ttgo");
    dev["name"] = "Wetterstation Receiver";
    dev["model"] = "LilyGO T-Beam Receiver";
    dev["manufacturer"] = "LilyGO";

    char discTopic[128];
    snprintf(discTopic, sizeof(discTopic), "homeassistant/binary_sensor/bresser_receiver/%s/config", id);

    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(discTopic, payload, true, 0); // Retained
    delay(15);
    mqttClient.loop();
}

void sendHomeAssistantDiscovery() {
    log_i("Publishing Home Assistant MQTT discovery configurations...");

    // 1. Weather Station Sensors
    publishWeatherDiscoverySensor("temperature", "Temperature", "temperature", "°C", "temperature", "measurement");
    publishWeatherDiscoverySensor("humidity", "Humidity", "humidity", "%", "humidity", "measurement");
    publishWeatherDiscoverySensor("wind_speed", "Wind Speed", "wind_speed", "m/s", "wind_speed", "measurement");
    publishWeatherDiscoverySensor("wind_gust", "Wind Gust", "wind_speed", "m/s", "wind_gust", "measurement");
    publishWeatherDiscoverySensor("wind_direction", "Wind Direction", "wind_direction", "°", "wind_direction");
    publishWeatherDiscoverySensor("rain", "Total Rain", "precipitation", "mm", "rain", "total_increasing");
    publishWeatherDiscoverySensor("light_lux", "Illuminance", "illuminance", "lx", "light_lux", "measurement");
    publishWeatherDiscoverySensor("uv", "UV Index", nullptr, nullptr, "uv", "measurement");
    publishWeatherDiscoverySensor("dewpoint", "Dew Point", "temperature", "°C", "dewpoint", "measurement");
    publishWeatherDiscoverySensor("battery", "Battery", "battery", "%", "battery");
    publishWeatherDiscoverySensor("rssi", "Signal Strength", "signal_strength", "dBm", "rssi", "measurement");

    // Clean up old orphaned battery_receiver config under weather station topic if any
    mqttClient.publish("homeassistant/sensor/bresser_weatherstation_9in1/battery_receiver/config", "", true, 0);

    // 2. Wetterstation Receiver Device Sensors & Binary Sensors
    publishReceiverDiscoverySensor("battery", "Battery", "battery", "%", "battery", "measurement");
    publishReceiverDiscoveryBinarySensor("charging", "Charging", "battery_charging", "charging");
    publishReceiverDiscoveryBinarySensor("usb_power", "USB Power", "power", "usb_power");
    publishReceiverDiscoverySensor("voltage", "Battery Voltage", "voltage", "V", "voltage", "measurement");
    publishReceiverDiscoverySensor("charger_status", "Charger Status", nullptr, nullptr, "charger_status", nullptr, "mdi:battery-sync");
    publishReceiverDiscoverySensor("wifi_signal", "WiFi Signal", "signal_strength", "dBm", "wifi_rssi", "measurement");
}

// --- 3. Telemetry Publishing ---
void publishReceiverTelemetry() {
    if (!mqttClient.connected()) return;

    JsonDocument doc;
    int bat = getReceiverBatteryPercent();
    if (bat >= 0) {
        doc["battery"] = bat;
    } else {
        doc["battery"] = nullptr;
    }

    doc["voltage"] = serialized(String(getReceiverBatteryVoltage(), 2));
    doc["charging"] = isReceiverCharging();
    doc["usb_power"] = isReceiverUsbPower();
    doc["charger_status"] = getReceiverChargerStatus();
    doc["wifi_rssi"] = WiFi.RSSI();

    String payload;
    serializeJson(doc, payload);
    char topic[128];
    snprintf(topic, sizeof(topic), "%s/state", MQTT_RECEIVER_BASE_TOPIC);
    bool ok = mqttClient.publish(topic, payload, false, 0);
    if (ok) {
        Serial.printf("[MQTT] Receiver Telemetry: %s\n", payload.c_str());
    } else {
        Serial.printf("[MQTT] Receiver Telemetry FAILED (Error: %d)\n", mqttClient.lastError());
    }
}

// --- 4. WiFi and MQTT Connection Handling ---
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
            publishReceiverTelemetry();
        } else {
            Serial.println("\nMQTT connection failed.");
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== Bresser 9-in-1 Weatherstation Receiver Started ===");

    // 1. Initialize PMU (Enable SX1262 LoRa power, battery charging & charge LED)
    if (!initPMU()) {
        Serial.println("Warning: PMU initialization failed!");
    } else {
        Serial.printf("[PMU] VBUS connected: %s\n", isReceiverUsbPower() ? "YES" : "NO");
        Serial.printf("[PMU] Battery Voltage: %.2f V\n", getReceiverBatteryVoltage());
        Serial.printf("[PMU] Battery Percent: %d %%\n", getReceiverBatteryPercent());
        Serial.printf("[PMU] Charging: %s (%s)\n", isReceiverCharging() ? "YES" : "NO", getReceiverChargerStatus());
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

    // 4. ArduinoOTA for wireless updates
    ArduinoOTA.setHostname("bresser-receiver");
    ArduinoOTA.begin();
    Serial.println("ArduinoOTA ready for wireless updates.");
}

void loop() {
    ArduinoOTA.handle();
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

    // Periodically publish receiver telemetry (every 30 seconds or on USB power change)
    static unsigned long lastReceiverTelemetry = 0;
    static bool lastUsbState = false;
    bool currentUsb = isReceiverUsbPower();
    if (millis() - lastReceiverTelemetry > 30000 || currentUsb != lastUsbState) {
        if (mqttClient.connected()) {
            publishReceiverTelemetry();
        }
        lastReceiverTelemetry = millis();
        lastUsbState = currentUsb;
    }

    // 4. Receive and decode weather sensor radio packets (100 ms timeout for reactive loop)
    int decode_status = weatherSensor.getData(1000, DATA_COMPLETE);

    if (decode_status == DECODE_OK) {
        for (size_t i = 0; i < weatherSensor.sensor.size(); i++) {
            if (!weatherSensor.sensor[i].valid) continue;

            auto &s = weatherSensor.sensor[i];
            Serial.printf("Received sensor [0x%08X]. RSSI: %.1f dBm\n", s.sensor_id, s.rssi);

            JsonDocument stateDoc;
            stateDoc["sensor_id"] = s.sensor_id;
            stateDoc["rssi"] = s.rssi;
            stateDoc["battery"] = s.battery_ok ? 100 : 10;

            int rxBat = getReceiverBatteryPercent();
            if (rxBat >= 0) {
                stateDoc["battery_receiver"] = rxBat;
            } else {
                stateDoc["battery_receiver"] = nullptr;
            }

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
                bool ok = mqttClient.publish(stateTopic, jsonPayload, true, 0);
                if (ok) {
                    Serial.printf("[MQTT] Published to %s: %s\n", stateTopic, jsonPayload.c_str());
                } else {
                    Serial.printf("[MQTT] Publish FAILED to %s (Error: %d)\n", stateTopic, mqttClient.lastError());
                }
            } else {
                Serial.println("[MQTT] Not connected! Cannot publish state.");
            }
        }
        weatherSensor.clearSlots();
    }
}
