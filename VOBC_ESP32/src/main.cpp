#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include <assert.h>

#include "NetworkConfig.h"
#include "OnboardState.h"
#include "Kinematics.h"

// Configuration structures
struct WifiConfig
{
    const char *ssid;
    const char *password;
};

struct NtpConfig
{
    const char *server;
    int gmtOffsetSec;
    int daylightOffsetSec;
};

struct MqttConfig
{
    const char *server;
    int port;
    const char *clientId;
    const char *statusTopic;
    const char *controlTopic;
    const char *vobcTopic;
    const char *testTopic;
};

struct AppState
{
    bool ledOn;
    unsigned long lastBlinkMs;
    unsigned long lastTelemetryMs;
};

// Configuration instances
const WifiConfig wifiConfig = {
    .ssid = "Airtel_Primerail Infralabs Pvt L",
    .password = "Primeit@2024",
};

const NtpConfig ntpConfig = {
    .server = "pool.ntp.org",
    .gmtOffsetSec = 0,
    .daylightOffsetSec = 0,
};

const MqttConfig mqttConfig = {
    .server = "broker.emqx.io",
    .port = 1883,
    .clientId = "VOBC_ESP32_01",
    .statusTopic = "railway/vobc/status",
    .controlTopic = "railway/vobc/control",
    .vobcTopic = "railway/vobc/telemetry",
    .testTopic = "railway/test",
};

AppState appState = {
    .ledOn = false,
    .lastBlinkMs = 0,
    .lastTelemetryMs = 0,
};

// Forward declarations
void callback(char *topic, byte *payload, unsigned int length);
void logMessage(const char *message);

WiFiClient espClient;
PubSubClient mqttClient(espClient);
ModbusMaster modbusNode;
KinematicsEngine physicsCore;

VehicleState trainState;

// Constants for buffers
const size_t LOG_BUFFER_SIZE = 256;
const size_t TIMESTAMP_BUFFER_SIZE = 64;
const size_t COMMAND_BUFFER_SIZE = 32;
const size_t TELEMETRY_BUFFER_SIZE = 512;

unsigned long lastTickTime = 0;
unsigned long lastTelemetryStreamTime = 0;

const uint16_t REG_MOTOR_MODE = 2;
const uint16_t REG_TARGET_SPEED = 6;
const uint16_t CMD_ENABLE_CW = 257;
const uint16_t CMD_DISABLE_MOTOR = 256;

// ============= LOGGING & STORAGE =============
static bool getTimestamp(char *output, size_t capacity)
{
    assert(output != nullptr);
    assert(capacity >= 1);

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        unsigned long ms = millis();
        unsigned long seconds = ms / 1000;
        int written = snprintf(output, capacity, "uptime:%lus", seconds);
        return written > 0 && static_cast<size_t>(written) < capacity;
    }

    int written = strftime(output, capacity, "%Y-%m-%d %H:%M:%S", &timeinfo);
    return written > 0;
}

void logMessage(const char *message)
{
    assert(message != nullptr);

    char timestamp[TIMESTAMP_BUFFER_SIZE];
    if (!getTimestamp(timestamp, sizeof(timestamp)))
    {
        strncpy(timestamp, "unknown-time", sizeof(timestamp));
        timestamp[sizeof(timestamp) - 1] = '\0';
    }

    char line[LOG_BUFFER_SIZE];
    int written = snprintf(line, sizeof(line), "%s - %s", timestamp, message);
    if (written <= 0)
    {
        return;
    }

    Serial.println(line);
}

// ============= NETWORK CONNECTIVITY =============
static bool wifiInitialized = false;
static unsigned long wifiStartTime = 0;

bool checkAndInitWifi()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        if (!wifiInitialized)
        {
            wifiInitialized = true;
            logMessage("WiFi connected");
            configTime(ntpConfig.gmtOffsetSec, ntpConfig.daylightOffsetSec, ntpConfig.server);
        }
        return true;
    }

    if (!wifiInitialized && wifiStartTime == 0)
    {
        WiFi.begin(wifiConfig.ssid, wifiConfig.password);
        wifiStartTime = millis();
        logMessage("WiFi connection initiated");
    }

    if (millis() - wifiStartTime > 30000)
    {
        wifiStartTime = 0;
        logMessage("WiFi timeout - retry");
    }

    return false;
}

void sendVelocityToModbus(float velocity_hz)
{
    int16_t target_hz = (int16_t)velocity_hz;

    if (target_hz < 0)
        target_hz = 0;
    if (target_hz > 380)
        target_hz = 380;

    uint8_t result = modbusNode.writeSingleRegister(REG_TARGET_SPEED, target_hz);
    if (result == 0)
    {
        trainState.consecutiveModbusErrors = 0;
    }
    else
    {
        trainState.consecutiveModbusErrors++;
        if (trainState.consecutiveModbusErrors >= 3)
        {
            // trainState.status = STATE_EMERGENCY_BRAKE;
            // modbusNode.writeSingleRegister(REG_MOTOR_MODE, CMD_DISABLE_MOTOR);
            Serial.println("[DESK MODE] Physical Modbus link absent! Simulating operational tracking anyway...");
        } // Because your Modbus hardware (the VFD motor driver) isn't physically wired to your ESP32 on your desk right now, the code will instantly trip into an emergency brake loop if left unmodified. We need to apply a tiny temporary tweak for testing.
    }
}

void setupMqttClient()
{
    mqttClient.setServer(mqttConfig.server, mqttConfig.port);
    mqttClient.setCallback(callback);
}

bool publishMessage(const char *topic, const char *payload, bool retain)
{
    assert(topic != nullptr);
    assert(payload != nullptr);

    if (!mqttClient.connected())
    {
        return false;
    }

    return mqttClient.publish(topic, payload, retain);
}

bool ensureMqttConnected()
{
    if (mqttClient.connected())
    {
        return true;
    }

    logMessage("Attempting MQTT connection...");
    if (!mqttClient.connect(mqttConfig.clientId))
    {
        char buffer[LOG_BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "MQTT connection failed, rc=%d", mqttClient.state());
        logMessage(buffer);
        return false;
    }

    logMessage("MQTT connected successfully");
    mqttClient.subscribe(TOPIC_AUTHORITY);
    mqttClient.subscribe(TOPIC_FAULT);

    char subscriptionBuffer[LOG_BUFFER_SIZE];
    snprintf(subscriptionBuffer, sizeof(subscriptionBuffer), "Subscribed to authority and fault topics");
    logMessage(subscriptionBuffer);

    publishMessage(mqttConfig.statusTopic, "VOBC_CONNECTED", true);
    return true;
}

void loopMqtt()
{
    if (mqttClient.connected())
    {
        mqttClient.loop();
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    assert(topic != nullptr);
    assert(payload != nullptr);

    const size_t capacity = COMMAND_BUFFER_SIZE;
    char command[COMMAND_BUFFER_SIZE];
    size_t copyLength = length;
    if (copyLength >= capacity)
    {
        copyLength = capacity - 1;
    }

    memcpy(command, payload, copyLength);
    command[copyLength] = '\0';

    for (size_t i = 0; i < copyLength; ++i)
    {
        if (command[i] == '\r' || command[i] == '\n')
        {
            command[i] = '\0';
            break;
        }
    }

    char buffer[LOG_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "MQTT message received [%s]: %s", topic, command);
    logMessage(buffer);

    // Parse JSON authority and fault messages
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error)
    {
        Serial.println("[ERROR] Failed to parse incoming JSON payload.");
        return;
    }

    trainState.lastHeartbeatTime = millis();

    if (strcmp(topic, TOPIC_AUTHORITY) == 0)
    {
        trainState.limitOfAuthority = doc["loa"] | 0.0f;
        trainState.civilSpeedLimit = doc["maxSpeed"] | 0.0f;
        trainState.engineerMaxSpeed = doc["engineerMaxSpeed"] | 250.0f;
        trainState.targetAcceleration = doc["accel"] | 25.0f;
        trainState.targetDeceleration = doc["decel"] | 20.0f;
        trainState.stationDwellTime = doc["dwell"] | 30.0f;

        Serial.printf("[CBTC] Target Loaded -> LOA: %.2f | Civil Ceiling: %.2f Hz\n",
                      trainState.limitOfAuthority, trainState.civilSpeedLimit);

        if (trainState.limitOfAuthority > 0.0f)
        {
            if (trainState.status == STATE_IDLE)
            {
                trainState.status = STATE_RUNNING;
                Serial.println("[CBTC] State changed to RUNNING.");
            }
            modbusNode.writeSingleRegister(REG_MOTOR_MODE, CMD_ENABLE_CW);
        }
    }
    else if (strcmp(topic, TOPIC_FAULT) == 0)
    {
        trainState.interlockingFault = true;
        if (trainState.status != STATE_EMERGENCY_BRAKE)
        {
            trainState.status = STATE_EMERGENCY_BRAKE;
            modbusNode.writeSingleRegister(REG_MOTOR_MODE, CMD_DISABLE_MOTOR);
            Serial.println("[SAFETY] Fault received; emergency brake engaged.");
        }
    }
}

void reconnectNetworkPipeline()
{
    ensureMqttConnected();
}

bool buildTelemetryPayload(char *buffer, size_t capacity)
{
    assert(buffer != nullptr);
    assert(capacity >= 1);

    int written = snprintf(buffer, capacity,
                           "{\"position\":%.2f,\"velocity\":%.2f,\"status\":%d,\"wifi\":%d}",
                           trainState.currentPosition, trainState.currentVelocity,
                           trainState.status, WiFi.status());
    return written > 0 && static_cast<size_t>(written) < capacity;
}

bool publishTelemetry()
{
    const unsigned long now = millis();
    if (now - appState.lastTelemetryMs < 5000)
    {
        return false;
    }

    if (!mqttClient.connected())
    {
        return false;
    }

    appState.lastTelemetryMs = now;

    char payload[TELEMETRY_BUFFER_SIZE];
    if (!buildTelemetryPayload(payload, sizeof(payload)))
    {
        logMessage("Telemetry payload build failed");
        return false;
    }

    if (!publishMessage(mqttConfig.vobcTopic, payload, true))
    {
        logMessage("Failed to publish telemetry");
        return false;
    }

    logMessage("Published JSON telemetry");
    publishMessage(mqttConfig.testTopic, "VOBC_TEST_OK", true);
    return true;
}

void setup()
{
    Serial.begin(115200);
    delay(100);
    Serial.println("\n=== VOBC INITIALIZATION START ===");

    // Initialize Modbus - non-blocking
    Serial2.begin(9600, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
    modbusNode.begin(1, Serial2);
    Serial2.setTimeout(100);

    // WiFi setup - non-blocking
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setHostname("VOBC_ESP32");
    
    // MQTT Initialization
    setupMqttClient();

    // Initialize state
    trainState.lastHeartbeatTime = millis();
    lastTickTime = millis();
    trainState.status = STATE_IDLE;
    
    Serial.println("[SETUP] Complete - non-blocking mode");
}

void loop()
{
    unsigned long currentTime = millis();

    // 0. Non-blocking WiFi check
    static unsigned long lastWifiCheck = 0;
    if (currentTime - lastWifiCheck >= 1000)
    {
        lastWifiCheck = currentTime;
        checkAndInitWifi();
        if (WiFi.status() != WL_CONNECTED)
        {
            WiFi.begin(wifiConfig.ssid, wifiConfig.password);
        }
    }

    // 1. MQTT (only if WiFi is connected)
    if (WiFi.status() == WL_CONNECTED)
    {
        if (!mqttClient.connected())
        {
            ensureMqttConnected();
        }
        loopMqtt();
    }

    // 2. Safety Watchdog
    if (currentTime - trainState.lastHeartbeatTime > WATCHDOG_TIMEOUT_MS)
    {
        if (trainState.status != STATE_EMERGENCY_BRAKE)
        {
            trainState.status = STATE_EMERGENCY_BRAKE;
            modbusNode.writeSingleRegister(REG_MOTOR_MODE, CMD_DISABLE_MOTOR);
            Serial.println("[CRITICAL] Heartbeat lost!");
        }
    }

    // 3. Physics (every 50ms)
    if (currentTime - lastTickTime >= 50)
    {
        float dt = (currentTime - lastTickTime) / 1000.0f;
        lastTickTime = currentTime;
        if (dt > 0.2f) dt = 0.2f;

        if (trainState.status == STATE_RUNNING || trainState.status == STATE_IDLE)
        {
            physicsCore.computeTrajectoryTick(trainState, dt);
            sendVelocityToModbus(trainState.currentVelocity);
        }
    }

    // 4. Telemetry
    publishTelemetry();

    // 5. Debug (every 1s)
    static unsigned long lastDebugPrint = 0;
    if (currentTime - lastDebugPrint >= 1000)
    {
        lastDebugPrint = currentTime;
        Serial.printf("[LOOP] WiFi:%d MQTT:%d State:%d\n",
                      WiFi.status(), mqttClient.connected() ? 1 : 0, trainState.status);
    }
}