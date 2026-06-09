#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H

// --- Wi-Fi Network Credentials ---
const char *WIFI_SSID = "Airtel_Primerail Infralabs Pvt L";
const char *WIFI_PASSWORD = "Primeit@2024";

// --- MQTT Broker Configuration ---
// Using public EMQX broker instead of local Mosquitto service.
const char *MQTT_BROKER_IP = "broker.emqx.io";
const int MQTT_BROKER_PORT = 1883;

// --- CBTC Operational Channels ---
const char *TOPIC_TELEMETRY = "vehicle/telemetry";
const char *TOPIC_AUTHORITY = "wayside/authority";
const char *TOPIC_FAULT = "wayside/fault";

// --- ESP32-S3 Hardware Pin Allocations ---
// Secure hardware pins for the S3 chip to avoid internal flash conflicts
const int MODBUS_RX_PIN = 17;         // Connects to RO on your RS485 Transceiver
const int MODBUS_TX_PIN = 18;         // Connects to DI on your RS485 Transceiver
const int WATCHDOG_TIMEOUT_MS = 1000; // Hard radio link safety timeout threshold

#endif