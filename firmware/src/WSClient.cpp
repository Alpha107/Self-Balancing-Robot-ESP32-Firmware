#include "WSClient.h"
#include "Config.h"
#include <ArduinoJson.h>
#include <string.h>

WSClient* WSClient::_instance = nullptr;

void WSClient::begin(const char* host, uint16_t port, const char* path) {
    _instance = this;
    _ws.begin(host, port, path);
    _ws.onEvent(eventTrampoline);
    _ws.setReconnectInterval(WS_RECONNECT_INTERVAL_MS);
}

void WSClient::loop() {
    _ws.loop();
}

void WSClient::eventTrampoline(WStype_t type, uint8_t* payload, size_t length) {
    if (_instance) _instance->handleEvent(type, payload, length);
}

void WSClient::handleEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            _connected = true;
            Serial.println("[WS] connected");
            break;

        case WStype_DISCONNECTED:
            _connected = false;
            Serial.println("[WS] disconnected");
            break;

        case WStype_TEXT: {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, payload, length);
            if (err) {
                Serial.print("[WS] bad JSON: ");
                Serial.println(err.c_str());
                return;
            }
            const char* msgType = doc["type"] | "";
            if (_pidCallback && strcmp(msgType, "pid_update") == 0
                && doc["kp"].is<float>() && doc["ki"].is<float>() && doc["kd"].is<float>()) {
                _pidCallback(doc["kp"].as<float>(), doc["ki"].as<float>(), doc["kd"].as<float>());
            }
            break;
        }

        default:
            break;
    }
}

void WSClient::sendTelemetry(float angle, float output, float kp, float ki, float kd, unsigned long uptimeMs) {
    if (!_connected) return;

    JsonDocument doc;
    doc["type"] = "telemetry";
    doc["angle"] = angle;
    doc["output"] = output;
    doc["kp"] = kp;
    doc["ki"] = ki;
    doc["kd"] = kd;
    doc["uptime_ms"] = uptimeMs;

    String out;
    serializeJson(doc, out);
    _ws.sendTXT(out);
}
