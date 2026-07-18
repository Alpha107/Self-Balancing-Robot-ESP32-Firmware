#pragma once
#include <Arduino.h>
#include <WebSocketsClient.h>
#include <functional>

class WSClient {
public:
    using PidUpdateCallback = std::function<void(float kp, float ki, float kd)>;

    void begin(const char* host, uint16_t port, const char* path);
    void loop();

    void sendTelemetry(float angle, float output, float kp, float ki, float kd, unsigned long uptimeMs);
    void onPidUpdate(PidUpdateCallback cb) { _pidCallback = cb; }

    bool isConnected() const { return _connected; }

private:
    void handleEvent(WStype_t type, uint8_t* payload, size_t length);
    static void eventTrampoline(WStype_t type, uint8_t* payload, size_t length);

    static WSClient* _instance; // library callback is a plain fn pointer, not a member fn
    WebSocketsClient _ws;
    bool _connected = false;
    PidUpdateCallback _pidCallback;
};
