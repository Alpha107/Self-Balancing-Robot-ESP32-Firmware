#include <Arduino.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>

#include "Config.h"
#include "Secrets.h"
#include "IMU.h"
#include "MotorDriver.h"
#include "PIDController.h"
#include "WSClient.h"

IMU imu;
MotorDriver motors;
PIDController pid(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT, PID_OUTPUT_LIMIT);
WSClient ws;
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);

bool fallen = false;
unsigned long lastControlUs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastLcdMs = 0;
unsigned long lastSwitchMs = 0;

const unsigned long CONTROL_PERIOD_US = 1000000UL / CONTROL_LOOP_HZ;

void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connecting WiFi");

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(250);
        Serial.print(".");
    }

    lcd.clear();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("WiFi connected, IP: ");
        Serial.println(WiFi.localIP());
        lcd.setCursor(0, 0);
        lcd.print("WiFi OK");
    } else {
        Serial.println("WiFi connect failed, continuing offline");
        lcd.setCursor(0, 0);
        lcd.print("WiFi FAILED");
    }
    delay(500);
}

void onPidUpdate(float kp, float ki, float kd) {
    pid.setGains(kp, ki, kd);
    Serial.printf("[PID] updated Kp=%.2f Ki=%.2f Kd=%.2f\n", kp, ki, kd);
}

void setup() {
    Serial.begin(115200);
    delay(200);

    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_CALIBRATE_SWITCH, INPUT_PULLUP);

    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Balancing Robot");
    lcd.setCursor(0, 1);
    lcd.print("Initializing...");

    if (!imu.begin()) {
        Serial.println("IMU init failed! Check wiring.");
        lcd.clear();
        lcd.print("IMU NOT FOUND");
        while (true) delay(1000);
    }

    lcd.setCursor(0, 1);
    lcd.print("Calibrating IMU");
    imu.calibrate();

    motors.begin();

    connectWiFi();
    ws.begin(WS_HOST, WS_PORT, WS_PATH);
    ws.onPidUpdate(onPidUpdate);

    pid.setSetpoint(0.0f);
    lastControlUs = micros();
    lastTelemetryMs = millis();
    lastLcdMs = millis();
}

void loop() {
    ws.loop();

    // Hold robot upright and press the switch to re-zero the angle.
    if (digitalRead(PIN_CALIBRATE_SWITCH) == LOW && millis() - lastSwitchMs > 500) {
        lastSwitchMs = millis();
        motors.stop();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Calibrating...");
        imu.calibrate();
        pid.reset();
        fallen = false;
    }

    unsigned long nowUs = micros();
    if (nowUs - lastControlUs < CONTROL_PERIOD_US) return;
    float dt = (nowUs - lastControlUs) / 1000000.0f;
    lastControlUs = nowUs;

    imu.update();
    float pitch = imu.getPitch();

    if (fabsf(pitch) > FALL_ANGLE_DEG) {
        fallen = true;
    } else if (fallen && fabsf(pitch) < FALL_ANGLE_DEG * 0.5f) {
        // Robot was picked back up and re-righted; resume automatically.
        fallen = false;
        pid.reset();
    }

    float output = 0.0f;
    if (fallen) {
        motors.stop();
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_RED, HIGH);
    } else {
        output = pid.compute(pitch, dt);
        int speed = (int)output;
        motors.setSpeed(speed, speed);
        digitalWrite(PIN_LED_GREEN, HIGH);
        digitalWrite(PIN_LED_RED, LOW);
    }

    unsigned long nowMs = millis();

    if (nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryMs = nowMs;
        ws.sendTelemetry(pitch, output, pid.getKp(), pid.getKi(), pid.getKd(), nowMs);
    }

    if (nowMs - lastLcdMs >= LCD_UPDATE_INTERVAL_MS) {
        lastLcdMs = nowMs;
        lcd.setCursor(0, 0);
        lcd.printf("A:%6.1f %s   ", pitch, fallen ? "FALL" : "OK  ");
        lcd.setCursor(0, 1);
        lcd.printf("%s %4d", ws.isConnected() ? "WS:UP  " : "WS:DOWN", (int)output);
    }
}
