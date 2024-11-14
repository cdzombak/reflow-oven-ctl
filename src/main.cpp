#include <Arduino.h>
#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMQTTClient.h> // https://github.com/HeMan/async-mqtt-client
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library
#include <MAX6675.h> // https://github.com/adafruit/MAX6675-library
#include <ArduinoOTA.h> // https://github.com/espressif/arduino-esp32/blob/master/libraries/ArduinoOTA/examples/BasicOTA/BasicOTA.ino

#include "config.h"

#define PIN_OVEN_SSR 32
#define PIN_ONBOARD_LED 2
#define PIN_READY_LIGHT 33
#define PIN_PERMERR_LIGHT 25
#define PIN_SPI_THERMO_CS_A 12
#define PIN_SPI_THERMO_CS_B 14
#define PIN_SPI_SCK 18
#define PIN_SPI_MISO 19

#define DISAGREE_THRESHOLD 15.0
#define SLOW_RESPONSE_INTERVAL 55000
#define SLOW_RESPONSE_THRESHOLD 4.0

#define WIFI_BLINK_INTERVAL 1000
#define MQTT_BLINK_INTERVAL 300

#define OVEN_PID_kP 0.75
#define OVEN_PID_kI 0.0
#define OVEN_PID_kD 6.0
#define OVEN_PWM_WINDOW 10000

#define TC_READ_INTERVAL 500

#define PARSE_RESULT_OK 0
#define PARSE_RESULT_ERROR 1

#define VALIDATION_RESULT_OK 0
#define VALIDATION_RESULT_TOO_SHORT 1
#define VALIDATION_RESULT_TOO_LONG 2
#define VALIDATION_RESULT_BAD_START 3
#define VALIDATION_RESULT_BAD_END 4

#define CMD_NONE 0
#define CMD_RUN 1
#define CMD_STOP 2

#define MAX_PROGRAM_LEN (1024 * 8)

// permanent error flag (requires physical reset to clear):
volatile bool permErr = false;

// user program state:
bool running = false;
uint programLine = 0;
unsigned long runStartedAt = 0;
String runProgram = "";
uint nextSetpointAt = 0;
double nextSetpoint = 0;
bool isNextSetpointEnd = false;
int validateProgram(); // validates the current program; returns a VALIDATION_RESULT
int parseNextProgramLine(); // parses the next line from the current program into nextSetpoint/nextSetpointAt/isNextSetpointEnd; returns a PARSE_RESULT

// oven & PID state:
double setpoint;
double ovenPidIn, ovenPidOut;
PID ovenPID(&ovenPidIn, &ovenPidOut, &setpoint, OVEN_PID_kP, OVEN_PID_kI, OVEN_PID_kD, P_ON_M, DIRECT);
unsigned long ovenRelayOnAt = 0;
unsigned long ovenWindowStartAt = 0;
float tempAtOvenRelayOn = 0.0;

// thermocouple state:
MAX6675 tcA(PIN_SPI_SCK, PIN_SPI_THERMO_CS_A, PIN_SPI_MISO);
MAX6675 tcB(PIN_SPI_SCK, PIN_SPI_THERMO_CS_B, PIN_SPI_MISO);
unsigned long lastTcReadAt = 0;
float tempA;
float tempB;

// WiFi & MQTT state:
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
unsigned long lastStatusBroadcastAt = 0;
volatile int nextCmd = CMD_NONE;
String nextCmdPayload;

// pre-concatenate MQTT topics:
String mqttStateTopic_IP = String(CFG_MQTT_STATE_TOPIC) + "/ip";
String mqttStateTopic_WifiSignalStrength = String(CFG_MQTT_STATE_TOPIC) + "/wifi_signal_strength";
String mqttStateTopic_PermErr = String(CFG_MQTT_STATE_TOPIC) + "/perm_err";
String mqttStateTopic_Running = String(CFG_MQTT_STATE_TOPIC) + "/running";
String mqttStateTopic_Setpoint = String(CFG_MQTT_STATE_TOPIC) + "/setpoint";
String mqttStateTopic_Oven = String(CFG_MQTT_STATE_TOPIC) + "/oven";
String mqttStateTopic_CurrentTempA = String(CFG_MQTT_STATE_TOPIC) + "/temp_a";
String mqttStateTopic_CurrentTempB = String(CFG_MQTT_STATE_TOPIC) + "/temp_b";
String mqttStateTopic_RunDuration = String(CFG_MQTT_STATE_TOPIC) + "/run_duration";
String mqttStateTopic_Ready = String(CFG_MQTT_STATE_TOPIC) + "/ready";

// OTA:
volatile bool didOTAStart = false;
int lastOTAProgress = -1;

// logging forward declarations:
void otaMessage(String message);
void logMessage(String message);
void debugMessage(String message);

// preset programs:

// SnBi
String snBiProgram() {
    return "0 100\n30 150\n120 183\n150 235\n210 183\n240 END";
    // return "0 100\n90 150\n200 183\n230 235\n300 183\n360 END";
}

// SnPb
String snPbProgram() {
    return "0 90\n90 130\n180 138\n210 165\n240 138\n270 END";
}

// SilicaDry
String silicaDryProgram() {
    // 2h at 95C
    return "0 95\n7200 END";
}

// Preheat
String preheatProgram() {
    return "0 80\n300 END";
}

// wifi & mqtt callbacks:

void startOTA() {
    if (didOTAStart) {
        return;
    }
    didOTAStart = true;

    ArduinoOTA.setPort(8266);
    ArduinoOTA.setHostname(CFG_HOSTNAME);
    ArduinoOTA.setRebootOnSuccess(true);

    ArduinoOTA.onStart([]() {
        runProgram = "";
        running = false;
        digitalWrite(PIN_OVEN_SSR, LOW);
        digitalWrite(PIN_ONBOARD_LED, HIGH);
        digitalWrite(PIN_READY_LIGHT, HIGH);
        digitalWrite(PIN_PERMERR_LIGHT, HIGH);
        #ifdef CFG_MQTT_OTA_TOPIC
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {  // U_SPIFFS
            type = "filesystem";
        }
        otaMessage("OTA Start: " + type);
        #endif
    }).onProgress([](unsigned int progress, unsigned int total) {
        #ifdef CFG_MQTT_OTA_TOPIC
        int pct = progress / (total / 100);
        if (pct != lastOTAProgress && pct % 5 == 0) {
            lastOTAProgress = pct;
            otaMessage("OTA Progress: " + String(pct) + "%");
        }
        #endif
    }).onError([](ota_error_t error) {
        permErr = true;
        digitalWrite(PIN_ONBOARD_LED, LOW);
        digitalWrite(PIN_READY_LIGHT, LOW);
        digitalWrite(PIN_PERMERR_LIGHT, HIGH);
        logMessage("OTA failed. Physical reset required.");

        #ifdef CFG_MQTT_OTA_TOPIC
        if (error == OTA_AUTH_ERROR) {
            otaMessage("OTA: Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            otaMessage("OTA: Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            otaMessage("OTA: Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            otaMessage("OTA: Receive Failed");
        } else if (error == OTA_END_ERROR) {
            otaMessage("OTA: End Failed");
        } else {
            otaMessage("OTA Error: " + String(error));
        }
        #endif
    });

    ArduinoOTA.begin();
}

void connectWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.hostname(CFG_HOSTNAME);
    WiFi.begin(CFG_WIFI_ESSID, CFG_WIFI_PASSWORD);
}

void connectMqtt() {
    mqttClient.connect();
}

void onWiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            connectMqtt();
            startOTA();
            break;

        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            xTimerStop(mqttReconnectTimer, 0); // don't reconnect to MQTT while reconnecting to WiFi
            xTimerStart(wifiReconnectTimer, 0);
            break;
    }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    if (WiFi.isConnected()) {
        xTimerStart(mqttReconnectTimer, 0);
    }
}

void onMqttConnect(bool sessionPresent) {
    mqttClient.subscribe(CFG_MQTT_COMMAND_TOPIC, 0);
}

String mqttRunCmdBuf;
void onMqttMessage(char* topic, char* payload, const AsyncMqttClientMessageProperties& properties,
                   const size_t& len, const size_t& index, const size_t& total) {
    String topicStr = String(topic);
    if (topicStr.endsWith("/cmd/run")) {
        // ref: https://github.com/marvinroger/async-mqtt-client/issues/98#issuecomment-383327071
        if (index == 0) {
		    mqttRunCmdBuf = "";
	    }
        auto pl = len;
        auto p = payload;
        while (pl--) {
            mqttRunCmdBuf += *(p++);
        }
	    if (index + len == total) {
            nextCmdPayload = mqttRunCmdBuf;
            mqttRunCmdBuf = "";
            nextCmdPayload.trim();
            nextCmd = CMD_RUN;
    	}
    } else if(topicStr.endsWith("/cmd/stop")) {
        nextCmd = CMD_STOP;
    } else {
        nextCmd = CMD_NONE;
    }
}

// reflow oven program core:

void ovenRelayOn(float atTemp) {
    digitalWrite(PIN_OVEN_SSR, HIGH);
    ovenRelayOnAt = millis();
    tempAtOvenRelayOn = atTemp;
}

void ovenRelayOff() {
    digitalWrite(PIN_OVEN_SSR, LOW);
    ovenRelayOnAt = 0;
    tempAtOvenRelayOn = 0.0;
}

// figure out whether the oven should be on or off right now,
// based on PID output and program run state.
void setOvenRelayPWM(float currentTemp) {
    if (!running) {
        ovenRelayOff();
        return;
    }

    if (millis() - ovenWindowStartAt > OVEN_PWM_WINDOW) {
        ovenWindowStartAt += OVEN_PWM_WINDOW;
    }

    bool isRelayOn = digitalRead(PIN_OVEN_SSR) == HIGH;
    ovenPidIn = currentTemp + 2.0;
    ovenPID.Compute();

    if (ovenPidOut < 1000) {
        ovenRelayOff();
        return;
    } else if (ovenPidOut > (OVEN_PWM_WINDOW - 1000)) {
        if (!isRelayOn) {
            ovenRelayOn(currentTemp);
        }
        return;
    }

    if (ovenPidOut < (millis() - ovenWindowStartAt) && !isRelayOn) {
        ovenRelayOn(currentTemp);
    } else {
        ovenRelayOff();
    }
}

// toggling ready light is handled by this function to allow blink logic:
unsigned long readyLightLastToggleAt = 0;
void toggleReadyLight() {
    digitalWrite(PIN_READY_LIGHT, !digitalRead(PIN_READY_LIGHT));
    digitalWrite(PIN_ONBOARD_LED, !digitalRead(PIN_ONBOARD_LED));
    readyLightLastToggleAt = millis();
}

void setup() {
    pinMode(PIN_OVEN_SSR, OUTPUT);
    digitalWrite(PIN_OVEN_SSR, LOW);
    pinMode(PIN_READY_LIGHT, OUTPUT);
    digitalWrite(PIN_READY_LIGHT, LOW);
    pinMode(PIN_ONBOARD_LED, OUTPUT);
    digitalWrite(PIN_ONBOARD_LED, LOW);
    pinMode(PIN_PERMERR_LIGHT, OUTPUT);
    digitalWrite(PIN_PERMERR_LIGHT, LOW);

    ovenPID.SetOutputLimits(0, OVEN_PWM_WINDOW);
    ovenPID.SetMode(AUTOMATIC);

    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                      reinterpret_cast<TimerCallbackFunction_t>(connectMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                      reinterpret_cast<TimerCallbackFunction_t>(connectWifi));
    WiFi.onEvent(onWiFiEvent);
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.setCleanSession(true);
    mqttClient.setServer(CFG_MQTT_HOST, CFG_MQTT_PORT);
    #ifdef CFG_MQTT_PASSWORD
    mqttClient.setCredentials(CFG_MQTT_USER, CFG_MQTT_PASSWORD);
    #endif
    connectWifi();
}

void loop() {
    if (didOTAStart) {
        ArduinoOTA.handle();
    }

    if (millis() - lastTcReadAt > TC_READ_INTERVAL) {
        lastTcReadAt = millis();
        tempA = tcA.readCelsius();
        tempB = tcB.readCelsius();
    }

    if (mqttClient.connected() && millis() - lastStatusBroadcastAt > 1000) {
        lastStatusBroadcastAt = millis();

        mqttClient.publish(mqttStateTopic_IP.c_str(), 0, false, WiFi.localIP().toString().c_str());
        mqttClient.publish(mqttStateTopic_WifiSignalStrength.c_str(), 0, false, String(WiFi.RSSI()).c_str());
        mqttClient.publish(mqttStateTopic_PermErr.c_str(), 0, false, permErr ? "true" : "false");
        mqttClient.publish(mqttStateTopic_Running.c_str(), 0, false, running ? "true" : "false");
        mqttClient.publish(mqttStateTopic_Oven.c_str(), 0, false, digitalRead(PIN_OVEN_SSR) == HIGH ? "true" : "false");

        if (!isnan(tempA) && tempA != 0.0) {
            mqttClient.publish(mqttStateTopic_CurrentTempA.c_str(), 0, false, String(tempA).c_str());
        }
        if (!isnan(tempB) && tempB != 0.0) {
            mqttClient.publish(mqttStateTopic_CurrentTempB.c_str(), 0, false, String(tempB).c_str());
        }

        if (running) {
            mqttClient.publish(mqttStateTopic_Setpoint.c_str(), 0, false, String(setpoint).c_str());
            mqttClient.publish(mqttStateTopic_RunDuration.c_str(), 0, false, String((millis() - runStartedAt) / 1000.0).c_str());
        }

        bool ready = WiFi.isConnected() && mqttClient.connected() && !permErr;
        mqttClient.publish(mqttStateTopic_Ready.c_str(), 0, false, ready ? "true" : "false");

        yield();
    }

    if (permErr) {
        // permanent failure; requires power cycle to clear
        ovenRelayOff();
        return;
    }
    if (!running) {
        // safety/sanity check
        ovenRelayOff();
    }

    if (!WiFi.isConnected()) {
        if (millis() - readyLightLastToggleAt > WIFI_BLINK_INTERVAL) {
            toggleReadyLight();
        }
    } else if (!mqttClient.connected()) {
        if (millis() - readyLightLastToggleAt > MQTT_BLINK_INTERVAL) {
            toggleReadyLight();
        }
    } else {
        digitalWrite(PIN_READY_LIGHT, HIGH);
        digitalWrite(PIN_ONBOARD_LED, HIGH);
    }

    bool triggerPermErr = false;
    bool triggerProgramErr = false;
    bool triggerProgramEnded = false;

    if (!running && runProgram.length() > 0) {
        runProgram.trim();
        if (runProgram == "SnBi") {
            runProgram = snBiProgram();
        } else if (runProgram == "SnPb") {
            runProgram = snPbProgram();
        } else if (runProgram == "SilicaDry") {
            runProgram = silicaDryProgram();
        } else if (runProgram == "Preheat") {
            runProgram = preheatProgram();
        }

        running = true;
        runStartedAt = millis()-1;
        ovenWindowStartAt = millis();
        ovenRelayOnAt = 0;
        programLine = 0;
        ovenPidIn = 0;
        ovenPidOut = 0;
        setpoint = 0;

        int validationResult = validateProgram();
        if (validationResult == VALIDATION_RESULT_TOO_SHORT) {
            triggerProgramErr = true;
            logMessage("Error: invalid program (too short). Abort.");
        } else if (validationResult == VALIDATION_RESULT_TOO_LONG) {
            triggerProgramErr = true;
            logMessage("Error: invalid program (longer than 8 KB). Abort.");
        } else if (validationResult == VALIDATION_RESULT_BAD_START) {
            triggerProgramErr = true;
            logMessage("Error: invalid program (does not start with '0 '). Abort.");
        } else if (validationResult == VALIDATION_RESULT_BAD_END) {
            triggerProgramErr = true;
            logMessage("Error: invalid program (does not end with ' END'). Abort.");
        } else if (validationResult != VALIDATION_RESULT_OK) {
            triggerProgramErr = true;
            logMessage("Error: invalid program. Abort.");
        }
        if (triggerProgramErr) {
            goto END_OF_USER_PROGRAM_EXECUTION;
        }

        int parseResult = parseNextProgramLine();
        if (parseResult != PARSE_RESULT_OK) {
            triggerProgramErr = true;
            logMessage("Error: invalid program (line 1). Abort.");
            goto END_OF_USER_PROGRAM_EXECUTION;
        }
        yield();

        logMessage("Starting program.");
    }
    if (running) {
        if (runStartedAt + (1000 * nextSetpointAt) < millis()) {
            if (isNextSetpointEnd) {
                logMessage("Program ended.");
                triggerProgramEnded = true;
                goto END_OF_USER_PROGRAM_EXECUTION;
            }

            // advance to the next setpoint:
            setpoint = nextSetpoint;

            // advance the program; find next setpoint and timestamp (in seconds):
            int parseResult = parseNextProgramLine();
            if (parseResult != PARSE_RESULT_OK) {
                triggerProgramErr = true;
                logMessage("Error: invalid program (line " + String(programLine) + "). Abort.");
                goto END_OF_USER_PROGRAM_EXECUTION;
            }
            yield();
        }

        // check for NAN:
        if (isnan(tempA) || isnan(tempB) || tempA == 0.0 || tempB == 0.0) {
            triggerPermErr = true;
            logMessage("Error: thermocouple read returned NaN or 0. Abort. Physical intervention required.");
            goto END_OF_USER_PROGRAM_EXECUTION;
        }

        // check for tc disagree:
        if (abs(tempA - tempB) > DISAGREE_THRESHOLD) {
            triggerPermErr = true;
            logMessage("Error: thermocouples disagree. Abort. Physical intervention required.");
            goto END_OF_USER_PROGRAM_EXECUTION;
        }

        float tempAvg = (tempA + tempB) / 2.0;
        bool isRelayOn = digitalRead(PIN_OVEN_SSR) == HIGH;
        // check for slow response:
        if (isRelayOn
            && millis() - ovenRelayOnAt > SLOW_RESPONSE_INTERVAL
            && abs(tempAvg - tempAtOvenRelayOn) < SLOW_RESPONSE_THRESHOLD
        ) {
            triggerPermErr = true;
            logMessage("Error: oven temperature responded too slowly. Abort. Physical intervention required.");
            goto END_OF_USER_PROGRAM_EXECUTION;
        }

        setOvenRelayPWM(tempAvg);
    }

END_OF_USER_PROGRAM_EXECUTION:
    if (triggerProgramEnded || triggerPermErr || triggerProgramErr) {
        ovenRelayOff();
        if (triggerPermErr) {
            permErr = true;
            digitalWrite(PIN_READY_LIGHT, LOW);
            digitalWrite(PIN_PERMERR_LIGHT, HIGH);
        }

        if (triggerProgramErr) {
            debugMessage(runProgram);
        }

        runProgram = "";
        running = false;
        runStartedAt = 0;
        ovenRelayOnAt = 0;
        programLine = 0;
        ovenPidIn = 0;
        ovenPidOut = 0;
        setpoint = 0;
        isNextSetpointEnd = false;
        nextSetpoint = 0;
        nextSetpointAt = 0;
    }

    if (permErr) {
        return;
    }

    yield();

    // handle any pending command:
    if (nextCmd != CMD_NONE) {
        switch (nextCmd) {
            case CMD_RUN:
                if (running) {
                    logMessage("Error: program already running.");
                } else {
                    runProgram = nextCmdPayload;
                }
                break;
            case CMD_STOP:
                if (running) {
                    runProgram = "";
                    nextSetpointAt = 0;
                    nextSetpoint = 0.0;
                    isNextSetpointEnd = true;
                    ovenRelayOff();
                }
                break;
            break;
        }
        nextCmd = CMD_NONE;
    }
}

// user program validation & parsing:

int validateProgram() {
    if (runProgram.length() < 9) {
        return VALIDATION_RESULT_TOO_SHORT;
    }
    if (runProgram.length() > MAX_PROGRAM_LEN) {
        return VALIDATION_RESULT_TOO_LONG;
    }
    if (!runProgram.startsWith("0 ")) {
        return VALIDATION_RESULT_BAD_START;
    }
    if (!runProgram.endsWith(" END")) {
        return VALIDATION_RESULT_BAD_END;
    }

    return VALIDATION_RESULT_OK;
}

int parseNextProgramLine() {
    programLine++;

    String line;
    int nextNewlineIdx = runProgram.indexOf('\n');
    if (nextNewlineIdx != -1) {
        line = runProgram.substring(0, nextNewlineIdx);
        runProgram = runProgram.substring(nextNewlineIdx + 1);
    } else {
        line = runProgram;
        runProgram = "";
    }

    int spaceIdx = line.indexOf(' ');
    if (spaceIdx == -1) {
        nextSetpoint = 0.0;
        nextSetpointAt = 0;
        isNextSetpointEnd = false;
        return PARSE_RESULT_ERROR;
    }

    auto nextSetpointStr = line.substring(spaceIdx + 1);
    if (nextSetpointStr.startsWith("END")) {
        isNextSetpointEnd = true;
    } else {
        errno = 0;
        nextSetpoint = nextSetpointStr.toFloat();
        if (errno != 0) {
            nextSetpoint = 0.0;
            nextSetpointAt = 0;
            isNextSetpointEnd = false;
            return PARSE_RESULT_ERROR;
        }
    }

    errno = 0;
    nextSetpointAt = line.substring(0, spaceIdx).toInt();
    if (errno != 0) {
        nextSetpoint = 0.0;
        nextSetpointAt = 0;
        isNextSetpointEnd = false;
        return PARSE_RESULT_ERROR;
    }

    return PARSE_RESULT_OK;
}

// logging to mqtt:

void logMessage(String message) {
    mqttClient.publish(CFG_MQTT_MESSAGE_TOPIC, 1, true, message.c_str());
}

void debugMessage(String message) {
    #ifdef CFG_MQTT_DEBUG_TOPIC
    mqttClient.publish(CFG_MQTT_DEBUG_TOPIC, 1, true, message.c_str());
    #endif
}

void otaMessage(String message) {
    #ifdef CFG_MQTT_OTA_TOPIC
    mqttClient.publish(CFG_MQTT_OTA_TOPIC, 1, true, message.c_str());
    #endif
}
