#pragma once

#include <PID_v1.h>
#include <PubSubClient.h>
#include "wifi_manager.h"

extern PubSubClient client;
extern PID processPID;

extern double Kp;
extern double Ki;
extern double Kd;
extern double Setpoint;

extern float SAMPLE_TIME_MS;
extern float ALPHA;
extern float velMax;
extern float acelMax;
extern float jerkMax;
extern float ganhoPos;
extern float ganhoVel;

void callback(char* topic, byte* payload, unsigned int length);

void reconnect();

void publishConfig();
