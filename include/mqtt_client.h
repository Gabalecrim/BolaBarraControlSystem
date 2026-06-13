#pragma once

#include <PubSubClient.h>
#include "wifi_service.h"

extern PubSubClient client;

void callback(char* topic, byte* payload, unsigned int length);

void reconnect();