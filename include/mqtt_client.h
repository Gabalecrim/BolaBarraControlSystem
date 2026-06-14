#pragma once

#include <PubSubClient.h>
#include "wifi_manager.h"

extern PubSubClient client;

void callback(char* topic, byte* payload, unsigned int length);

void reconnect();