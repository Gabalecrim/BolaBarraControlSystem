#pragma once

#include <WiFi.h>

extern WiFiClient espClient;

void setup_wifi();

void printWifiStatus();