#pragma once

#include <pgmspace.h>
#include "battery_type.h"

BatteryType battery_type = BatteryType::meb12s;

constexpr char ssid[] = "Vodafone-A054";
constexpr char password[] = "ZfPsTdk74daGbGFp";

constexpr char mqtt_server[] = "meb-hauskraftwerk2.local";
constexpr uint16_t mqtt_port = 1883;
constexpr char mqtt_username[] = "easy-bms";
constexpr char mqtt_password[] = "ijoo";

const char mqtt_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
INSERT PEM CERTIFICATE HERE
-----END CERTIFICATE-----
)EOF";
inline X509List mqtt_cert_store(mqtt_cert);
