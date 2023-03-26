#include <ESP8266WiFi.h>
#include <lwip/dns.h>
#include <Arduino.h>

void connect_wifi(String hostname, String ssid, String password) {
    Serial.println();
    Serial.println("connecting to ");
    Serial.println(ssid);
    ESP8266WiFiClass::persistent(false);
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.hostname(hostname);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.println(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("DNS1: ");
    Serial.println(IPAddress(dns_getserver(0)));
    Serial.println("DNS2: ");
    Serial.println(IPAddress(dns_getserver(1)));
}


String mac_string() {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char mac_string[6 * 2 + 1] = {0};
    snprintf(mac_string, 6 * 2 + 1, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(mac_string);
}