#include <ESP8266WiFi.h>
#include <LTC6804.h>
#include <LTC6804.cpp>
#include <lwip/dns.h>
#include <PubSubClient.h>

#include "credentials.h"

#define DEBUG

#ifdef DEBUG
#define DEBUG_BEGIN(...) Serial.begin( __VA_ARGS__ )
#define DEBUG_PRINT(...) Serial.print( __VA_ARGS__ )
#define DEBUG_PRINTLN(...) Serial.println( __VA_ARGS__ )
#else
#define DEBUG_BEGIN(...)
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

static LTC68041 LTC = LTC68041(D8);

static const long MASTER_TIMEOUT = 5000;

unsigned int module_index = 9;
String hostname = String("esp-module-") + module_index;
String module_topic = String("esp-module/") + module_index;

WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, espClient);

std::array<unsigned long, 12> cells_to_balance{};

unsigned long last_connection = 0;
//bool is_total_voltage_measurer = false

// connect to wifi
void connectWifi() {
    DEBUG_PRINTLN();
    DEBUG_PRINT("connecting to ");
    DEBUG_PRINTLN(ssid);
    ESP8266WiFiClass::persistent(false);
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.hostname(hostname);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        DEBUG_PRINT(".");
    }
    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("WiFi connected");
    DEBUG_PRINTLN("IP address: ");
    DEBUG_PRINTLN(WiFi.localIP());

    DEBUG_PRINT("DNS1: ");
    DEBUG_PRINTLN(IPAddress(dns_getserver(0)));
    DEBUG_PRINT("DNS2: ");
    DEBUG_PRINTLN(IPAddress(dns_getserver(1)));
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        DEBUG_PRINT("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect(hostname.c_str(), mqtt_username, mqtt_password, (module_topic + "/available").c_str(), 0,
                           true, "offline")) {
            DEBUG_PRINTLN("connected");
            // Once connected, publish an announcement...
            client.publish((module_topic + "/available").c_str(), "online", true);
            // ... and resubscribe
            client.subscribe("master/uptime");
            for (int i = 0; i < 12; ++i) {
                client.subscribe((module_topic + "/cell/" + (i + 1) + "/balance/set").c_str());
            }
        } else {
            DEBUG_PRINT("failed, rc=");
            DEBUG_PRINT(client.state());
            //   DEBUG_PRINTLN(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            // delay(5000);
            if (last_connection != 0 && last_connection + 30000 < millis()) {
                ESP.restart();
            }
            delay(1000);
        }
    }
}

String payload_to_string(byte *payload, unsigned int length) {
    String result = "";
    for (unsigned int i = 0; i < length; i++) {
        result += static_cast<char>(payload[i]);
    }
    return result;
}

long last_master_uptime = 0;

void callback(char *topic, byte *payload, unsigned int length) {
    String topic_string = String(topic);
    if (topic_string.equals("master/uptime")) {
        String uptime = payload_to_string(payload, length);
        DEBUG_PRINT("Got heartbeat from master: ");
        DEBUG_PRINTLN(uptime);

        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
        digitalWrite(LED_BUILTIN, HIGH);

        long uptime_long = uptime.toInt();

        if (uptime_long - last_master_uptime > MASTER_TIMEOUT) {
            DEBUG_PRINTLN(">>> Master Timeout!!");
        }
    } else if (topic_string.startsWith(module_topic)) {
        if (topic_string.startsWith(module_topic + "/cell/")) {
            String get_cell = topic_string.substring((module_topic + "/cell/").length());
            get_cell = get_cell.substring(0, get_cell.indexOf("/"));
            long cell_number = get_cell.toInt();
            if (topic_string.equals(module_topic + "/cell/" + cell_number + "/balance_request")) {
                long balance_time = payload_to_string(payload, length).toInt();
                cells_to_balance.at(cell_number - 1) = millis() + balance_time;
            }
        }
    }
}

// the setup function runs once when you press reset or power the board
[[maybe_unused]] void setup() {
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(D5, OUTPUT);
    pinMode(D1, OUTPUT);

    DEBUG_BEGIN(74880);
    DEBUG_PRINT(hostname);
    DEBUG_PRINTLN(" init");

    connectWifi();
    randomSeed(micros());

    LTC.initSPI(D7, D6, D5); //Initialize LTC6804 hardware

    client.setCallback(callback);
}

// the loop function runs over and over again forever
void loop() {
    unsigned long uptime_millis = millis();

    if (LTC.checkSPI(true)) {
        digitalWrite(D1, HIGH);
        DEBUG_PRINTLN();
        DEBUG_PRINTLN("SPI ok");
    } else {
        digitalWrite(D1, LOW);
        DEBUG_PRINTLN();
        DEBUG_PRINTLN("SPI lost");
    }

    LTC.cfgSetVUV(3.1);
    LTC.cfgSetVOV(4.2);

    std::bitset<12> balance_bits{};
    for (size_t i = 0; i < cells_to_balance.size(); ++i) {
        if (cells_to_balance[i] >= uptime_millis) {
            balance_bits[i] = true;
        }
    }
    if (balance_bits.any()) {
        digitalWrite(D2, HIGH);
    } else {
        digitalWrite(D2, LOW);
    }
    LTC.cfgSetDCC(balance_bits);

    LTC.cfgWrite();
    //Start different Analog-Digital-Conversions in the Chip

    LTC.startCellConv(LTC68041::DCP_DISABLED);
    delay(5); //Wait until conversion is finished
    LTC.startAuxConv();
    delay(5); //Wait until conversion is finished
    LTC.startStatusConv();
    delay(5); //Wait until conversion is finished

    //Read the raw values into the controller
    std::array<float, 6> voltages{};
    LTC.getCellVoltages(voltages,
                        LTC68041::CH_ALL); // read all channel (2nd parameter default), use only 6 (size of array)
    LTC.cfgRead();

    //Print the clear text values cellVoltage, gpioVoltage, Undervoltage Bits, Overvoltage Bits
    LTC.readCfgDbg();
    LTC.readStatusDbg();
    LTC.readAuxDbg();
    LTC.readCellsDbg();

    std::array<float, 12> cell_voltages{};
    LTC.getCellVoltages(cell_voltages);
    float chip_temp = LTC.getStatusVoltage(LTC.StatusGroup::CHST_ITMP);

    float module_voltage = LTC.getStatusVoltage(LTC68041::CHST_SOC);

    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    last_connection = uptime_millis;

    client.publish((module_topic + "/uptime").c_str(), String(uptime_millis).c_str(), true);

    for (size_t i = 0; i < cell_voltages.size(); i++) {
        client.publish((module_topic + "/cell/" + String(i + 1) + "/voltage").c_str(), String(cell_voltages[i]).c_str(),
                       true);
        if (balance_bits.test(i)) {
            client.publish((module_topic + "/cell/" + String(i + 1) + "/is_balancing").c_str(), "1", true);
        } else {
            client.publish((module_topic + "/cell/" + String(i + 1) + "/is_balancing").c_str(), "0", true);
        }
    }
    client.publish((module_topic + "/module_voltage").c_str(), String(module_voltage).c_str(), true);
    client.publish((module_topic + "/module_temps").c_str(), (String(25.0) + "," + String(25.1)).c_str(), true);
    client.publish((module_topic + "/chip_temp").c_str(), String(chip_temp).c_str(), true);

    delay(1000);
}
