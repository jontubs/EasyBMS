#include <ESP8266httpUpdate.h>
#include <LTC6804.h>
#include <LTC6804.cpp> // used for template functions
#include <PubSubClient.h>

#include "config.h"
#include "version.h"
#include "display.h"
#include "soc.h"
#include "wifi.h"
#include "TimedHistory.hpp"
#include "Measurements.hpp"

#define DEBUG
#define SSL_ENABLED false

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

constexpr unsigned long MASTER_TIMEOUT = 5000;
constexpr unsigned long LTC_CHECK_INTERVAL = 1000;
constexpr unsigned long BLINK_TIME = 5000;

String hostname;
String mac_topic;
String module_topic;

#if SSL_ENABLED
WiFiClientSecure espClient;
#else
WiFiClient espClient;
#endif

PubSubClient client(mqtt_server, mqtt_port, espClient);

std::array<unsigned long, 12> cells_to_balance_start{};
std::array<unsigned long, 12> cells_to_balance_interval{};

unsigned long last_ltc_check = 0;
unsigned long last_connection = 0;
unsigned long last_blink_time = 0;
unsigned long long last_master_uptime = 0;

unsigned long pec15_error_count = 0;

// Store cell diff history with 1h retention and 1 min granularity
auto cell_diff_history = TimedHistory<float>(1000*60*60, 1000*60);

bool led_builtin_state = false;

template <typename T>
boolean publish(String topic, T value) {
    return client.publish(topic.c_str(), String(value).c_str(), true);
}

template <>
boolean publish(String topic, char* value) {
    return client.publish(topic.c_str(), value, true);
}

template <>
boolean publish(String topic, String value) {
    return client.publish(topic.c_str(), value.c_str(), true);
}

boolean subscribe(String topic) {
    return client.subscribe(topic.c_str());
}

String cpu_description() {
    return
        String(EspClass::getChipId(), HEX) +
        " " +
        EspClass::getCpuFreqMHz() + " MHz";
}

String flash_description() {
    return
        String(EspClass::getFlashChipId(), HEX) +
        ", " +
        (EspClass::getFlashChipSize() / 1024 / 1024) +
        " of " +
        (EspClass::getFlashChipRealSize() / 1024 / 1024) +
        " MiB, Mode: " +
        EspClass::getFlashChipMode() +
        ", Speed: " +
        (EspClass::getFlashChipSpeed() / 1000 / 1000) +
        " MHz, Vendor: " +
        String(EspClass::getFlashChipVendorId(), HEX);
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        DEBUG_PRINT("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect(hostname.c_str(), mqtt_username, mqtt_password, (module_topic + "/available").c_str(), 0,
                           true, "offline")) {
            DEBUG_PRINTLN("connected");
            // Once connected, publish an announcement
            publish(module_topic + "/available", "online");
            if (!module_topic.equals(mac_topic)) {
                publish(mac_topic + "/available", "undefined");
            }
            publish(mac_topic + "/module_topic", module_topic);
            publish(mac_topic + "/version", VERSION);
            publish(mac_topic + "/build_timestamp", BUILD_TIMESTAMP);
            publish(mac_topic + "/wifi", WiFi.SSID());
            publish(mac_topic + "/ip", WiFi.localIP().toString());
            publish(mac_topic + "/esp_sdk", EspClass::getFullVersion());
            publish(mac_topic + "/cpu", cpu_description());
            publish(mac_topic + "/flash", flash_description());
            // Resubscribe
            subscribe("master/uptime");
            for (int i = 0; i < 12; ++i) {
                subscribe(module_topic + "/cell/" + (i + 1) + "/balance_request");
            }
            subscribe(module_topic + "/read_accurate");
            subscribe(mac_topic + "/blink");
            subscribe(mac_topic + "/set_config");
            subscribe(mac_topic + "/restart");
            subscribe(mac_topic + "/ota");
        } else {
            DEBUG_PRINT("failed, rc=");
            DEBUG_PRINT(client.state());
            if (last_connection != 0 && millis() - last_connection >= 30000) {
                EspClass::restart();
            }
            delay(1000);
        }
    }
}

String cell_name_from_id(size_t cell_id) {
    size_t cell_number = cell_id + 1;
    if (battery_type == BatteryType::meb8s) {
        if (cell_number <= 4) {
            // do nothing, cell number is correct
        } else if (cell_number <= 8) {
            // These cells dont exist on 8s batteries
            return "undefined";
        } else {
            cell_number = cell_number - 4;
        }
    }

    return String(cell_number);
}

bool string_is_uint(const String &myString) {
    return std::all_of(myString.begin(), myString.end(), isDigit);
}

int cell_id_from_name(const String &cell_name) {
    // cell number needs to be an unsigned integer
    if (!string_is_uint(cell_name)) {
        return -1;
    }

    int cell_number = cell_name.toInt();
    int highest_cell_number = 12;
    if (battery_type == BatteryType::meb8s) {
        highest_cell_number = 8;
    }
    // cell number needs to be between 1 and 12 or 1 and 8 for 8s modules
    if (cell_number < 1 || cell_number > highest_cell_number) {
        return -1;
    }

    // do the 8s cell mapping
    if (battery_type == BatteryType::meb8s) {
        if (cell_number >= 5) {
            cell_number = cell_number + 4;
        }
    }

    return cell_number - 1;
}

void set_balance_bits(std::bitset<12> &balance_bits) {
    for (size_t i = 0; i < cells_to_balance_start.size(); ++i) {
        if (millis() - cells_to_balance_start.at(i) <= cells_to_balance_interval.at(i)) {
            balance_bits.set(i, true);
        } else if (cells_to_balance_interval.at(i) > 0) {
            cells_to_balance_interval.at(i) = 0;
        }
    }
}

void set_LTC(std::bitset<12> &balance_bits) {
    /*
     * check LTC SPI
     */
#ifdef DEBUG
    if (LTC.checkSPI(true)) {
#else
        if (LTC.checkSPI(false)) {
#endif
        digitalWrite(D1, HIGH);
//        DEBUG_PRINTLN();
//        DEBUG_PRINTLN("SPI ok");
    } else {
        digitalWrite(D1, LOW);
//        DEBUG_PRINTLN();
//        DEBUG_PRINTLN("SPI lost");
    }

    LTC.cfgSetRefOn(true);
    /*
     * set LTC config
     */
    LTC.cfgSetVUV(3.1);
    LTC.cfgSetVOV(4.2);

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
    if (!LTC.cfgRead()) {
        pec15_error_count++;
    }

    //Print the clear text values cellVoltage, gpioVoltage, Undervoltage Bits, Overvoltage Bits
#ifdef DEBUG
    LTC.readCfgDbg();
    LTC.readStatusDbg();
    LTC.readAuxDbg();
    LTC.readCellsDbg();
#endif
}

float raw_voltage_to_real_module_temp(float raw_voltage) {
    return 32.0513f * raw_voltage - 23.0769f;
}

Measurements get_measurements() {
    std::array<float, 12> cell_voltages{};
    if (!LTC.getCellVoltages(cell_voltages)) {
        pec15_error_count++;
    }
    float raw_voltage_module_temp_1 = LTC.getAuxVoltage(LTC68041::AuxChannel::CHG_GPIO1);
    float raw_voltage_module_temp_2 = LTC.getAuxVoltage(LTC68041::AuxChannel::CHG_GPIO2);

    float sum = 0.0f;
    float voltage_min = cell_voltages[0];
    float voltage_max = cell_voltages[0];
    for (size_t i = 0; i < cell_voltages.size(); i++) {
        sum += cell_voltages[i];
        if (cell_voltages[i] < voltage_min) {
            voltage_min = cell_voltages[i];
        }
        if (cell_voltages[i] > voltage_max) {
            voltage_max = cell_voltages[i];
        }
    }
    float voltage_avg = 0;
    if (battery_type == BatteryType::meb8s) {
        voltage_avg = sum / 8.0f;
    } else {
        voltage_avg = sum / 12.0f;
    }

    Measurements m;
    for (size_t i = 0; i < cell_voltages.size(); i++) {
        m.cell_diffs_to_avg[i] = cell_voltages[i] - voltage_avg;
    }
    m.cell_voltages = cell_voltages;
    m.module_voltage = LTC.getStatusVoltage(LTC68041::CHST_SOC);
    m.module_temp_1 = raw_voltage_to_real_module_temp(raw_voltage_module_temp_1);
    m.module_temp_2 = raw_voltage_to_real_module_temp(raw_voltage_module_temp_2);
    m.chip_temp = LTC.getStatusVoltage(LTC.StatusGroup::CHST_ITMP);
    m.avg_cell_voltage = voltage_avg;
    m.min_cell_voltage = voltage_min;
    m.max_cell_voltage = voltage_max;
    m.cell_diff = voltage_max - voltage_min;
    m.soc = voltage_to_soc(voltage_avg);
    m.cell_diff_trend = 0.0f;

    // Calculate cell diff trend
    cell_diff_history.insert(m.cell_diff);
    long current_time = millis();
    auto result = cell_diff_history.oldest_element();
    if (result.has_value()) {
        float history_cell_diff = result.value().value;
        float history_timestamp = result.value().timestamp;

        if (current_time > history_timestamp) {
            // Cell diff change per hour in the last hour
            float change = m.cell_diff - history_cell_diff;
            float time_hours = (float)(current_time - history_timestamp) / (float)(1000*60*60);
            m.cell_diff_trend = change / time_hours;
            DEBUG_PRINT(String("Cell Diff Trend: ") + m.cell_diff_trend + " mV/h");
        }
    }

    return m;
}

void publish_mqtt_values(const std::bitset<12>& balance_bits, const String& topic, const Measurements& m) {
    publish(module_topic + "/uptime", millis());
    publish(module_topic + "/pec15_error_count", pec15_error_count);

    for (size_t i = 0; i < m.cell_voltages.size(); i++) {
        String cell_name = cell_name_from_id(i);
        if (cell_name != "undefined") {
            publish(topic + "/cell/" + cell_name + "/voltage", String(m.cell_voltages[i], 3));
            publish(module_topic + "/cell/" + cell_name + "/is_balancing", balance_bits.test(i) ? "1" : "0");
        }
    }

    publish(topic + "/module_voltage", m.module_voltage);
    publish(topic + "/module_temps", String(m.module_temp_1) + "," + String(m.module_temp_2));
    publish(topic + "/chip_temp", m.chip_temp);
}

void callback(char *topic, byte *payload, unsigned int length) {
    String topic_string = String(topic);
    String payload_string = String();
    payload_string.concat((char *) payload, length);
    if (topic_string == "master/uptime") {
        DEBUG_PRINT("Got heartbeat from master: ");
        DEBUG_PRINTLN(payload_string);

        digitalWrite(LED_BUILTIN, led_builtin_state);
        led_builtin_state = !led_builtin_state;

        unsigned long long uptime_u_long = std::stoull(payload_string.c_str());

        if (uptime_u_long - last_master_uptime > MASTER_TIMEOUT) {
            DEBUG_PRINTLN(uptime_u_long);
            DEBUG_PRINTLN(last_master_uptime);
            DEBUG_PRINTLN(">>> Master Timeout!!");
        }
        last_master_uptime = uptime_u_long;
    } else if (topic_string.startsWith(module_topic + "/cell/")) {
        String cell_name = topic_string.substring((module_topic + "/cell/").length());
        cell_name = cell_name.substring(0, cell_name.indexOf("/"));
        long cell_id = cell_id_from_name(cell_name);
        if (cell_id == -1) {
            return;
        }
        if (topic_string == module_topic + "/cell/" + cell_name + "/balance_request") {
            unsigned long balance_time = std::stoul(payload_string.c_str());
            cells_to_balance_start.at(cell_id) = millis();
            cells_to_balance_interval.at(cell_id) = balance_time;
        }
    } else if (topic_string == module_topic + "/read_accurate") {
        if (payload_string == "1") {
            last_ltc_check = millis();
            std::bitset<12> balance_bits{};
            set_balance_bits(balance_bits);
            WiFi.forceSleepBegin();
            delay(1);
            set_LTC(balance_bits);
            WiFi.forceSleepWake();
            delay(1);
            while (WiFi.status() != WL_CONNECTED) {
                delay(100);
            }
            reconnect();
            Measurements m = get_measurements();
            publish_mqtt_values(balance_bits, module_topic + "/accurate", m);
        }
    } else if (topic_string == mac_topic + "/blink") {
        last_blink_time = millis();
    } else if (topic_string == mac_topic + "/set_config") {
        String module_number_string = payload_string.substring(0, payload_string.indexOf(","));
        String total_voltage_measurer_string = payload_string.substring(payload_string.indexOf(",") + 1,
                                                                        payload_string.lastIndexOf(","));
        String total_current_measurer_string = payload_string.substring(payload_string.lastIndexOf(",") + 1);
        module_topic = String("esp-module/") + module_number_string;
        client.disconnect();
        reconnect();
    } else if (topic_string == mac_topic + "/restart") {
        if (payload_string == "1") {
            EspClass::restart();
        }
    } else if (topic_string == mac_topic + "/ota") {
        publish(mac_topic + "/ota_start", String("ota started [") + payload_string + "] (" + millis() + ")");
        publish(mac_topic + "/ota_url", String("https://") + ota_server + payload_string);
        WiFiClientSecure client_secure;
        client_secure.setTrustAnchors(&cert);
        client_secure.setTimeout(60);
        ESPhttpUpdate.setLedPin(LED_BUILTIN, HIGH);
        switch (ESPhttpUpdate.update(client_secure, String("https://") + ota_server + payload_string)) {
            case HTTP_UPDATE_FAILED: {
                String error_string = String("HTTP_UPDATE_FAILED Error (");
                error_string += ESPhttpUpdate.getLastError();
                error_string += "): ";
                error_string += ESPhttpUpdate.getLastErrorString();
                error_string += "\n";
                DEBUG_PRINTLN(error_string);
                publish(mac_topic + "/ota_ret", error_string);
            }
                break;
            case HTTP_UPDATE_NO_UPDATES:
                DEBUG_PRINTLN("HTTP_UPDATE_NO_UPDATES");
                publish(mac_topic + "/ota_ret", "HTTP_UPDATE_NO_UPDATES");
                break;
            case HTTP_UPDATE_OK:
                DEBUG_PRINTLN("HTTP_UPDATE_OK");
                publish(mac_topic + "/ota_ret", "HTTP_UPDATE_OK");
                break;
        }
    }
}

// the setup function runs once when you press reset or power the board
[[maybe_unused]] void setup() {
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(D5, OUTPUT);
    pinMode(D1, OUTPUT);
    digitalWrite(LED_BUILTIN, false);

    DEBUG_BEGIN(74880);
    DEBUG_PRINTLN("init");

    if (use_mqtt) {
        String mac = mac_string();

        hostname = String("easybms-") + mac;
        mac_topic = String("esp-module/") + mac;
        module_topic = mac_topic;

        connect_wifi(hostname, ssid, password);
        digitalWrite(LED_BUILTIN, true);
    }

    randomSeed(micros());
    LTC.initSPI(D7, D6, D5); //Initialize LTC6804 hardware

#if SSL_ENABLED
    espClient.setTrustAnchors(&mqtt_cert_store);
#endif

    client.setCallback(callback);
    setup_display();

}

bool runTests() {
    LTC.startCellConvTest(LTC68041::ST_SELF_TEST_1);
    delay(5);

    std::array<float, 12> cell_voltages_test{};
    LTC.getCellVoltages(cell_voltages_test);

    for (float pattern: cell_voltages_test) {
        DEBUG_PRINT("Conversion Test results: ");
        DEBUG_PRINT(pattern);
        DEBUG_PRINT(", ");
    }

    DEBUG_PRINTLN();

    LTC.startOpenWireCheck(LTC68041::PUP_PULL_UP, LTC68041::DCP_DISABLED);
    delay(5);
    LTC.startOpenWireCheck(LTC68041::PUP_PULL_UP, LTC68041::DCP_DISABLED);
    delay(5);

    std::array<float, 12> cell_voltages_pup{};
    LTC.getCellVoltages(cell_voltages_pup);

    LTC.startOpenWireCheck(LTC68041::PUP_PULL_DOWN, LTC68041::DCP_DISABLED);
    delay(5);
    LTC.startOpenWireCheck(LTC68041::PUP_PULL_DOWN, LTC68041::DCP_DISABLED);
    delay(5);

    std::array<float, 12> cell_voltages_pdown{};
    LTC.getCellVoltages(cell_voltages_pdown);

    if (cell_voltages_pup[0] < 0.0001) {
        DEBUG_PRINTLN("C0 is open");
        return false;
    }

    if (cell_voltages_pdown[11] < 0.0001) {
        DEBUG_PRINTLN("C12 is open");
        return false;
    }

    for (int i = 1; i < 12; i++) {
        if ((cell_voltages_pup[i] - cell_voltages_pdown[i]) < -0.4) {
            DEBUG_PRINT("C");
            DEBUG_PRINT(i);
            DEBUG_PRINTLN(" is open");
            return false;
        }
    }

    return true;
}

void update_display(const std::bitset<12>& balance_bits, const Measurements& m) {
    DisplayData data;

    data.measurements = m;
    data.balance_bits = balance_bits;
    data.uptime_seconds = millis() / 1000;

    draw_cell_voltages(data);
}

// the loop function runs over and over again forever
void loop() {
    if (use_mqtt) {
        // MQTT reconnect if needed
        if (!client.connected()) {
            reconnect();
        }
        last_connection = millis();
        client.loop();
    }

    if (millis() - last_ltc_check > LTC_CHECK_INTERVAL) {
        last_ltc_check = millis();
        std::bitset<12> balance_bits{};
        set_balance_bits(balance_bits);
        set_LTC(balance_bits);

        Measurements measurements = get_measurements();

        if (use_mqtt) {
            publish_mqtt_values(balance_bits, module_topic, measurements);
        }

        if (use_display) {
            update_display(balance_bits, measurements);
        }
    }
    if (millis() - last_blink_time < BLINK_TIME) {
        if ((millis() - last_blink_time) % 100 < 50) {
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            digitalWrite(LED_BUILTIN, LOW);
        }
    }
}
