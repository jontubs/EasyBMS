#include <ESP8266WiFi.h>
#include <LTC6804.h>
#include <LTC6804.cpp>
#include <lwip/dns.h>
#include <PubSubClient.h>

#include "credentials.h"

int error = 0;
static LTC68041 LTC = LTC68041(D8);

static const long master_timeout = 5000;

unsigned int module_index = 3;
String hostname = String("esp-module/") + module_index;

WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, espClient);

unsigned long uptime_millis;
// std::array<float, 12> cell_voltage;

// connect to wifi
void connectWifi()
{
    Serial.println();
    Serial.print("connecting to ");
    Serial.println(ssid);
    WiFi.persistent(false);
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.hostname(hostname);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.print("DNS1: ");
    Serial.println(IPAddress(dns_getserver(0)));
    Serial.print("DNS2: ");
    Serial.println(IPAddress(dns_getserver(1)));
}

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect(WiFi.macAddress().c_str(), mqtt_username, mqtt_password, (hostname + "/available").c_str(), 0, true, "offline"))
        {
            Serial.println("connected");
            // Once connected, publish an announcement...
            client.publish((hostname + "/available").c_str(), "online", true);
            // ... and resubscribe
            client.subscribe((hostname + "/cell_discharge").c_str());
            client.subscribe(String("master/uptime").c_str());
            client.subscribe((hostname + "/cell_overvoltage_limit").c_str());
            client.subscribe((hostname + "/cell_undervoltage_limit").c_str());
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            //   Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            // delay(5000);
            return;
        }
    }
}

long last_master_uptime = 0;

void callback(char *topic, byte *payload, unsigned int length)
{
    if (!strcmp(topic, String("master/uptime").c_str()))
    {
        String uptime = "";
        for (unsigned int i = 0; i < length; i++)
        {
            uptime += static_cast<char>(payload[i]);
        }
        Serial.print("Got heartbeat from master: ");
        Serial.println(uptime);

        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
        digitalWrite(LED_BUILTIN, HIGH);

        long uptime_long = uptime.toInt();

        if (uptime_long - last_master_uptime > master_timeout)
        {
            Serial.println(">>> Master Timeout!!");
        }
    }
    else if (!strcmp(topic, (hostname + "/cell_discharge").c_str()))
    {
        String cell_discharge = "";
        for (unsigned int i = 0; i < length; i++)
        {
            cell_discharge += static_cast<char>(payload[i]);
        }
        Serial.println(cell_discharge);
        // bool cell_discharge = payload[0];
        // Serial.println(cell_discharge);
    }
    else if (!strcmp(topic, (hostname + "/alive_counter_master").c_str()))
    {
        String alive_counter_master = "";
        for (unsigned int i = 0; i < length; i++)
        {
            alive_counter_master += static_cast<char>(payload[i]);
        }
        Serial.println(alive_counter_master);
    }
    else if (!strcmp(topic, (hostname + "/cell_overvoltage_limit").c_str()))
    {
        String alive_counter_master = "";
        for (unsigned int i = 0; i < length; i++)
        {
            alive_counter_master += static_cast<char>(payload[i]);
        }
        Serial.println(alive_counter_master);
    }
    else if (!strcmp(topic, (hostname + "/cell_undervoltage_limit").c_str()))
    {
        String alive_counter_master = "";
        for (unsigned int i = 0; i < length; i++)
        {
            alive_counter_master += static_cast<char>(payload[i]);
        }
        Serial.println(alive_counter_master);
    }
}

// the setup function runs once when you press reset or power the board
void setup()
{
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(D5, OUTPUT);
    pinMode(D1, OUTPUT);

    Serial.begin(74880);
    Serial.println("Init");

    connectWifi();
    randomSeed(micros());

    LTC.initSPI(D7, D6, D5); //Initialize LTC6804 hardware

    // for (int i = 0; i < 12; i++)
    // {
    //     cell_voltage.at(i) = 2.8 + (0.1 * i);
    // }
    client.setCallback(callback);
}

// the loop function runs over and over again forever
void loop()
{
    if (LTC.checkSPI(true))
    {
        digitalWrite(D1, HIGH);
        Serial.println();
        Serial.println("SPI ok");
    }
    else
    {
        digitalWrite(D1, LOW);
        Serial.println();
        Serial.println("SPI lost");
    }

    LTC.cfgSetVUV(3.1);
    LTC.cfgSetVOV(4.2);

    // digitalWrite(D2, HIGH);
    // LTC.cfgSetDCC(std::bitset<12>{0b000000000000});  //alternative bitset.reset()

    // //einfach ein bisschen die discharge pins flippen um zu gucken obs geht
    // digitalWrite(D2, LOW);
    // LTC.cfgSetDCC(std::bitset<12>{0b111111111111});  //alternative bitset.set()

    LTC.cfgWrite();
    //Start different Analog-Digital-Conversions in the Chip

    LTC.startCellConv(LTC68041::DCP_DISABLED);
    delay(5); //Wait until conversion is finished
    LTC.startAuxConv();
    delay(5); //Wait until conversion is finished
    LTC.startStatusConv();
    delay(5); //Wait until conversion is finished

    //Read the raw values into the controller
    std::array<float, 6> voltages;
    LTC.getCellVoltages(voltages, LTC68041::CH_ALL); // read all channel (2nd parameter default), use only 6 (size of array)
    LTC.getAuxVoltage(LTC68041::CHG_ALL);            // nothing is read, just for documenation of usage
    LTC.getStatusVoltage(LTC68041::CHST_ALL);        // nothing is read, just for documenation of usage
    LTC.cfgRead();

    //Print the clear text values cellVoltage, gpioVoltage, Undervoltage Bits, Overvoltage Bits
    LTC.readCfgDbg();
    LTC.readStatusDbg();
    LTC.readAuxDbg();
    LTC.readCellsDbg();

    std::array<float, 12> cell_voltages;
    LTC.getCellVoltages(cell_voltages);
    float chip_temp = LTC.getStatusVoltage(LTC.StatusGroup::CHST_ITMP);

    float module_voltage = LTC.getStatusVoltage(LTC68041::CHST_SOC);

    delay(500);
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    uptime_millis = millis();
    client.publish((hostname + "/uptime").c_str(), String(uptime_millis).c_str(), true);

    for (size_t i = 0; i < cell_voltages.size(); i++)
    {
        client.publish((hostname + "/cell/" + String(i + 1) + "/voltage").c_str(), String(cell_voltages[i]).c_str(), true);
    }
    client.publish((hostname + "/module_voltage").c_str(), String(module_voltage).c_str(), true);
    client.publish((hostname + "/module_temps").c_str(), (String(25.0) + "," + String(25.1)).c_str(), true);
    client.publish((hostname + "/chip_temp").c_str(), String(chip_temp).c_str(), true);

    delay(1000);
}
