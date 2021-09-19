#include <ESP8266WiFi.h>
//#include <LTC6804.h>
//#include <LTC6804.cpp>
#include <lwip/dns.h>
#include <PubSubClient.h>

#include "credentials.h"

int error = 0;
//static LTC68041 LTC = LTC68041(D8);

static unsigned long timer;
static unsigned long diff = 1000;
static bool flip;

static const long master_timeout = 5000;

unsigned int module_index = 1;
String hostname = String("esp-module") + module_index;

WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, espClient);

unsigned long uptime_millis_slave = millis();
std::array<float, 12> cell_voltage;

// connect to wifi
void connectWifi()
{
    Serial.println();
    Serial.print("connecting to ");
    Serial.println(ssid);
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
        if (client.connect(hostname.c_str(), NULL, NULL, (hostname + "/available").c_str(), 0, true, "offline"))
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

        if(uptime_long - last_master_uptime > master_timeout) {
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

   //LTC.initSPI(D7, D6, D5); //Initialize LTC6804 hardware

    for (int i = 0; i < 12; i++)
    {
        cell_voltage.at(i) = 2.8 + (0.1 * i);
    }
    client.setCallback(callback);
}

// the loop function runs over and over again forever
void loop()
{
    /*
    if (LTC.checkSPI(false)) // true
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

    */
    /*
    LTC.cfgSetVUV(3.1);
    LTC.cfgSetVOV(3.9);

    //Local Variables
    if ((timer + diff) < millis())
    {
        timer = millis();
        flip = !flip;
    }

    if (flip)
    {

        digitalWrite(D2, HIGH);
        LTC.cfgSetDCC(std::bitset<12>{0b000000000000}); //alternative bitset.reset()
        //LTC.DischargeW[0]=1;
        //LTC.DischargeW[8]=1;
    }
    else
    {
        //einfach ein bisschen die discharge pins flippen um zu gucken obs geht
        digitalWrite(D2, LOW);
        LTC.cfgSetDCC(std::bitset<12>{0b111111111111}); //alternative bitset.set()
        //LTC.DischargeW[0]=0;
        //LTC.DischargeW[8]=0;
    }

    LTC.cfgWrite();
    //Start different Analog-Digital-Conversions in the Chip

    LTC.cmdADCV(LTC68041::DCP_DISABLED);
    LTC.cmdADAX();
    LTC.cmdADSTAT();
    delay(20); //Wait until everything is finished

    //Read the raw values into the controller
    std::array<float, 6> tmp;
    LTC.getCellVoltages<6>(tmp, LTC68041::CH_ALL); // read all channel (2nd parameter default), use only 6 (size of array)
    LTC.getAuxVoltage(LTC68041::CHG_ALL);
    LTC.getStatusVoltage(LTC68041::CHST_ALL);
    LTC.cfgRead();

    //LTC.cnvDischarge(LTC.DischargeW);
    //LTC.rditemp();
    //Print the clear text values cellVoltage, gpioVoltage, Undervoltage Bits, Overvoltage Bits

    // LTC.readCfgDbg();
    // LTC.readStatusDbg();
    // LTC.readCellsDbg();

    */
    // Serial.print("Sum of all Cells Voltage: ");
    // Serial.println(LTC.getStatusVoltage(LTC68041::CHST_SOC));

    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    uptime_millis_slave = millis();
    client.publish((hostname + "/uptime_slave").c_str(), String(uptime_millis_slave).c_str(), true);
    String pub_cell_voltage = String("");
    float cell_voltage_sum = 0.0;
    float cell_voltage_min = 5000.0;
    float cell_voltage_max = -5000.0;
    for (const auto &s : cell_voltage)
    {
        pub_cell_voltage += String(s) + ",";
        cell_voltage_sum += s;
        if (s < cell_voltage_min)
            cell_voltage_min = s;
        if (s > cell_voltage_max)
            cell_voltage_max = s;
    }
    client.publish((hostname + "/cell_voltage").c_str(), pub_cell_voltage.c_str(), true);
    client.publish((hostname + "/module_voltage").c_str(), String(cell_voltage_sum).c_str(), true);
    client.publish((hostname + "/cell_voltage_diff").c_str(), String(cell_voltage_max - cell_voltage_min).c_str(), true);

    delay(1000);
}
