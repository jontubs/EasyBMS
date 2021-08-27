#include <LTC6804.h>
#include <LTC6804.cpp>

int error = 0;
static LTC68041 LTC = LTC68041(D8);

static unsigned long timer;
static unsigned long diff = 1000;
static bool flip = true;


// the setup function runs once when you press reset or power the board
void setup()
{
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(D5, OUTPUT);
    pinMode(D1, OUTPUT);

    Serial.begin(74880);
    Serial.println("Init");
    LTC.initSPI(D7, D6, D5); //Initialize LTC6804 hardware
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

    //Local Variables
    if((timer + diff) < millis())
    {
        timer = millis();
        //flip = !flip;
    }

    if (flip)
    {

        digitalWrite(D2, HIGH);
        LTC.cfgSetDCC(std::bitset<12>{0b000000000000});  //alternative bitset.reset()
        //LTC.DischargeW[0]=1;
        //LTC.DischargeW[8]=1;
    }
    else
    {
        //einfach ein bisschen die discharge pins flippen um zu gucken obs geht
        digitalWrite(D2, LOW);
        LTC.cfgSetDCC(std::bitset<12>{0b111111111111});  //alternative bitset.set()
        //LTC.DischargeW[0]=0;
        //LTC.DischargeW[8]=0;
    }

    LTC.cfgWrite();
    //Start different Analog-Digital-Conversions in the Chip

    LTC.cmdADCV(LTC68041::DCP_DISABLED);
    delay(5);   //Wait until conversion is finished
    LTC.cmdADAX();
    delay(5);   //Wait until conversion is finished
    LTC.cmdADSTAT();
    delay(5);   //Wait until conversion is finished

    //Read the raw values into the controller
    std::array<float, 6> voltages;
    LTC.getCellVoltages(voltages, LTC68041::CH_ALL);  // read all channel (2nd parameter default), use only 6 (size of array)
    LTC.getAuxVoltage(LTC68041::CHG_ALL);        // nothing is read, just for documenation of usage
    LTC.getStatusVoltage(LTC68041::CHST_ALL);    // nothing is read, just for documenation of usage
    LTC.cfgRead();

    //LTC.cnvDischarge(LTC.DischargeW);
    //LTC.rditemp();
    //Print the clear text values cellVoltage, gpioVoltage, Undervoltage Bits, Overvoltage Bits

    LTC.readCfgDbg();
    LTC.readStatusDbg();
    LTC.readAuxDbg();
    LTC.readCellsDbg();

    delay(500);
}
