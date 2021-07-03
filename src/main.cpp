#include <Arduino.h>
#include <LTC6804.h>
#include <SPI.h>

int error = 0;
static LTC68041 LTC = LTC68041(D8);

static unsigned long timer;
static unsigned long diff=1000;
static bool flip;


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
        Serial.println("\nSPI ok");
    }
    else
    {
        digitalWrite(D1, LOW);
        Serial.println("\nSPI lost");
    }

    LTC.cfgSetVUV(3.1);
    LTC.cfgSetVOV(3.9);

    //Local Variables
    if((timer+diff)<millis())
    {
        timer=millis();
        flip= !flip;
    }

    if (flip)
    {

        digitalWrite(D2, HIGH);
        LTC.CFGRw[4]=0;
        LTC.CFGRw[5]=0;
        //LTC.DischargeW[0]=1;
        //LTC.DischargeW[8]=1;
    }
    else
    {
        //einfach ein bisschen die discharge pins flippen um zu gucken obs geht
        digitalWrite(D2, LOW);
        LTC.CFGRw[4]=0xFF;
        LTC.CFGRw[5]=0x0F;
        //LTC.DischargeW[0]=0;
        //LTC.DischargeW[8]=0;
    }

    LTC.cfgWrite(LTC.CFGRw);
    //Start different Analog-Digital-Conversions in the Chip
    
    LTC.cmdADCV(LTC68041::DischargeCtrl::DCP_DISABLED);
    LTC.cmdADAX();
    LTC.cmdADSTAT();
    delay(20);   //Wait until everything is finished

    //Read the raw values into the controller
    LTC.readCells();
    LTC.readAUX(0xA);
    LTC.readAUX(0xB);
    LTC.readStatus(0xA);
    LTC.readStatus(0xB);
    LTC.cfgRead();

    //LTC.cnvDischarge(LTC.DischargeW);
    //LTC.rditemp();
    //Print the clear text values cellVoltage, gpioVoltage, Undervoltage Bits, Overvoltage Bits

    Serial.print("\nConfig zurückgelesen: ");
    LTC.readStatusDbg();
    Serial.println("");

    LTC.readCellsDbg();

    Serial.print("\nModule Voltage: ");
    Serial.print(LTC.SumCellVoltages);
    Serial.print("\r\n");

    delay(100);
}
