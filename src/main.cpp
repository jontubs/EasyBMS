#include <Arduino.h>
#include <LTC6804.h>
#include <SPI.h>

int error = 0;

//byte pinMOSI, byte pinMISO, byte pinCLK, byte pinCS
static LTC68041 LTC = LTC68041(D7, D6, D5, D8);

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
    LTC.initialize(); //Initialize LTC6804 hardware
}

// the loop function runs over and over again forever
void loop()
{
    if (LTC.checkSPI_mute())
    {
        digitalWrite(D1, HIGH);
        Serial.println("\nSPI ok");
    }
    else
    {
        digitalWrite(D1, LOW);
        Serial.println("\nSPI lost");
    }
    //Set the CFGWrite bytes
    LTC.CFGRw[0]=0xFE;   //0x07;     //0xFE;

    LTC.CFGRw[5]=0;
    LTC.setVUV(3.1);
    LTC.setVOV(3.9);

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

    LTC.wrcfg(LTC.CFGRw);
    //Start different Analog-Digital-Conversions in the Chip
    
    LTC.adcv();
    LTC.adax();
    LTC.adstat();
    delay(20);   //Wait until everything is finished

    //Read the raw values into the controller
    LTC.rdcv();
    LTC.rdauxa();
    LTC.rdauxb();
    LTC.rdstata();
    LTC.rdstatb();
    LTC.rdcfg();

    //Convert the raw values into clear text values
    LTC.cnvCellVolt();
    LTC.cnvAuxVolt();
    LTC.cnvStatus();
    //LTC.cnvDischarge(LTC.DischargeW);
    //LTC.rditemp();
    //Print the clear text values cellVoltage, gpioVoltage, Undervoltage Bits, Overvoltage Bits

    Serial.print("\nConfig zurückgelesen: ");
    printArrayByte(6, LTC.CFGRr);
    Serial.print("\r\n");

    Serial.print("\nCell Voltages: ");
    for(int i=0;i<LTC.cellNum;i++)
    {
        Serial.print(LTC.cellVoltage[i]);
        Serial.print("\t");
    }

    Serial.print("\nModule Voltage: ");
    Serial.print(LTC.SumCellVoltages);
    Serial.print("\r\n");

    delay(100);
}
