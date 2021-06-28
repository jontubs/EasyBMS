#include <Arduino.h>
#include <LTC68041ESP.h>
#include <SPI.h>

int error = 0;

//byte pinMOSI, byte pinMISO, byte pinCLK, byte csPin
static LTC68041 LTC = LTC68041(D7, D6, D5, D8);


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
  //Set some config values
  LTC.CFGRw[0]=0xFE;   //0x07;     //0xFE;
  LTC.CFGRw[5]=0;
  LTC.setVUV(3.1);    //undervoltage detection
  LTC.setVOV(3.9);    //overvoltage detection

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
  //Start different Analog-Digital-Conversions in the Chip
  LTC.adcv();
  LTC.adax();
  LTC.adstat();

  delay(20); //Wait until everything is finished

  //Read the raw values into the controller
  LTC.rdcv();
  LTC.rdauxa();
  LTC.rdauxb();
  LTC.rdstata();
  LTC.rdstatb();
  //Convert the raw values into clear text values
  LTC.cnvCellVolt();
  LTC.cnvAuxVolt();
  LTC.cnvStatus();

  Serial.print("\nIC Temperature: ");
  Serial.print(LTC.InternalTemp);
  Serial.print("\r\n");
  /*
  Serial.print("\nCell Voltages: ");
  printArrayFloat(LTC.cellNum, LTC.cellVoltage);
  Serial.print("\r\n");

  Serial.print("\nGPIO Voltages: ");
  printArrayFloat(LTC.gpioNum, LTC.gpioVoltage);
  Serial.print("\r\n");

  Serial.print("\n Raw Status Values: ");
  printArrayByte(LTC.SizeReg, LTC.STAR);
  Serial.print("\r\n");

  Serial.print("\n Cell Undervoltage detected: ");
  printArrayBool(LTC.cellNum, LTC.CUV);
  Serial.print("\r\n");

  Serial.print("\n Cell Overvoltage detected:  ");
  printArrayBool(LTC.cellNum, LTC.COV);
  Serial.print("\r\n");
  */
  delay(200);
}
