/************************************************************

This library is based on the LTC68041.cpp by linear technology.
http://www.linear.com/product/LTC6804-1

I modified it make it compatible with the ESP8622

***********************************************************/


#include <stdint.h>
#include <Arduino.h>
#include "LTC68041ESP.h"
#include <SPI.h>



/*!
  6804 conversion command variables.
*/
uint8_t ADCV[2]; //!< Cell Voltage conversion command.
uint8_t ADAX[2]; //!< GPIO conversion command.
uint8_t dummy = 0x55;
uint8_t  SizeConfigReg = 6; //Len Conifiguration Register = 6
uint8_t  SizeStatusRegA = 6; //Len Conifiguration Register = 6
uint8_t  SizeStatusRegB = 6; //Len Conifiguration Register = 6
uint8_t  PEClen = 2;		//Len PEC Bytes = 2

/*!
  \brief This function will initialize all 6804 variables and the SPI port.

  This function will initialize the Linduino to communicate with the LTC6804 with a 1MHz SPI clock.
  The Function also intializes the ADCV and ADAX commands to convert all cell and GPIO voltages in
  the Normal ADC mode.
*/
void LTC6804_initialize()
{
  //quikeval_SPI_connect();
  //spi_enable(SPI_CLOCK_DIV128); //CHANGED: This will set the ESP to have a 1MHz Clock 
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.begin();
  
  
  set_adc(MD_NORMAL,DCP_DISABLED,CELL_CH_ALL,AUX_CH_ALL);
}

/*!*******************************************************************************************************************
 \brief Maps  global ADC control variables to the appropriate control bytes for each of the different ADC commands

@param[in] uint8_t MD The adc conversion mode
@param[in] uint8_t DCP Controls if Discharge is permitted during cell conversions
@param[in] uint8_t CH Determines which cells are measured during an ADC conversion command
@param[in] uint8_t CHG Determines which GPIO channels are measured during Auxiliary conversion command

Command Code:
-------------

|command  |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCV:      |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
|ADAX:      |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
 ******************************************************************************************************************/
void set_adc(uint8_t MD, //ADC Mode
             uint8_t DCP, //Discharge Permit
             uint8_t CH, //Cell Channels to be measured
             uint8_t CHG //GPIO Channels to be measured
            )
{
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  ADCV[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  ADCV[1] =  md_bits + 0x60 + (DCP<<4) + CH;

  md_bits = (MD & 0x02) >> 1;
  ADAX[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  ADAX[1] = md_bits + 0x60 + CHG ;

}


/*!*********************************************************************************************
  \brief Starts cell voltage conversion

  Starts ADC conversions of the LTC6804 Cpin inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CH     | Determines which cell channels are converted |
 | DCP    | Determines if Discharge is Permitted       |

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCV:      |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
***********************************************************************************************/
void LTC6804_adcv()
{

  uint8_t cmd[4];
  uint16_t cmd_pec;

  //1
  cmd[0] = 0x03;
  cmd[1] = 0x60;

  //2
  cmd_pec = pec15_calc(2, ADCV);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  //4
  digitalWrite(LTC6804_CS, LOW);
  spi_write_array(4,cmd);
  digitalWrite(LTC6804_CS,HIGH);

}



/*!******************************************************************************************************
Starts cell voltage conversion with test values from selftest 1
  1. Load adcv command into cmd array
  2. Calculate adcv cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast adcv command to LTC6804 daisy chain
*********************************************************************************************************/
void LTC6804_adcv_test1()
{

  uint8_t cmd[4];
  uint16_t cmd_pec;

  //1
  cmd[0] = 0x3;
  cmd[1] = 0x27;

  //2
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
  
  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  //4
  digitalWrite(LTC6804_CS, LOW);
  spi_write_array(4,cmd);
  digitalWrite(LTC6804_CS,HIGH);
}




/*******************************************************************************************************
Starts cell voltage conversion with test values from selftest 2
  1. Load adcv command into cmd array
  2. Calculate adcv cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast adcv command to LTC6804 daisy chain
*********************************************************************************************************/
void LTC6804_adcv_test2()
{

  uint8_t cmd[4];
  uint16_t cmd_pec;

  //1
  cmd[0] = 0x3;
  cmd[1] = 0x47;

  //2
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
  
  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  //4
  digitalWrite(LTC6804_CS, LOW);
  spi_write_array(4,cmd);
  digitalWrite(LTC6804_CS,HIGH);
}






/*!******************************************************************************************************
 Test the SPI connection by asking status
 1. Request for config register
 2. Read fully config register
 3. calc PEC from response
 4. extract PEC from response
 5. compare PECs
 6. Send Serial message with result 
*********************************************************************************************************/
void LTC6804_checkSPI()
{
  uint16_t response_pec_calc;
  uint16_t response_pec;			
  uint8_t cmd[8];

  uint8_t response[16];
  cmd[0] = 0x00;
  cmd[1] = 0x02;
  cmd[2] = 0x2b;
  cmd[3] = 0x0A;
  Serial.print("CMD: ");
  Serial.print(cmd[0], HEX);
  Serial.print(" ");
  Serial.print(cmd[1], HEX);
  Serial.print(" ");
  Serial.print(cmd[2], HEX);
  Serial.print(" ");
  Serial.print(cmd[3], HEX);

  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LTC6804_CS, LOW);
  SPI.transfer(cmd[0]);  
  SPI.transfer(cmd[1]);  
  SPI.transfer(cmd[2]);  
  SPI.transfer(cmd[3]);
  for(int i=0; i<(SizeConfigReg+PEClen);i++)
  {
    response[i] =SPI.transfer(0); 
  }
  digitalWrite(LTC6804_CS, HIGH);
  digitalWrite(LED_BUILTIN, LOW);  
  Serial.print("\nRSP: ");
  for(int i=0; i<(SizeConfigReg+PEClen);i++)
  {
    Serial.print(response[i], HEX);
  	Serial.print(" ");
  }
  response_pec_calc = pec15_calc(SizeConfigReg, response);
  response_pec=(response[SizeConfigReg]<<8)+response[SizeConfigReg+1];
  Serial.print("\nPEC Calc: ");
  Serial.print(response_pec_calc, HEX);
  Serial.print("\nPEC Resp: ");
  Serial.print(response_pec, HEX);
  
  
  if(response_pec==response_pec_calc&(response_pec_calc!=0))
  {
  	Serial.print("\nPEC was correct ");
  }
  else
  {
  	Serial.print("\nPEC was NOT correct, check Hardware ");
  }  
  Serial.print("\n\n");
}

bool LTC6804_checkSPI_mute()
{
  uint16_t response_pec_calc;
  uint16_t response_pec;			
  uint8_t cmd[8];

  uint8_t response[16];
  cmd[0] = 0x00;
  cmd[1] = 0x02;
  cmd[2] = 0x2b;
  cmd[3] = 0x0A;


  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  digitalWrite(LTC6804_CS, LOW);
  SPI.transfer(cmd[0]);  
  SPI.transfer(cmd[1]);  
  SPI.transfer(cmd[2]);  
  SPI.transfer(cmd[3]);
  for(int i=0; i<(SizeConfigReg+PEClen);i++)
  {
    response[i] =SPI.transfer(0); 
  }
  digitalWrite(LTC6804_CS, HIGH);


  response_pec_calc = pec15_calc(SizeConfigReg, response);
  response_pec=(response[SizeConfigReg]<<8)+response[SizeConfigReg+1];
 
  
  if(response_pec==response_pec_calc&(response_pec_calc!=0))
  {
  	return 1;
  }
  else
  {
  	return 0;
  }  
}

bool LTC6804_rdstatus_debug()
{
  uint16_t response_pec_calc;
  uint16_t response_pec;			
  uint8_t cmd[8];
  uint8_t STAR[6];		//Status register A
  uint8_t response[16];
  uint16_t cmd_pec;
  uint16_t ITMP;
  float InternalTemp;
  float offset=10;
  
  
  //Start Status group ADC Conversion and Poll Status
  cmd[0] = 0x5;		//ADSTAT (all)
  cmd[1] = 0x68;		//ADSTAT (all)
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
  digitalWrite(LTC6804_CS, LOW);
  SPI.transfer(cmd[0]);  
  SPI.transfer(cmd[1]);  
  SPI.transfer(cmd[2]);  
  SPI.transfer(cmd[3]);

  digitalWrite(LTC6804_CS, HIGH);
// spi_write_array(uint8_t len, uint8_t data[])
	
  delay(10); //wait for conversion


  cmd[0] = 0x00;		//Read Status Register Group A
  cmd[1] = 0x10;		//Read Status Register Group A
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  digitalWrite(LTC6804_CS, LOW);
  SPI.transfer(cmd[0]);  
  SPI.transfer(cmd[1]);  
  SPI.transfer(cmd[2]);  
  SPI.transfer(cmd[3]);
  for(int i=0; i<(SizeStatusRegA+PEClen);i++)
  //for(int i=0; i<15;i++)
  {
    response[i] =SPI.transfer(1); 
  }
  digitalWrite(LTC6804_CS, HIGH);

  
  Serial.print("\nRSP Status A: ");
  for(int i=0; i<(SizeStatusRegA+PEClen);i++)
  {
    Serial.print(response[i], HEX);
  	Serial.print(" ");
  }
  
  //Serial.print("\nSTAR:");
  for(int i=0; i<SizeStatusRegA;i++)
  {
	STAR[i]=response[i];
	
	//Serial.print(STAR[i], HEX);
  }
  //Serial.print("\nSTAR2:");
  //Serial.print(STAR[2], HEX);
  //Serial.print("\nSTAR3:");
  //Serial.print(STAR[3], HEX);
  ITMP=STAR[2] | ((uint16_t)STAR[3])<<8;

  
  Serial.print("\nITMP:");
  Serial.print(ITMP);

  //16-Bit ADC Measurement Value of Internal Die Temperature Temperature Measurement (°C) = ITMP • 100µV/7.5mV/°C – 273°C
  InternalTemp= ((float)ITMP * 100E-6 / 7.5E-3 - 273.0 ) +offset;
  Serial.print("\nInternalTemp:");
  Serial.print(InternalTemp);
  Serial.print("\n\n");
  response_pec_calc = pec15_calc(SizeStatusRegA, response);
  response_pec=(response[SizeStatusRegA]<<8)+response[SizeStatusRegA+1];
 
  
  if(response_pec==response_pec_calc&(response_pec_calc!=0))
  {
  	return 1;
  }
  else
  {
  	return 0;
  }  
}

float  LTC6804_rditemp()
{
  uint16_t response_pec_calc;
  uint16_t response_pec;			
  uint8_t cmd[8];
  uint8_t STAR[6];		//Status register A
  uint8_t response[16];
  uint16_t cmd_pec;
  uint16_t ITMP;
  uint8_t data[8];
  float InternalTemp;
  float offset=10;
  
  
  //Start Status group ADC Conversion and Poll Status
  cmd[0] = 0x5;		//ADSTAT (all)
  cmd[1] = 0x68;		//ADSTAT (all)
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
  
  //spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  
  
  digitalWrite(LTC6804_CS, LOW);
  SPI.transfer(cmd[0]);  
  SPI.transfer(cmd[1]);  
  SPI.transfer(cmd[2]);  
  SPI.transfer(cmd[3]);

  digitalWrite(LTC6804_CS, HIGH);
// spi_write_array(uint8_t len, uint8_t data[])
	
  delay(10); //wait for conversion


  cmd[0] = 0x00;		//Read Status Register Group A
  cmd[1] = 0x10;		//Read Status Register Group A
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  spi_write_read(cmd, 4, response, (SizeStatusRegA+PEClen));         //Read the configuration data of all ICs on the daisy chain into

  
  //Serial.print("\nSTAR:");
  for(int i=0; i<SizeStatusRegA;i++)
  {
	STAR[i]=response[i];
  }

  ITMP=STAR[2] | ((uint16_t)STAR[3])<<8;


  //16-Bit ADC Measurement Value of Internal Die Temperature Temperature Measurement (°C) = ITMP • 100µV/7.5mV/°C – 273°C
  InternalTemp= ((float)ITMP * 100E-6 / 7.5E-3 - 273.0 ) +offset;
  response_pec_calc = pec15_calc(SizeStatusRegA, response);
  response_pec=(response[SizeStatusRegA]<<8)+response[SizeStatusRegA+1];
 
  
  if(response_pec==response_pec_calc&(response_pec_calc!=0))
  {
  	return InternalTemp;
  }
  else
  {
  	return 5555;
  }  
}

float LTC6804_rditemp2()
{
  uint16_t response_pec_calc;
  uint16_t response_pec;			
  uint8_t cmd[4];
  uint8_t STAR[6];		//Status register A
  uint8_t response[16];
  uint16_t cmd_pec;
  uint16_t ITMP;
  uint8_t data[8];		//Puffer für ankommende Daten
  float InternalTemp;
  float offset=10;
  
  
  //Start Status group ADC Conversion and Poll Status
  cmd[0] = 0x5;		//ADSTAT (all)
  cmd[1] = 0x68;		//ADSTAT (all)
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  //Schreibe Befehl auf SPI und erwarte Antwort mit der Länge 0 im Array "data"
  spi_write_read(cmd,4,data,0);
	
  delay(10); //wait for conversion


  cmd[0] = 0x00;		//Read Status Register Group A
  cmd[1] = 0x10;		//Read Status Register Group A
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  spi_write_read(cmd,4,response,16);
  
  //Übertrage der eigentlichen Payload ohne CRC
  for(int i=0; i<SizeStatusRegA;i++)
  {
	STAR[i]=response[i];
  }

  ITMP=STAR[2] | ((uint16_t)STAR[3])<<8;

  InternalTemp= LTC6804_convertITMP(ITMP, offset);
  response_pec_calc = pec15_calc(SizeStatusRegA, response);
  response_pec=(response[SizeStatusRegA]<<8)+response[SizeStatusRegA+1];
   
  
  if(response_pec==response_pec_calc&(response_pec_calc!=0))
  {
  	return InternalTemp;
  }
  else
  {
  	return 5555;
  }  
}

float LTC6804_convertITMP(uint16_t ITMP, float offset)
{
  float InternalTemp;
  //16-Bit ADC Measurement Value of Internal Die Temperature Temperature Measurement (°C) = ITMP • 100µV/7.5mV/°C – 273°C
  InternalTemp= ((float)ITMP * 100E-6 / 7.5E-3 - 273.0 ) + offset;
  return InternalTemp;
}

double LTC6804_convertV(uint16_t v) {
    
    return v * 100E-6;
}

/*!******************************************************************************************************
 \brief Start an GPIO Conversion

  Starts an ADC conversions of the LTC6804 GPIO inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CHG    | Determines which GPIO channels are converted |


Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADAX:      |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
*********************************************************************************************************/
void LTC6804_adax()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = ADAX[0];
  cmd[1] = ADAX[1];
  cmd_pec = pec15_calc(2, ADAX);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  digitalWrite(LTC6804_CS, LOW);
  spi_write_array(4,cmd);
  digitalWrite(LTC6804_CS, HIGH);;

}
/*
  LTC6804_adax Function sequence:

  1. Load adax command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast adax command to LTC6804 daisy chain
*/


/***********************************************//**
 \brief Reads and parses the LTC6804 cell voltage registers.

 The function is used to read the cell codes of the LTC6804.
 This function will send the requested read commands parse the data
 and store the cell voltages in cell_codes variable.

 @param[in] uint8_t reg; This controls which cell voltage register is read back.

          0: Read back all Cell registers

          1: Read back cell group A

          2: Read back cell group B

          3: Read back cell group C

          4: Read back cell group D

 @param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)

 @param[out] uint16_t cell_codes[]; An array of the parsed cell codes from lowest to highest. The cell codes will
  be stored in the cell_codes[] array in the following format:
  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |

  @return int8_t, PEC Status.

    0: No PEC error detected

    -1: PEC error detected, retry read


 *************************************************/
/*uint8_t LTC6804_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
                     uint8_t total_ic, // the number of ICs in the system
                     uint16_t cell_codes[][12] // Array of the parsed cell codes
                    )
{

  const uint8_t NUM_RX_BYT = 8;
  const uint8_t BYT_IN_REG = 6;
  const uint8_t CELL_IN_REG = 3;

  uint8_t *cell_data;
  uint8_t pec_error = 0;
  uint16_t parsed_cell;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t data_counter=0; //data counter
  cell_data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));
  //1.a
  
  if (reg == 0)
  {
    //a.i
    Serial.println("Reg war 0");
    for (uint8_t cell_reg = 1; cell_reg<5; cell_reg++)                    //executes once for each of the LTC6804 cell voltage registers
    {
      data_counter = 0;
      Serial.println("Lese Zelle");
      LTC6804_rdcv_reg(cell_reg, total_ic,cell_data );                //Reads a single Cell voltage register

      for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every LTC6804 in the daisy chain
      {
        // current_ic is used as the IC counter

        //a.ii
        for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
        {
          // loops once for each of the 3 cell voltage codes in the register
	      
          parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);//Each cell code is received as two bytes and is combined to
          // create the parsed cell voltage code

          cell_codes[current_ic][current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
          data_counter = data_counter + 2;                       //Because cell voltage codes are two bytes the data counter
          //must increment by two for each parsed cell code
         Serial.print("\n Cell Code ");
         Serial.print(current_cell);
	 	 Serial.print(": ");
	 	 Serial.print(cell_codes[current_ic][current_cell]);
	 	 Serial.print(" :\n");
        }
        //a.iii

        received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 cell voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);
        if (received_pec != data_pec)
        {
          pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
          //are detected in the serial data
        }
        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs cell voltage data
      }
    }
  }
//1.b
  else
  {
  	Serial.println("Reg war nicht 0");
    //b.i
    LTC6804_rdcv_reg(reg, total_ic,cell_data);
    for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)        // executes for every LTC6804 in the daisy chain
    {
      // current_ic is used as the IC counter
      //b.ii
      for (uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
      {
        // loops once for each of the 3 cell voltage codes in the register

        parsed_cell = cell_data[data_counter] + (cell_data[data_counter+1]<<8); //Each cell code is received as two bytes and is combined to
        // create the parsed cell voltage code

        cell_codes[current_ic][current_cell + ((reg - 1) * CELL_IN_REG)] = 0x0000FFFF & parsed_cell;
        data_counter= data_counter + 2;                       //Because cell voltage codes are two bytes the data counter
        //must increment by two for each parsed cell code
      }
      //b.iii
      received_pec = (cell_data[data_counter] << 8 )+ cell_data[data_counter + 1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
      //after the 6 cell voltage data bytes
      data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);

      if (received_pec != data_pec)
      {
        pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
        //are detected in the serial data
      }
      data_counter= data_counter + 2;                       //Because the transmitted PEC code is 2 bytes long the data_counter
      //must be incremented by 2 bytes to point to the next ICs cell voltage data
    }
  }

//2
  free(cell_data);
  return(pec_error);
}
*/

/*
  LTC6804_rdcv Sequence

  1. Switch Statement:
    a. Reg = 0
      i. Read cell voltage registers A-D for every IC in the daisy chain
      ii. Parse raw cell voltage data in cell_codes array
      iii. Check the PEC of the data read back vs the calculated PEC for each read register command
    b. Reg != 0

*/






/*!******************************************************************************************************
Reads and parses the LTC6804 cell voltage registers.

 The function is used to read the cell codes of the LTC6804.
 This function will send the requested read commands parse the data
 and store the cell voltages in cell_codes variable.
 
  1. Read every single cell voltage register
  2. Parse raw cell voltage data in cell_codes array
  3. Check the PEC of the data read back vs the calculated PEC for each read register command
  4. Return pec_error flag
  
*********************************************************************************************************/
uint8_t LTC6804_rdcv_debug(uint16_t cell_codes[CellNum]) // Array of the parsed cell codes                  
{

  const uint8_t NUM_RX_BYT = 8;
  const uint8_t BYT_IN_REG = 6;
  const uint8_t CELL_IN_REG = 3;

  uint8_t *cell_pointer;
  uint8_t pec_error = 0;
  uint16_t parsed_cell;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t data_counter=0; //data counter
  uint8_t cell_data[100];
  //1.
  //Lies alle
  //läuft von 1-4
    for (uint8_t cell_reg = 1; cell_reg<5; cell_reg++)                    //executes once for each of the LTC6804 cell voltage registers
    {
      data_counter = 0;
      LTC6804_rdcv_reg(cell_reg, cell_data );                //Reads a single Cell voltage register

		//2.
        for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
        {
          // loops once for each of the 3 cell voltage codes in the register

          parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);//Each cell code is received as two bytes and is combined to
          // create the parsed cell voltage code

          cell_codes[current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
          data_counter = data_counter + 2;                       //Because cell voltage codes are two bytes the data counter
          //must increment by two for each parsed cell code
        }
		//3.
        received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 cell voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &cell_data[NUM_RX_BYT]);
        if (received_pec != data_pec)
        {
          pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
          //are detected in the serial data
        }
        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs cell voltage data
    }
  
    
	float CellVoltage[12];
	Serial.print("\nCellCodes: ");
	for (int i = 0 ; i < 12; i++)
	{
		CellVoltage[i]= LTC6804_convertV(cell_codes[i]);
		Serial.print(cell_codes[i], HEX);
		Serial.print("\t");
	}
	Serial.print("\nCell Voltages: ");
		for (int i = 0 ; i < 12; i++)
	{
		Serial.print(CellVoltage[i]);
		Serial.print("\t");
	}
	Serial.println();

//4

  return(pec_error);
}






/*!******************************************************************************************************
Reads and parses the LTC6804 cell voltage registers.

 The function is used to read the cell codes of the LTC6804.
 This function will send the requested read commands parse the data
 and store the cell voltages in cell_codes variable.
 
  1. Read every single cell voltage register
  2. Parse raw cell voltage data in cell_codes array
  3. Check the PEC of the data read back vs the calculated PEC for each read register command
  4. Return pec_error flag
  
*********************************************************************************************************/
uint8_t LTC6804_rdcv(uint16_t cell_codes[CellNum]) // Array of the parsed cell codes                  
{

  const uint8_t NUM_RX_BYT = 8;
  const uint8_t BYT_IN_REG = 6;
  const uint8_t CELL_IN_REG = 3;

  uint8_t *cell_pointer;
  uint8_t pec_error = 0;
  uint16_t parsed_cell;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t data_counter=0; //data counter
  uint8_t cell_data[100];
  //1.
  //Lies alle
  //läuft von 1-4
    for (uint8_t cell_reg = 1; cell_reg<5; cell_reg++)                    //executes once for each of the LTC6804 cell voltage registers
    {
      data_counter = 0;
      LTC6804_rdcv_reg(cell_reg, cell_data );                //Reads a single Cell voltage register

		//2.
        for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
        {
          // loops once for each of the 3 cell voltage codes in the register

          parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);//Each cell code is received as two bytes and is combined to
          // create the parsed cell voltage code

          cell_codes[current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
          data_counter = data_counter + 2;                       //Because cell voltage codes are two bytes the data counter
          //must increment by two for each parsed cell code
        }
		//3.
        received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 cell voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &cell_data[NUM_RX_BYT]);
        if (received_pec != data_pec)
        {
          pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
          //are detected in the serial data
        }
        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs cell voltage data
    }
  

//4

  return(pec_error);
}



/***********************************************//**
 \brief Read the raw data from the LTC6804 cell voltage register

 The function reads a single cell voltage register and stores the read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC6804_rdcv() command.

 @param[in] uint8_t reg; This controls which cell voltage register is read back.

          1: Read back cell group A

          2: Read back cell group B

          3: Read back cell group C

          4: Read back cell group D

 @param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)

 @param[out] uint8_t *data; An array of the unparsed cell codes

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDCVA:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |
|RDCVB:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   0   |
|RDCVC:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   0   |
|RDCVD:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   1   |   0   |

 *************************************************/
void LTC6804_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
                      uint8_t *data //An array of the unparsed cell codes
                     )
{
  const uint8_t REG_LEN = 8; //number of bytes in each ICs register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;

  //1
  if (reg == 1)     //1: RDCVA
  {
    cmd[1] = 0x04;
    cmd[0] = 0x00;
  }
  else if (reg == 2) //2: RDCVB
  {
    cmd[1] = 0x06;
    cmd[0] = 0x00;
  }
  else if (reg == 3) //3: RDCVC
  {
    cmd[1] = 0x08;
    cmd[0] = 0x00;
  }
  else if (reg == 4) //4: RDCVD
  {
    cmd[1] = 0x0A;
    cmd[0] = 0x00;
  }

  //2
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  //4

  spi_write_read(cmd,4,data,REG_LEN);



}
/*
  LTC6804_rdcv_reg Function Process:
  1. Determine Command and initialize command array
  2. Calculate Command PEC
  3. Wake up isoSPI, this step is optional
  4. Send Global Command to LTC6804 daisy chain
*/


/***********************************************************************************//**
 \brief Reads and parses the LTC6804 auxiliary registers.

 The function is used
 to read the  parsed GPIO codes of the LTC6804. This function will send the requested
 read commands parse the data and store the gpio voltages in aux_codes variable

@param[in] uint8_t reg; This controls which GPIO voltage register is read back.

          0: Read back all auxiliary registers

          1: Read back auxiliary group A

          2: Read back auxiliary group B


@param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)


 @param[out] uint16_t aux_codes[][6]; A two dimensional array of the gpio voltage codes. The GPIO codes will
 be stored in the aux_codes[][6] array in the following format:
 |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
 |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
 |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |

@return  int8_t, PEC Status

  0: No PEC error detected

 -1: PEC error detected, retry read
 *************************************************/
int8_t LTC6804_rdaux(uint8_t reg, //Determines which GPIO voltage register is read back.
                     uint8_t total_ic,//the number of ICs in the system
                     uint16_t aux_codes[][6]//A two dimensional array of the gpio voltage codes.
                    )
{


  const uint8_t NUM_RX_BYT = 8;
  const uint8_t BYT_IN_REG = 6;
  const uint8_t GPIO_IN_REG = 3;

  uint8_t *data;
  uint8_t data_counter = 0;
  int8_t pec_error = 0;
  uint16_t parsed_aux;
  uint16_t received_pec;
  uint16_t data_pec;
  data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));
  //1.a
  if (reg == 0)
  {
    //a.i
    for (uint8_t gpio_reg = 1; gpio_reg<3; gpio_reg++)                //executes once for each of the LTC6804 aux voltage registers
    {
      data_counter = 0;
      LTC6804_rdaux_reg(gpio_reg, total_ic,data);                 //Reads the raw auxiliary register data into the data[] array

      for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every LTC6804 in the daisy chain
      {
        // current_ic is used as the IC counter

        //a.ii
        for (uint8_t current_gpio = 0; current_gpio< GPIO_IN_REG; current_gpio++) // This loop parses the read back data into GPIO voltages, it
        {
          // loops once for each of the 3 gpio voltage codes in the register

          parsed_aux = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
          // create the parsed gpio voltage code

          aux_codes[current_ic][current_gpio +((gpio_reg-1)*GPIO_IN_REG)] = parsed_aux;
          data_counter=data_counter+2;                        //Because gpio voltage codes are two bytes the data counter
          //must increment by two for each parsed gpio voltage code

        }
        //a.iii
        received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 gpio voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);
        if (received_pec != data_pec)
        {
          pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
          //are detected in the received serial data
        }

        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs gpio voltage data
      }


    }

  }
  else
  {
    //b.i
    LTC6804_rdaux_reg(reg, total_ic, data);
    for (int current_ic = 0 ; current_ic < total_ic; current_ic++)            // executes for every LTC6804 in the daisy chain
    {
      // current_ic is used as an IC counter

      //b.ii
      for (int current_gpio = 0; current_gpio<GPIO_IN_REG; current_gpio++)    // This loop parses the read back data. Loops
      {
        // once for each aux voltage in the register

        parsed_aux = (data[data_counter] + (data[data_counter+1]<<8));        //Each gpio codes is received as two bytes and is combined to
        // create the parsed gpio voltage code
        aux_codes[current_ic][current_gpio +((reg-1)*GPIO_IN_REG)] = parsed_aux;
        data_counter=data_counter+2;                      //Because gpio voltage codes are two bytes the data counter
        //must increment by two for each parsed gpio voltage code
      }
      //b.iii
      received_pec = (data[data_counter]<<8) + data[data_counter+1];         //The received PEC for the current_ic is transmitted as the 7th and 8th
      //after the 6 gpio voltage data bytes
      data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);
      if (received_pec != data_pec)
      {
        pec_error = -1;                               //The pec_error variable is simply set negative if any PEC errors
        //are detected in the received serial data
      }

      data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
      //must be incremented by 2 bytes to point to the next ICs gpio voltage data
    }
  }
  for (int i = 0; i < NUM_RX_BYT; i++)
  {
  	Serial.print(data[NUM_RX_BYT]);
  	Serial.print("  ");
   }
  Serial.println("  "); 
  free(data);
  return (pec_error);
}
/*
  LTC6804_rdaux Sequence

  1. Switch Statement:
    a. Reg = 0
      i. Read GPIO voltage registers A-D for every IC in the daisy chain
      ii. Parse raw GPIO voltage data in cell_codes array
      iii. Check the PEC of the data read back vs the calculated PEC for each read register command
    b. Reg != 0
      i.Read single GPIO voltage register for all ICs in daisy chain
      ii. Parse raw GPIO voltage data in cell_codes array
      iii. Check the PEC of the data read back vs the calculated PEC for each read register command
  2. Return pec_error flag
*/


/***********************************************//**
 \brief Read the raw data from the LTC6804 auxiliary register

 The function reads a single GPIO voltage register and stores thre read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC6804_rdaux() command.

 @param[in] uint8_t reg; This controls which GPIO voltage register is read back.

          1: Read back auxiliary group A

          2: Read back auxiliary group B


@param[in] uint8_t total_ic; This is the number of ICs in the daisy chain

@param[out] uint8_t *data; An array of the unparsed aux codes



Command Code:
-------------

|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDAUXA:      |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   0   |   0   |
|RDAUXB:      |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |

 *************************************************/
void LTC6804_rdaux_reg(uint8_t reg, //Determines which GPIO voltage register is read back
                       uint8_t total_ic, //The number of ICs in the system
                       uint8_t *data //Array of the unparsed auxiliary codes
                      )
{
  const uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;

  //1
  if (reg == 1)     //Read back auxiliary group A
  {
    cmd[1] = 0x0C;
    cmd[0] = 0x00;
  }
  else if (reg == 2)  //Read back auxiliary group B
  {
    cmd[1] = 0x0e;
    cmd[0] = 0x00;
  }
  else          //Read back auxiliary group A
  {
    cmd[1] = 0x0C;
    cmd[0] = 0x00;
  }
  //2
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.
  //4
  digitalWrite(LTC6804_CS, LOW);
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  digitalWrite(LTC6804_CS, HIGH);;

}
/*
  LTC6804_rdaux_reg Function Process:
  1. Determine Command and initialize command array
  2. Calculate Command PEC
  3. Wake up isoSPI, this step is optional
  4. Send Global Command to LTC6804 daisy chain
*/

/********************************************************//**
 \brief Clears the LTC6804 cell voltage registers

 The command clears the cell voltage registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.


Command Code:
-------------

|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|CLRCELL:     |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |   0   |   0   |   1   |   0   |   0   |   0   |   1   |
************************************************************/
void LTC6804_clrcell()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  //1
  cmd[0] = 0x07;
  cmd[1] = 0x11;

  //2
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec );

  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  //4
  digitalWrite(LTC6804_CS, LOW);
  spi_write_read(cmd,4,0,0);
  digitalWrite(LTC6804_CS, HIGH);;
}
/*
  LTC6804_clrcell Function sequence:

  1. Load clrcell command into cmd array
  2. Calculate clrcell cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast clrcell command to LTC6804 daisy chain
*/


/***********************************************************//**
 \brief Clears the LTC6804 Auxiliary registers

 The command clears the Auxiliary registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.


Command Code:
-------------

|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|CLRAUX:      |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |   0   |   0   |   1   |   0   |   0   |   2   |   0   |
***************************************************************/
void LTC6804_clraux()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  //1
  cmd[0] = 0x07;
  cmd[1] = 0x12;

  //2
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
  //4
  digitalWrite(LTC6804_CS, LOW);
  spi_write_read(cmd,4,0,0);
  digitalWrite(LTC6804_CS, HIGH);;
}
/*
  LTC6804_clraux Function sequence:

  1. Load clraux command into cmd array
  2. Calculate clraux cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast clraux command to LTC6804 daisy chain
*/











/*!***********************************
  \brief 
  uses CFGR4 and lowest 4 bits of CGFR5
 **************************************/
 
/*******************************************************************************************************
Sets  the configuration array for cell balancing
  1. Reset all Discharge Pins
  2. Calculate adcv cmd PEC and load pec into cmd array
  3. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  4. send broadcast adcv command to LTC6804 daisy chain
*********************************************************************************************************/
//		Discharge this cell (1-12), disable all other, IF -1 then all off
void LTC6804_balance_cfg(int cell, uint8_t cfg[6])
{
  cfg[4] = 0x00; // clears S1-8
  cfg[5] = 0x00; // clears S9-12 and keep value of software timer cfg[5]  & 0xF0
  //Serial.println(tx_cfg[ic][5] & 0xF0,BIN);
  if (cell >= 0 and cell <= 7) {
    cfg[4] = 0x1;//cfg[4] | 1 << cell;
  }
  if ( cell > 7) {
    cfg[5] = 0x2;//cfg[5] | ( 1 << (cell - 8));
  }
    Serial.print("\n erzeugte Config");
    for (int i = 0 ; i < 8; i++)
    {
      Serial.print(cfg[i], HEX);
      Serial.print("\t");
    }
    Serial.print("\n"); 
}



















/*****************************************************//**
 \brief Write the LTC6804 configuration register

 This command will write the configuration registers of the LTC6804-1s
 connected in a daisy chain stack. The configuration is written in descending
 order so the last device's configuration is written first.

 @param[in] uint8_t total_ic; The number of ICs being written to.

 @param[in] uint8_t config[][6] is a two dimensional array of the configuration data that will be written, the array should contain the 6 bytes for each
 IC in the daisy chain. The lowest IC in the daisy chain should be the first 6 byte block in the array. The array should
 have the following format:
 |  config[0][0]| config[0][1] |  config[0][2]|  config[0][3]|  config[0][4]|  config[0][5]| config[1][0] |  config[1][1]|  config[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

 The function will calculate the needed PEC codes for the write data
 and then transmit data to the ICs on a daisy chain.


Command Code:
-------------
|               |             CMD[0]                              |                            CMD[1]                             |
|---------------|---------------------------------------------------------------|---------------------------------------------------------------|
|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|WRCFG:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |
********************************************************/
void LTC6804_wrcfg(uint8_t config[6])//A two dimensional array of the configuration data that will be written
{
  const uint8_t BYTES_IN_REG = 6;
  const uint8_t CMD_LEN = 4+(8);
  uint8_t *cmd;
  uint16_t cfg_pec;
  uint8_t cmd_index; //command counter

  cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));

  //1
  cmd[0] = 0x00;
  cmd[1] = 0x01;
  cmd[2] = 0x3d;
  cmd[3] = 0x6e;

  //2
  cmd_index = 4;
    // the last IC on the stack. The first configuration written is
    // received by the last IC in the daisy chain

    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
    {
      // current_byte is the byte counter

      cmd[cmd_index] = config[current_byte];            //adding the config data to the array to be sent
      cmd_index = cmd_index + 1;
    }
    //3
    cfg_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &config[0]);   // calculating the PEC for each ICs configuration register data
    cmd[cmd_index] = (uint8_t)(cfg_pec >> 8);
    cmd[cmd_index + 1] = (uint8_t)cfg_pec;
    cmd_index = cmd_index + 2;


  //4
  wakeup_idle ();                                 //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
  //5
  digitalWrite(LTC6804_CS, LOW);
  spi_write_array(CMD_LEN, cmd);
  digitalWrite(LTC6804_CS, HIGH);;
  free(cmd);
}
/*
  WRCFG Sequence:

  1. Load cmd array with the write configuration command and PEC
  2. Load the cmd with LTC6804 configuration data
  3. Calculate the pec for the LTC6804 configuration data being transmitted
  4. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  5. Write configuration data to the LTC6804 daisy chain

*/

/*!******************************************************
 \brief Reads configuration registers of a LTC6804 daisy chain

@param[in] uint8_t total_ic: number of ICs in the daisy chain

@param[out] uint8_t r_config[][8] is a two dimensional array that the function stores the read configuration data. The configuration data for each IC
is stored in blocks of 8 bytes with the configuration data of the lowest IC on the stack in the first 8 bytes
block of the array, the second IC in the second 8 byte etc. Below is an table illustrating the array organization:

|r_config[0][0]|r_config[0][1]|r_config[0][2]|r_config[0][3]|r_config[0][4]|r_config[0][5]|r_config[0][6]  |r_config[0][7] |r_config[1][0]|r_config[1][1]|  .....    |
|--------------|--------------|--------------|--------------|--------------|--------------|----------------|---------------|--------------|--------------|-----------|
|IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC1 PEC High    |IC1 PEC Low    |IC2 CFGR0     |IC2 CFGR1     |  .....    |


@return int8_t, PEC Status.

  0: Data read back has matching PEC

  -1: Data read back has incorrect PEC


Command Code:
-------------

|CMD[0:1]   |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDCFG:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   1   |   0   |
********************************************************/
int8_t LTC6804_rdcfg(uint8_t r_config[8]) //A two dimensional array that the function stores the read configuration data.
                    
{
	const uint8_t BYTES_IN_REG = 8;
	
	uint8_t cmd[4];
	uint8_t *rx_data_pointer;
	int8_t pec_error = 0;
	uint16_t data_pec;
	uint16_t received_pec;
	
	uint8_t rx_data[8];
	
	//1
	cmd[0] = 0x00;
	cmd[1] = 0x02;
	cmd[2] = 0x2b;
	cmd[3] = 0x0A;
	
	//2
	wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
	//3
	digitalWrite(LTC6804_CS, LOW);
	spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG));         //Read the configuration data of all ICs on the daisy chain into
	digitalWrite(LTC6804_CS, HIGH);;                          //rx_data[] array
	
	Serial.print("gelesene Config: ");
	for (int i = 0 ; i < 8; i++)
	{
		Serial.print(rx_data[i], HEX);
		Serial.print("\t");
	}
	Serial.print("\n");
		
	//into the r_config array as well as check the received Config data
	//for any bit errors
	//4.a
	for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
	{
	  r_config[current_byte] = rx_data[current_byte + (BYTES_IN_REG)];
	}
	//4.b
	received_pec = (r_config[6]<<8) + r_config[7];
	data_pec = pec15_calc(6, &r_config[0]);
	if (received_pec != data_pec)
	{
	  pec_error = -1;
	}
	

	
	//5
	return(pec_error);
}
/*
  RDCFG Sequence:

  1. Load cmd array with the write configuration command and PEC
  2. wakeup isoSPI port, this step can be removed if isoSPI status is previously guaranteed
  3. Send command and read back configuration data
  4. For each LTC6804 in the daisy chain
    a. load configuration data into r_config array
    b. calculate PEC of received data and compare against calculated PEC
  5. Return PEC Error

*/

/*!****************************************************
  \brief Wake isoSPI up from idle state
 Generic wakeup commannd to wake isoSPI up out of idle
 *****************************************************/
void wakeup_idle()
{
  digitalWrite(LTC6804_CS, LOW);
  delay(2); //Guarantees the isoSPI will be in ready mode
  //SPI.transfer(dummy);  //Test
  digitalWrite(LTC6804_CS, HIGH);
  delay(2); //Guarantees the isoSPI will be in ready mode
}

/*!****************************************************
  \brief Wake the LTC6804 from the sleep state

 Generic wakeup commannd to wake the LTC6804 from sleep
 *****************************************************/
void wakeup_sleep()
{
  digitalWrite(LTC6804_CS, LOW);
  delay(1); // Guarantees the LTC6804 will be in standby
  //SPI.transfer(dummy);   //Test
  digitalWrite(LTC6804_CS, HIGH);;
}
/*!**********************************************************
 \brief calaculates  and returns the CRC15

  @param[in] uint8_t len: the length of the data array being passed to the function

  @param[in] uint8_t data[] : the array of data that the PEC will be generated from


  @returns The calculated pec15 as an unsigned int
***********************************************************/
uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate  a PEC
                   )
{
  uint16_t remainder,addr;

  remainder = 16;//initialize the PEC
  for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
  {
    addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
    remainder = (remainder<<8)^crc15Table[addr];
  }
  return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}


/*!
 \brief Writes an array of bytes out of the SPI port

 @param[in] uint8_t len length of the data array being written on the SPI port
 @param[in] uint8_t data[] the data array to be written on the SPI port

*/
void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    )
{
  for (uint8_t i = 0; i < len; i++)
  {
    SPI.transfer((int8_t)data[i]);
  }
}

/*!
 \brief Writes and read a set number of bytes using the SPI port.

@param[in] uint8_t tx_data[] array of data to be written on the SPI port
@param[in] uint8_t tx_len length of the tx_data array
@param[out] uint8_t rx_data array that read data will be written too.
@param[in] uint8_t rx_len number of bytes to be read from the SPI port.

*/

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
  digitalWrite(LTC6804_CS, LOW);
  for (uint8_t i = 0; i < tx_len; i++)
  {
    SPI.transfer(tx_Data[i]);
  }
  for (uint8_t i = 0; i < rx_len; i++)
  {
    rx_data[i]=SPI.transfer(1);
  }
  digitalWrite(LTC6804_CS, HIGH);
}

bool LTC6804_SetVUVVOV(float Undervoltage, float Overvoltage, uint8_t cfg[8])
{
  //float Undervoltage=3.123;
  //float Overvoltage=3.923;
  uint16_t VUV=0;
  uint16_t VOV=0;

  VUV=(Undervoltage/(0.0001*16))-1;
  VOV=(Overvoltage/(0.0001*16));

  //Serial.print("\nVUV:");
  //Serial.print(VUV, HEX);
  //Serial.print("\nVOV:");
  //Serial.print(VOV, HEX);
  //Serial.print("\n"); 


  
  cfg[0] = 0xFE;
  cfg[1] = (uint8_t) VUV;        //0x4E1 ; // 2.0V
  cfg[2] = (VUV >> 8) & 0x0F ;
  cfg[2] = cfg[2] | ((VOV&0x0F) << 4) ;
  cfg[3] = (uint8_t)(VOV>>4) ;
  //cfg[4] = 0x00 ; // discharge switches  0->off  1-> on.  S0 = 0x01, S1 = 0x02, S2 = 0x04, 0x08, 0x10, 0x20, 0x40, 0x80
  //cfg[5] = 0x00 ;
 
}

/***              Public Functions              ***/

/*! Perform LUT lookup of cell SOC based on cell open circuit voltage.
 *  \param voc Cell open circuit voltage, Volts.
 *  \return Function returns SOC from 0-1.
 *  Note: Function interpolates between LUT points to provide continuous output.
 Funktion ist kaputt , muss ich mal fixen
 */
float cell_compute_soc(float voc) {
    
    /* SOC based on OCV lookup table.
     * SOC as a function of open cell voltage is non=linear, a lookup table seems to
     * be the best way to map between these two values.  
     */
    /*const double t_offset = 3.00205;
    const double t_step = 0.01;
    const double t_gain = 1000;
    u_int16_t tbl[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,17,18,19,21,22,23,25,26,28,30,31,33,35,37,39,41,44,46,49,52,55,58,61,65,69,73,78,84,90,97,105,115,126,141,159,182,210,242,277,314,352,389,427,465,503,541,579,616,654,692,730,768,806,843,878,905,923,935,943,949,954,958,961,964,967,969,971,973,975,977,978,979,981,982,983,984,985,986,987,988,989,990,991,991,992,993,994,994,995,996,996,997,997,998,};*/
    // offset: 2.41498V, step size:0.01V
    const float t_offset = 2.415;
    const float t_step = 0.01;
    const float t_gain = 1000;
    float result;
    uint16_t tbl[] = {0,1,2,3,5,6,7,8,10,11,12,14,15,17,18,20,22,24,25,27,29,32,34,36,39,42,45,48,51,55,59,63,68,73,79,87,95,105,117,133,153,181,216,258,303,349,396,443,490,537,584,632,679,726,773,820,865,902,925,938,947,953,958,962,966,969,971,974,976,978,979,981,983,984,985,986,988,989,990,991,992,993,994,994,995,996,997,997,998,999,999,1000,};


    // number of elements in lut
    if (voc <= t_offset) {
        return 0.0;
    }
    // table size
    unsigned int n = sizeof (tbl) / sizeof (tbl[0]);
    // compute index
    unsigned int index = (unsigned int) ((voc - t_offset) / t_step);
    // compute fractional index
    float f_index = ((voc - t_offset) / t_step) - index;
    // limit to valid table index, -1
    index = (index < n-2) ? index : n-2;
    f_index = (f_index < 1.0) ? f_index : 1.0;  // don't extrapolate beyond LUT limits
    // compute local slope
    float delta = (tbl[index+1] - tbl[index]) / t_gain;
    result=tbl[index] / t_gain + f_index * delta;
    Serial.print("\nresult:");
    Serial.print(result);
    return result;
}

