/************************************************************

This library is based on the LTC68041.cpp by linear technology.
http://www.linear.com/product/LTC6804-1

I modified it make it compatible with the ESP8622

https://github.com/jontubs/EasyBMS
***********************************************************/


#include <stdint.h>
#include <Arduino.h>
#include "LTC68041ESP.h"
#include <SPI.h>


/*!*******************************************************************************************************
Creating of the object LTC68041
*********************************************************************************************************/
LTC68041::LTC68041(byte pMOSI, byte pMISO, byte pCLK, byte pCS)
  : pinMOSI(pMOSI), pinMISO(pMISO), pinCLK(pCLK), pinCS(pCS)
{
    Serial.print("Objekt angelegt");
    pinMode(pinMOSI, OUTPUT);
    pinMode(pinMISO, INPUT);
    pinMode(pinCLK, OUTPUT);
    pinMode(pinCS, OUTPUT);
}


/*!******************************************************************************************************
Reads and parses the LTC6804 cell voltage registers.
This function will initialize all 6804 variables and the SPI port.
*********************************************************************************************************/
void LTC68041::initialize()
{
    SPI.begin();
    initCFGR();
}


/*!******************************************************************************************************
Simple Serial print to check in object exists. Just for Debug 
*********************************************************************************************************/
void LTC68041::helloworld()
{
    Serial.print("Hello World, used Pins are MOSI:");
    Serial.print(pinMOSI);
    Serial.print("\t MISO:");
    Serial.print(pinMISO);
    Serial.print("\t Clock:");
    Serial.print(pinCLK);
    Serial.print("\t Chipselect:");
    Serial.println(pinCS);
}


/*!******************************************************************************************************
Wake isoSPI up from idle state
Generic wakeup commannd to wake isoSPI up out of idle 
*********************************************************************************************************/
void LTC68041::wakeup_idle()
{
    digitalWrite(pinCS, LOW);
    delay(2); //Guarantees the isoSPI will be in ready mode
    //SPI.transfer(dummy);  //Test
    digitalWrite(pinCS, HIGH);
    delay(2); //Guarantees the isoSPI will be in ready mode
}


/*!******************************************************************************************************
Calculates the CRC sum of some data bytes given by the pointer "data" with the length "len"
*********************************************************************************************************/
uint16_t LTC68041::pec15_calc(uint8_t len, uint8_t *data)
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


/*!******************************************************************************************************
Writes and read a set number of bytes using the SPI port.
Tested and runs fine
[in] uint8_t tx_data[] array of data to be written on the SPI port
[in] uint8_t tx_len length of the tx_data array
[out] uint8_t rx_data array that read data will be written too.
[in] uint8_t rx_len number of bytes to be read from the SPI port.
*********************************************************************************************************/
void LTC68041::spi_write_read(uint8_t tx_Data[], uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len)
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);
    for (uint8_t i = 0; i < tx_len; i++)
    {
        SPI.transfer(tx_Data[i]);
    }
    for (uint8_t i = 0; i < rx_len; i++)
    {
        rx_data[i]=SPI.transfer(1);
    }
    digitalWrite(pinCS, HIGH);
    SPI.endTransaction();

}


/*!******************************************************************************************************
Writes and read a set number of bytes using the SPI port without expecting an answer
uint8_t len, // Option: Number of bytes to be written on the SPI port
uint8_t data[] //Array of bytes to be written on the SPI port
*********************************************************************************************************/
void LTC68041::spi_write_array(uint8_t len, uint8_t data[])
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);
    for (uint8_t i = 0; i < len; i++)
    {
        SPI.transfer((int8_t)data[i]);
    }
    digitalWrite(pinCS, HIGH);
    SPI.endTransaction();
}


/*!******************************************************************************************************
calculates the bitpattern in the config for Undervoltage detection  
the config has to be written to the chip after this!
*********************************************************************************************************/
void LTC68041::initCFGR()
{ 
    CFGRw[0] = 0xFE;
    CFGRw[1] = 0 ;
    CFGRw[2] = 0 ;
    CFGRw[3] = 0 ;
    CFGRw[4] = 0 ;
    CFGRw[5] = 0 ;
}

/*!******************************************************************************************************
calculates the bitpattern in the config for Undervoltage detection  
the config has to be written to the chip after this!
*********************************************************************************************************/
void LTC68041::setVUV(float Undervoltage)
{
    uint16_t VUV=0;

    VUV=(Undervoltage/(0.0001*16))-1;		//calc bitpattern for UV

    //CFGRw[0] = 0xFE;
    CFGRw[1] = (uint8_t) VUV;        //0x4E1 ; // 2.0V
    CFGRw[2] = (VUV >> 8) & 0x0F ;
}

/*!******************************************************************************************************
calculates the bitpattern in the config for Overvoltage detection  
the config has to be written to the chip after this!
*********************************************************************************************************/
void LTC68041::setVOV(float Overvoltage)
{
    //float Undervoltage=3.123;
    //float Overvoltage=3.923;
    uint16_t VOV=0;

    VOV=(Overvoltage/(0.0001*16));		//Calc bitpattern for OV

    //CFGRw[0] = 0xFE;
    CFGRw[2] = CFGRw[2] | ((VOV&0x0F) << 4) ;
    CFGRw[3] = (uint8_t)(VOV>>4) ;
}

/*!*******************************************************************************************************
Reads configuration registers of a LTC6804

[out] uint8_t r_config[8] is an array that the function stores the read configuration data. 
return int8_t, PEC Status.
  0: Data read back has matching PEC
  -1: Data read back has incorrect PEC
  
Giving additional Debug infos via Serial
*********************************************************************************************************/

int8_t LTC68041::rdcfg_debug(uint8_t r_config[8]) //An array that the function stores the read configuration data.                 
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
    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    //3
    spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG));         //Read the configuration data of all ICs on the daisy chain into

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


/*!******************************************************************************************************
Sets  the configuration array for cell balancing
  1. Reset all Discharge Pins
  2. Calculate adcv cmd PEC and load pec into cmd array
  Discharge this cell (1-12), disable all other, IF -1 then all off
*********************************************************************************************************/
void LTC68041::balance_cfg(int cell, uint8_t cfg[6])
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


/*!******************************************************************************************************
This command will write the configuration registers of the LTC6804-1.
Write the LTC6804 configuration register
  1. Load cmd array with the write configuration command and PEC
  2. Load the cmd with LTC6804 configuration data
  3. Calculate the pec for the LTC6804 configuration data being transmitted
  4. Write configuration data to the LTC6804

 uint8_t config[6] is an array of the configuration data that will be written.
*********************************************************************************************************/
void LTC68041::wrcfg(uint8_t config[6])//A two dimensional array of the configuration data that will be written
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
    //wakeup_idle ();                                 //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
    //5
    spi_write_array(CMD_LEN, cmd);
    free(cmd);
}


/*!*******************************************************************************************************
Perform LUT lookup of cell SOC based on cell open circuit voltage.
SOC based on OCV lookup table.
SOC as a function of open cell voltage is non=linear, a lookup table seems to
be the best way to map between these two values.  
voc: Cell open circuit voltage, Volts.
return Function returns SOC from 0-1.

Funktion ist kaputt , muss ich mal fixen
*********************************************************************************************************/
float LTC68041::cell_compute_soc(float voc) {
    
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


/*!******************************************************************************************************
 Clears the LTC6804 Auxiliary registers

 The command clears the Auxiliary registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.
 LTC68041::clraux Function sequence:

  1. Load clraux command into cmd array
  2. Calculate clraux cmd PEC and load pec into cmd array
  3. send broadcast clraux command
*********************************************************************************************************/
void LTC68041::clraux()
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
    spi_write_read(cmd,4,0,0);
}


/*!*******************************************************************************************************
Reads and parses the LTC6804 cell voltage registers.

 The function is used to read the cell codes of the LTC6804.
 This function will send the requested read commands parse the data
 and store the cell voltages in cell_codes variable.
 
  1. Read every single cell voltage register
  2. Parse raw cell voltage data in cell_codes array
  3. Check the PEC of the data read back vs the calculated PEC for each read register command
  4. Return pec_error flag  
*********************************************************************************************************/
uint8_t LTC68041::rdcv() // Array of the parsed cell codes                  
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
    //l�uft von 1-4
    for (uint8_t cell_reg = 1; cell_reg<5; cell_reg++)                    //executes once for each of the LTC6804 cell voltage registers
    {
        data_counter = 0;
        rdcv_reg(cell_reg, cell_data );                //Reads a single Cell voltage register

        //2.
        for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
        {
            // loops once for each of the 3 cell voltage codes in the register

            parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);//Each cell code is received as two bytes and is combined to
            // create the parsed cell voltage code

            cellCodes[current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
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



/*!*******************************************************************************************************
 The function is used to read the of the Config Register Group A  of one LTC6804.
 This function sends the read commands, parses the data and stores the response in CFGR.
 
 1. Set command to RDCFG
 2. Calc the PEC 
 3. Wakeup Chip
 4. Send command and read response
 5. Check PEC
 6. Copy data to object
 7. Return Result -1= Error , 0= DataOkay
*********************************************************************************************************/
uint8_t LTC68041::rdcfg()                
{

    const uint8_t NUM_RX_BYT = 8; // number of bytes in the register + 2 bytes for the PEC
    const uint8_t BYT_IN_REG = 6;

    uint8_t pec_error = 0;
    uint16_t received_pec;
    uint8_t received_data[NUM_RX_BYT]; //data counter
    uint16_t data_pec;
    uint8_t data_counter=0; //data counter
    uint8_t cmd[4];
    uint16_t cmd_pec;

    //1
    cmd[0] = 0x00;
    cmd[1] = 0x02;
    //2
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);
    //3
    wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.
    //4
    spi_write_read(cmd,4,received_data,NUM_RX_BYT);
    //5
    //Set data pointer to the first PEC byte
    data_counter= BYT_IN_REG;
    received_pec = (received_data[data_counter] << 8) + received_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
    //(uint8_t len, uint8_t *data)
    data_pec = pec15_calc(BYT_IN_REG, received_data);
    if (received_pec != data_pec)
    {
        pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
        //are detected in the serial data
    }
    else
    {
        //6
        //Copy 	Target   	Source  	Size
        memcpy( CFGRr, received_data, BYT_IN_REG );
    }
    //printArray(NUM_RX_BYT, received_data);
    //7
    return(pec_error);
}



/*!*******************************************************************************************************
 The function is used to read the of the Auxiliary Register Group A  of one LTC6804.
 This function sends the read commands, parses the data and stores the response in AVAR.
 
 1. Set command to AVAR
 2. Calc the PEC 
 3. Wakeup Chip
 4. Send command and read response
 5. Check PEC
 6. Copy data to object
 7. Return Result -1= Error , 0= DataOkay
*********************************************************************************************************/
uint8_t LTC68041::rdauxa()                
{

    const uint8_t NUM_RX_BYT = 8; // number of bytes in the register + 2 bytes for the PEC
    const uint8_t BYT_IN_REG = 6;

    uint8_t pec_error = 0;
    uint16_t received_pec;
    uint8_t received_data[NUM_RX_BYT]; //data counter
    uint16_t data_pec;
    uint8_t data_counter=0; //data counter
    uint8_t cmd[4];
    uint16_t cmd_pec;

    //1
    cmd[0] = 0x00;
    cmd[1] = 0x0C;
    //2
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);
    //3
    wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.
    //4
    spi_write_read(cmd,4,received_data,NUM_RX_BYT);
    //5
    //Set data pointer to the first PEC byte
    data_counter= BYT_IN_REG;
    received_pec = (received_data[data_counter] << 8) + received_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
    //(uint8_t len, uint8_t *data)
    data_pec = pec15_calc(BYT_IN_REG, received_data);
    if (received_pec != data_pec)
    {
            pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
        //are detected in the serial data
    }
    else
    {
        //6
        //Copy 	Target   	Source  	Size
            memcpy( AVAR, received_data, BYT_IN_REG );
    }
    //printArray(NUM_RX_BYT, received_data);
    //7
    return(pec_error);
}


/*!*******************************************************************************************************
 The function is used to read the of the Auxiliary Register Group B  of one LTC6804.
 This function sends the read commands, parses the data and stores the response in AVBR.
 
 1. Set command to AVBR
 2. Calc the PEC 
 3. Wakeup Chip
 4. Send command and read response
 5. Check PEC
 6. Copy data to object
 7. Return Result -1= Error , 0= DataOkay
*********************************************************************************************************/
uint8_t LTC68041::rdauxb()                
{

    const uint8_t NUM_RX_BYT = 8; // number of bytes in the register + 2 bytes for the PEC
    const uint8_t BYT_IN_REG = 6;

    uint8_t pec_error = 0;
    uint16_t received_pec;
    uint8_t received_data[NUM_RX_BYT]; //data counter
    uint16_t data_pec;
    uint8_t data_counter=0; //data counter
    uint8_t cmd[4];
    uint16_t cmd_pec;

    //1
    cmd[0] = 0x00;
    cmd[1] = 0x0E;
    //2
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);
    //3
    wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.
    //4
    spi_write_read(cmd,4,received_data,NUM_RX_BYT);
    //5
    //Set data pointer to the first PEC byte
    data_counter= BYT_IN_REG;
    received_pec = (received_data[data_counter] << 8) + received_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
    //(uint8_t len, uint8_t *data)
    data_pec = pec15_calc(BYT_IN_REG, received_data);
    if (received_pec != data_pec)
    {
        pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
        //are detected in the serial data
    }
    else
    {
        //6
        //Copy 	Target   	Source  	Size
        memcpy( AVBR, received_data, BYT_IN_REG );
    }
    //printArray(NUM_RX_BYT, received_data);
    //7
    return(pec_error);
}


/*!*******************************************************************************************************
 The function is used to read the of the Status Register Group A of one LTC6804.
 This function sends the read commands, parses the data and stores the response in STAR.
 
 1. Set command to RDSTATA
 2. Calc the PEC 
 3. Wakeup Chip
 4. Send command and read response
 5. Check PEC
 6. Copy data to object
 7. Return Result -1= Error , 0= DataOkay
*********************************************************************************************************/


uint8_t LTC68041::rdstata()                
{

    const uint8_t NUM_RX_BYT = 8; // number of bytes in the register + 2 bytes for the PEC
    const uint8_t BYT_IN_REG = 6;

    uint8_t pec_error = 0;
    uint16_t received_pec;
    uint8_t received_data[NUM_RX_BYT]; //data counter
    uint16_t data_pec;
    uint8_t data_counter=0; //data counter
    uint8_t cmd[4];
    uint16_t cmd_pec;

    //1
    cmd[0] = 0x00;
    cmd[1] = 0x10;
    //2
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);
    //3
    wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.
    //4
    spi_write_read(cmd,4,received_data,NUM_RX_BYT);
    //5
    //Set data pointer to the first PEC byte
    data_counter= BYT_IN_REG;
    received_pec = (received_data[data_counter] << 8) + received_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
    //(uint8_t len, uint8_t *data)
    data_pec = pec15_calc(BYT_IN_REG, received_data);
    if (received_pec != data_pec)
    {
        pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
        //are detected in the serial data
    }
    else
    {
        //6
        //Copy 	Target   	Source  	Size
        memcpy( STAR, received_data, BYT_IN_REG );
        //Serial.println("Data Ok");
    }
    //printArrayByte(BYT_IN_REG, STAR);
    //printArray(NUM_RX_BYT, received_data);
    //7
    return(pec_error);
}


/*!*******************************************************************************************************
 The function is used to read the of the Status Register Group B  of one LTC6804.
 This function sends the read commands, parses the data and stores the response in STBR.
 
 1. Set command to RDSTATB
 2. Calc the PEC 
 3. Wakeup Chip
 4. Send command and read response
 5. Check PEC
 6. Copy data to object
 7. Return Result -1= Error , 0= DataOkay
*********************************************************************************************************/
uint8_t LTC68041::rdstatb()                
{

    const uint8_t NUM_RX_BYT = 8; // number of bytes in the register + 2 bytes for the PEC
    const uint8_t BYT_IN_REG = 6;

    uint8_t pec_error = 0;
    uint16_t received_pec;
    uint8_t received_data[NUM_RX_BYT]; //data counter
    uint16_t data_pec;
    uint8_t data_counter=0; //data counter
    uint8_t cmd[4];
    uint16_t cmd_pec;

    //1
    cmd[0] = 0x00;
    cmd[1] = 0x12;
    //2
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);
    //3
    wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.
    //4
    spi_write_read(cmd,4,received_data,NUM_RX_BYT);
    //5
    //Set data pointer to the first PEC byte
    data_counter= BYT_IN_REG;
    received_pec = (received_data[data_counter] << 8) + received_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
    //(uint8_t len, uint8_t *data)
    data_pec = pec15_calc(BYT_IN_REG, received_data);
    if (received_pec != data_pec)
    {
        pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
        //are detected in the serial data
    }
    else
    {
        //6
        //Copy 	Target   	Source  	Size
        memcpy( STBR, received_data, BYT_IN_REG );
    }
    //printArray(NUM_RX_BYT, received_data);
    //7
    return(pec_error);
}


/*!*******************************************************************************************************
 The command clears the cell voltage registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.

  1. Load clrcell command into cmd array
  2. Calculate clrcell cmd PEC and load pec into cmd array
  3. send broadcast clrcell command to LTC6804
*********************************************************************************************************/
void LTC68041::clrcell()
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
    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

    //4
    spi_write_read(cmd,4,0,0);
}


/*!*******************************************************************************************************
 The function reads a single GPIO voltage register and stores thre read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC68041::rdaux() command.
  1. Determine Command and initialize command array
  2. Calculate Command PEC
  3. Wake up isoSPI, this step is optional
  4. Send Global Command to LTC6804
[in] uint8_t reg; This controls which GPIO voltage register is read back.
          1: Read back auxiliary group A
          2: Read back auxiliary group B
[out] uint8_t *data; An array of the unparsed aux codes
*********************************************************************************************************/
void LTC68041::rdaux_reg(uint8_t reg, uint8_t *data)
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
    spi_write_read(cmd,4,data,REG_LEN);

}


/*!*******************************************************************************************************
Maps  global ADC control variables to the appropriate control bytes for each of the different ADC commands

[in] uint8_t MD The adc conversion mode
[in] uint8_t DCP Controls if Discharge is permitted during cell conversions
[in] uint8_t CH Determines which cells are measured during an ADC conversion command
[in] uint8_t CHG Determines which GPIO channels are measured during Auxiliary conversion command
*********************************************************************************************************/
/*
void LTC68041::set_adc(uint8_t MD, uint8_t DCP, uint8_t CH, uint8_t CHG )
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
*/

/*!*******************************************************************************************************
A complete SPI communication check.
Reads the Status register and checks PECs of the response
No other command necessary, Just call this and get 
[in] bool Result of the Check 1=Communication ok, 0=failure
 1. Request for status register
 2. Read fully status register
 3. calc PEC from response
 4. extract PEC from response
 5. compare PECs
 6. Send Serial message with result
*********************************************************************************************************/
void LTC68041::checkSPI()
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

    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    digitalWrite(LED_BUILTIN, HIGH);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);
    SPI.transfer(cmd[0]);
    SPI.transfer(cmd[1]);
    SPI.transfer(cmd[2]);
    SPI.transfer(cmd[3]);
    for(int i=0; i<(SizeConfigReg+PEClen);i++)
    {
        response[i] =SPI.transfer(0);
    }
    digitalWrite(pinCS, HIGH);
    SPI.endTransaction();
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


/*!*******************************************************************************************************
A complete SPI communication check.
Reads the Status register and checks PECs of the response
No other command necessary, Just call this and get 
[return] bool Result of the Check 1=Communication ok, 0=failure

This command does not send messages via Serial
 1. Request for status register
 2. Read fully status register
 3. calc PEC from response
 4. extract PEC from response
 5. compare PECs
 6. Return Result   
*********************************************************************************************************/
bool LTC68041::checkSPI_mute()
{
    uint16_t response_pec_calc;
    uint16_t response_pec;
    uint8_t cmd[8];

    uint8_t response[16];
    cmd[0] = 0x00;
    cmd[1] = 0x02;
    cmd[2] = 0x2b;
    cmd[3] = 0x0A;


    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);
    SPI.transfer(cmd[0]);
    SPI.transfer(cmd[1]);
    SPI.transfer(cmd[2]);
    SPI.transfer(cmd[3]);
    for(int i=0; i<(SizeConfigReg+PEClen);i++)
    {
        response[i] =SPI.transfer(0);
    }
    digitalWrite(pinCS, HIGH);
    SPI.endTransaction();

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

 
/*!*******************************************************************************************************
 The function is used to read the  parsed GPIO codes of the LTC6804.
 This function will send the requested  read commands parse the data
 and store the gpio voltages in aux_codes variable 
 
 [in] uint8_t reg; This controls which GPIO voltage register is read back.
  0: Read back all auxiliary registers
  1: Read back auxiliary group A
  2: Read back auxiliary group B
[out] uint16_t aux_codes[][6]; A two dimensional array of the gpio voltage codes.
[return]  int8_t, PEC Status  0: No PEC error detected -1: PEC error detected, retry read

*********************************************************************************************************/ 
int8_t LTC68041::rdaux(uint8_t reg, uint16_t aux_codes[6])
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
    data = (uint8_t *) malloc((NUM_RX_BYT)*sizeof(uint8_t));
    //1.a
    if (reg == 0)
    {
        //a.i
        for (uint8_t gpio_reg = 1; gpio_reg<3; gpio_reg++)                //executes once for each of the LTC6804 aux voltage registers
        {
        data_counter = 0;
        LTC68041::rdaux_reg(gpio_reg, data);                 //Reads the raw auxiliary register data into the data[] array

            //a.ii
            for (uint8_t current_gpio = 0; current_gpio< GPIO_IN_REG; current_gpio++) // This loop parses the read back data into GPIO voltages, it
            {
            // loops once for each of the 3 gpio voltage codes in the register

            parsed_aux = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
            // create the parsed gpio voltage code

            aux_codes[current_gpio +((gpio_reg-1)*GPIO_IN_REG)] = parsed_aux;
            data_counter=data_counter+2;                        //Because gpio voltage codes are two bytes the data counter
            //must increment by two for each parsed gpio voltage code

            }
            //a.iii
            received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
            //after the 6 gpio voltage data bytes
            data_pec = pec15_calc(BYT_IN_REG, &data[NUM_RX_BYT]);
            if (received_pec != data_pec)
            {
            pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
            //are detected in the received serial data
            }

            data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
            //must be incremented by 2 bytes to point to the next ICs gpio voltage data
        }
    }
    else
    {
        //b.i
        rdaux_reg(reg, data);
        // current_ic is used as an IC counter

        //b.ii
        for (int current_gpio = 0; current_gpio<GPIO_IN_REG; current_gpio++)    // This loop parses the read back data. Loops
        {
            // once for each aux voltage in the register

            parsed_aux = (data[data_counter] + (data[data_counter+1]<<8));        //Each gpio codes is received as two bytes and is combined to
            // create the parsed gpio voltage code
            aux_codes[current_gpio +((reg-1)*GPIO_IN_REG)] = parsed_aux;
            data_counter=data_counter+2;                      //Because gpio voltage codes are two bytes the data counter
            //must increment by two for each parsed gpio voltage code
        }
        //b.iii
        received_pec = (data[data_counter]<<8) + data[data_counter+1];         //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 gpio voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &data[NUM_RX_BYT]);
        if (received_pec != data_pec)
        {
            pec_error = -1;                               //The pec_error variable is simply set negative if any PEC errors
            //are detected in the received serial data
        }

        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs gpio voltage data
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


    /*!*******************************************************************************************************
    Starts cell voltage ADC conversions of the LTC6804 Cpin inputs.
    The type of ADC conversion executed can be changed by setting the associated global variables:

    MD   Determines the filter corner of the ADC
    CH   Determines which cell channels are converted
    DCP  Determines if Discharge is Permitted
    *********************************************************************************************************/
    void LTC68041::adcv()
    {

    uint8_t cmd[4];
    uint16_t cmd_pec;

    //1
    cmd[0] = 0x03;
    cmd[1] = 0x70;  //70=mit discharge , 60=Ohne

    //2
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);
    //3
    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    //4
    spi_write_array(4,cmd);

}


/*!******************************************************************************************************
Starts cell voltage conversion with test values from selftest 1
*********************************************************************************************************/
void LTC68041::adcv_test1()
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
    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

    //4
    spi_write_array(4,cmd);
}


/*!******************************************************************************************************
Starts cell voltage conversion with test values from selftest 2
*********************************************************************************************************/
void LTC68041::adcv_test2()
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
    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

    //4
    spi_write_array(4,cmd);
}


/*!******************************************************************************************************
Kind of debug function, a lot information are printed via Serial
*********************************************************************************************************/
bool LTC68041::rdstatus_debug()
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
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);
    SPI.transfer(cmd[0]);
    SPI.transfer(cmd[1]);
    SPI.transfer(cmd[2]);
    SPI.transfer(cmd[3]);

    digitalWrite(pinCS, HIGH);
    // spi_write_array(uint8_t len, uint8_t data[])
    delay(10); //wait for conversion

    cmd[0] = 0x00;		//Read Status Register Group A
    cmd[1] = 0x10;		//Read Status Register Group A
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    digitalWrite(pinCS, LOW);
    SPI.transfer(cmd[0]);
    SPI.transfer(cmd[1]);
    SPI.transfer(cmd[2]);
    SPI.transfer(cmd[3]);
    for(int i=0; i<(SizeStatusRegA+PEClen);i++)
    //for(int i=0; i<15;i++)
    {
        response[i] =SPI.transfer(1);
    }
    digitalWrite(pinCS, HIGH);
    SPI.endTransaction();

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

    //16-Bit ADC Measurement Value of Internal Die Temperature Temperature Measurement (�C) = ITMP � 100�V/7.5mV/�C � 273�C
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



/*!*******************************************************************************************************
Starts the conversion and reads the Statusregister
All in one Function for debugging
Calculates the internal Temperature
Checks the CRC and returns the Temperature
If itmp=5555, then CRC is wrong  
*********************************************************************************************************/
float  LTC68041::rditemp_debug()
{
    uint16_t response_pec_calc;
    uint16_t response_pec;
    uint8_t cmd[8];
    uint8_t response[16];
    uint16_t cmd_pec;
    uint16_t ITMP;
    uint8_t data[8];
    float offset=10;

    //Start Status group ADC Conversion and Poll Status
    cmd[0] = 0x5;			//ADSTAT (all)
    cmd[1] = 0x68;		//ADSTAT (all)
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //Send data and expect no answer yet
    spi_write_read(cmd, 4, response, 0);

    delay(10); //wait for conversion

    cmd[0] = 0x00;		//Read Status Register Group A
    cmd[1] = 0x10;		//Read Status Register Group A
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    spi_write_read(cmd, 4, response, (SizeStatusRegA+PEClen));         //Read the configuration data of all ICs on the daisy chain into

    for(int i=0; i<SizeStatusRegA;i++)
    {
        STAR[i]=response[i];
    }

    InternalTemp=cnvITMP(offset);
    Serial.print("\nTEMP:");
    Serial.print(InternalTemp);

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


/*!*******************************************************************************************************
calculates the Voltage out of the cellcodes
stored into cell voltages
*********************************************************************************************************/
void LTC68041::cnvCellVolt() 
{
    for(int i=0;i<cellNum;i++)
    {
        cellVoltage[i]=cellCodes[i] * 100E-6;
    }
}


/*!*******************************************************************************************************
calculates the Voltage out of the cellcodes
stored into cell voltages
*********************************************************************************************************/
void LTC68041::cnvAuxVolt() 
{
    AuxCodes[0]=AVAR[0]+((uint16_t)AVAR[1])<<8;
    AuxCodes[1]=AVAR[2]+((uint16_t)AVAR[3])<<8;
    AuxCodes[2]=AVAR[4]+((uint16_t)AVAR[5])<<8;
    AuxCodes[3]=AVBR[0]+((uint16_t)AVBR[1])<<8;
    AuxCodes[4]=AVBR[2]+((uint16_t)AVBR[3])<<8;

    for(int i=0;i<gpioNum;i++)
    {
        gpioVoltage[i]=AuxCodes[i] * 100E-6;
    }
}

/*!*******************************************************************************************************
converts the complete Status register with all values included
*********************************************************************************************************/
void LTC68041::cnvStatus() 
{
    SOC=(uint16_t)STAR[0];
    SOC=SOC+(((uint16_t)STAR[1]) << 8);
    SumCellVoltages=SOC*20*100E-6;		//Sum of All Cells Voltage = SOC � 100�V � 20
    for(int i=0;i<cellNum;i++)
    {
        cellVoltage[i]=cellCodes[i] * 100E-6;
    }
   ITMP=((uint16_t)STAR[2]) + (((uint16_t)STAR[3])<<8);
   //16-Bit ADC Measurement Value of Internal Die Temperature Temperature Measurement (�C) = ITMP � 100�V/7.5mV/�C � 273�C
   InternalTemp= ((float)ITMP * 100E-6 / 7.5E-3 - 273.0 ) + OffsetTemp;
   //Calc Analog Supply Voltage
   VA=(uint16_t)STAR[4];
   VA=VA+((uint16_t)STAR[5] << 8);
   AnalogSupplyVoltage=(float)VA*100E-6;
   //Calc Digital Supply Voltage
   VD=(uint16_t)STBR[0];
   VD=VD+((uint16_t)STBR[1] << 8);
   AnalogSupplyVoltage=(float)VD*100E-6;
   
   CUV[0]=bitRead(STBR[2], 0);
   COV[0]=bitRead(STBR[2], 1);
   
   CUV[1]=bitRead(STBR[2], 2);
   COV[1]=bitRead(STBR[2], 3);
   
   CUV[2]=bitRead(STBR[2], 4);
   COV[2]=bitRead(STBR[2], 5);
   
   CUV[3]=bitRead(STBR[2], 6);
   COV[3]=bitRead(STBR[2], 7);
   
   CUV[4]=bitRead(STBR[3], 0);
   COV[4]=bitRead(STBR[3], 1);
   
   CUV[5]=bitRead(STBR[3], 2);
   COV[5]=bitRead(STBR[3], 3);
   
   CUV[6]=bitRead(STBR[3], 4);
   COV[6]=bitRead(STBR[3], 5);  
   
   CUV[7]=bitRead(STBR[3], 6);
   COV[7]=bitRead(STBR[3], 7);  
   
   CUV[8]=bitRead(STBR[4], 0);
   COV[8]=bitRead(STBR[4], 1);  
   
   CUV[9]=bitRead(STBR[4], 2);
   COV[9]=bitRead(STBR[4], 3); 
    
   CUV[10]=bitRead(STBR[4], 4);
   COV[10]=bitRead(STBR[4], 5);  
   
   CUV[11]=bitRead(STBR[4], 6);
   COV[11]=bitRead(STBR[4], 7);  

   REV=STBR[5]>>4;
   
   MUXFAIL=bitRead(STBR[5], 1);
   
   THSD=bitRead(STBR[5], 0);
   
   
}

/*!*******************************************************************************************************
converts the complete config register from the last Read 
*********************************************************************************************************/
void LTC68041::cnvConfigRead() 
{
}


/*!*******************************************************************************************************
converts the complete config register from the next Write
*********************************************************************************************************/
void LTC68041::cnvConfigWrite() 
{
}

/*!*******************************************************************************************************
Reset all Discharge bits in the WriteConfig
*********************************************************************************************************/
bool LTC68041::setDischarge(int Discharge) 
{
    if(Discharge>0 & Discharge<12)
    {
        //bitWrite(, Discharge);
        return 1; //success
    }
    else
    {
        return 0;//wrong value
    }
   
}


void LTC68041::resetDischargeAll()
{
    Discharge=0;
}

/*!*******************************************************************************************************
  Starts an ADC conversions of the LTC6804 GPIO inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables.
  1. Load adax command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
void LTC68041::adax()
{
    uint8_t cmd[4];
    uint16_t cmd_pec;

    cmd[0] = 0x05;
    cmd[1] = 0x60;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    spi_write_array(4,cmd);
}


/*!*******************************************************************************************************
  Starts an ADC conversions of all cell voltages and the LTC6804 GPIO inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables.
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
void LTC68041::adcvax()
{
    uint8_t cmd[4];
    uint16_t cmd_pec;

    cmd[0] = 0x05;
    cmd[1] = 0x6F;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    spi_write_array(4,cmd);
}


/*!*******************************************************************************************************
  Starts an ADC conversions of the status values
  The type of ADC conversion executed can be changed by setting the associated global variables.
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
void LTC68041::adstat()
{
    uint8_t cmd[4];
    uint16_t cmd_pec;

    cmd[0] = 0x05;
    cmd[1] = 0x68;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    spi_write_array(4,cmd);
}

/*!*******************************************************************************************************
  Starts an ADC conversions of the open wire check with pullup
  The type of ADC conversion executed can be changed by the command value
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
void LTC68041::adowpu()
{
    uint8_t cmd[4];
    uint16_t cmd_pec;

    cmd[0] = 0x03;
    cmd[1] = 0x68;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    spi_write_array(4,cmd);
}


/*!*******************************************************************************************************
  Starts an ADC conversions of the open wire check with pulldown
  The type of ADC conversion executed can be changed by the command value
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
void LTC68041::adowpd()
{
    uint8_t cmd[4];
    uint16_t cmd_pec;

    cmd[0] = 0x03;
    cmd[1] = 0x28;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    spi_write_array(4,cmd);
}

/*!*******************************************************************************************************
Converts the raw temperature data into internal temperature
*********************************************************************************************************/
float LTC68041::cnvITMP(float offset)
{
    float InternalTemp;
    uint16_t ITMP;
    ITMP=STAR[2] | ((uint16_t)STAR[3])<<8;
    //16-Bit ADC Measurement Value of Internal Die Temperature Temperature Measurement (�C) = ITMP � 100�V/7.5mV/�C � 273�C
    InternalTemp= ((float)ITMP * 100E-6 / 7.5E-3 - 273.0 ) + offset;
    return InternalTemp;
}

/*!******************************************************************************************************
Reads and parses the LTC6804 cell voltage registers and returns some additional infos via Serial

 The function is used to read the cell codes of the LTC6804.
 This function will send the requested read commands parse the data
 and store the cell voltages in cell_codes variable.
 
  1. Read every single cell voltage register
  2. Parse raw cell voltage data in cell_codes array
  3. Check the PEC of the data read back vs the calculated PEC for each read register command
  4. Return pec_error flag 
*********************************************************************************************************/
uint8_t LTC68041::rdcv_debug(uint16_t cell_codes[cellNum]) // Array of the parsed cell codes                  
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
    //l�uft von 1-4
    for (uint8_t cell_reg = 1; cell_reg<5; cell_reg++)                    //executes once for each of the LTC6804 cell voltage registers
    {
        data_counter = 0;
        rdcv_reg(cell_reg, cell_data );                //Reads a single Cell voltage register

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
    cnvCellVolt();
    for (int i = 0 ; i < 12; i++)
    {
        CellVoltage[i]= cellVoltage[i];
        Serial.print(cellCodes[i], HEX);
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


/*!*******************************************************************************************************
 The function reads a single cell voltage register and stores the read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC68041::rdcv() command.

[in] uint8_t reg; This controls which cell voltage register is read back.
          1: Read back cell group A
          2: Read back cell group B
          3: Read back cell group C
          4: Read back cell group D

[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)
[out] uint8_t *data; An array of the unparsed cell codes
  LTC68041::rdcv_reg Function Process:
  1. Determine Command and initialize command array
  2. Calculate Command PEC
  3. Send Global Command to LTC6804 
*********************************************************************************************************/
void LTC68041::rdcv_reg(uint8_t reg, uint8_t *data)
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
    spi_write_read(cmd,4,data,REG_LEN);
}

void printArrayByte(uint8_t size, uint8_t *data)
{
    Serial.print("\nArray Content | ");
    for(int i=0;i<size;i++)
    {
        Serial.print(data[i], HEX);
        Serial.print("\t");
    }
    Serial.print(" |END \n");
}

void printArrayBool(uint8_t size, bool *data)
{
    Serial.print("\nArray Content | ");
    for(int i=0;i<size;i++)
    {
        Serial.print(data[i]);
        Serial.print("\t");
    }
    Serial.print(" |END \n");
}

void printArrayFloat(uint8_t size, float *data)
{
    Serial.print("\nArray Content | ");
    for(int i=0;i<size;i++)
    {
        Serial.print(data[i]);
        Serial.print("\t");
    }
    Serial.print(" |END \n");
}
