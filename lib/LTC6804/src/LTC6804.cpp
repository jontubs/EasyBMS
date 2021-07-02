/************************************************************

This library is based on the LTC68041.cpp by linear technology.
http://www.linear.com/product/LTC6804-1

I modified it make it compatible with the ESP8622

https://github.com/jontubs/EasyBMS
***********************************************************/


#include <stdint.h>
#include <Arduino.h>
#include "LTC6804.h"
#include <SPI.h>


/**
 * @brief Creating of the object LTC68041
 * 
 * @param pCS Pin used as chip select
 */
LTC68041::LTC68041(byte pCS)
  : pinCS(pCS), regs({}), md(MD_NORMAL)
{
    Serial.print("Objekt angelegt");

    regs.CFGR[CFGR0] = 0xFE;
}

/**
 * @brief Initializes the SPI instance used for communication
 * 
 * @param pinMOSI Pin used as MOSI
 * @param pinMISO Pin used as MISO
 * @param pinCLK Pin used as SCK
 */
void LTC68041::initSPI(byte pinMOSI, byte pinMISO, byte pinCLK)
{
    pinMode(pinMOSI, OUTPUT);
    pinMode(pinMISO, INPUT);
    pinMode(pinCLK, OUTPUT);
    pinMode(pinCS, OUTPUT);

    SPI.begin();
}

/**
 * @brief Uninitializes the used SPI instance
 * 
 */
void LTC68041::destroySPI()
{
    SPI.end()
}

/**
 * @brief Wake isoSPI up from idle state
 *        Generic wakeup commannd to wake isoSPI up out of idle 
 */
void LTC68041::wakeup_idle()
{
    digitalWrite(pinCS, LOW);
    delayMicroseconds(2); //Guarantees the isoSPI will be in ready mode
    digitalWrite(pinCS, HIGH);
}

/*!******************************************************************************************************
Calculates the CRC sum of some data bytes given by the array "data"
*********************************************************************************************************/
std::uint16_t LTC68041::calcPEC15(const std::uint16_t data)
{
    std::uint16_t remainder,addr;
    const std::uint8_t *p_data = reinterpret_cast<const std::uint8_t *>(&data);

    remainder = 16;//initialize the PEC
    for(int i = 0; i < sizeof(std::uint16_t); i++)
    {
        addr = ((remainder >> 7) ^ p_data[i]) & 0xff;//calculate PEC table address
        remainder = (remainder << 8) ^ crc15Table[addr];
    }

    return(remainder * 2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

/*!******************************************************************************************************
Calculates the CRC sum of some data bytes given by the array "data"
*********************************************************************************************************/
template<std::size_t N>
std::uint16_t LTC68041::calcPEC15(const std::array<std::uint8_t, N> &data)
{
    std::uint16_t remainder,addr;

    remainder = 16;//initialize the PEC
    for (const auto &element : data) // loops for each byte in data array
    {
        addr = ((remainder >> 7) ^ element) & 0xff;//calculate PEC table address
        remainder = (remainder << 8) ^ crc15Table[addr];
    }

    return(remainder * 2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

/*!******************************************************************************************************
Writes and read a set number of bytes using the SPI port.
Tested and runs fine
[in] std::array<std::uint8_t, N1> &tx_Data array of data to be written on the SPI port
[out] std::array<std::uint8_t, N2> &rx_data array that read data will be written too.
*********************************************************************************************************/
template<std::size_t N>
bool LTC68041::spi_read_cmd(const std::uint16_t cmd, std::array<std::uint8_t, N> &rx_data)
{
    std::uint16_t pec = calcPEC15(cmd);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);

    SPI.transfer16(cmd);
    SPI.transfer16(pec);

    for (auto &element : rx_data)
    {
        element = SPI.transfer(1);
    }

    pec = SPI.transfer16(1);

    digitalWrite(pinCS, HIGH);
    SPI.endTransaction();

    return (pec == calcPEC15(rx_data));
}

/*!******************************************************************************************************
Writes and read a set number of bytes using the SPI port without expecting an answer
std::array<std::uint8_t, N> &data //Array of bytes to be written on the SPI port
*********************************************************************************************************/
void LTC68041::spi_write_cmd(const std::uint16_t cmd)
{
    std::uint16_t pec = calcPEC15(cmd);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);

    SPI.transfer16(cmd);
    SPI.transfer16(pec);

    digitalWrite(pinCS, HIGH);
    SPI.endTransaction();
}

/*!******************************************************************************************************
calculates the bitpattern in the config for Undervoltage detection  
the config has to be written to the chip after this!
*********************************************************************************************************/
void LTC68041::cfgSetVUV(const float Undervoltage)
{
    uint16_t VUV = (Undervoltage / (0.0001 * 16)) - 1;		//calc bitpattern for UV

    //regs.CFGR[CFGR0] = 0xFE;
    regs.CFGR[CFGR1] = VUV & 0xFF;        //0x4E1 ; // 2.0V
    regs.CFGR[CFGR2] = (regs.CFGR[CFGR2] & 0xF0) | ((VUV >> 8) & 0x0F);
}

/*!******************************************************************************************************
calculates the bitpattern in the config for Overvoltage detection  
the config has to be written to the chip after this!
*********************************************************************************************************/
void LTC68041::cfgSetVOV(const float Overvoltage)
{
    //float Undervoltage=3.123;
    //float Overvoltage=3.923;
    uint16_t VOV = (Overvoltage / (0.0001 * 16));		//Calc bitpattern for OV

    //regs.CFGR[CFGR0] = 0xFE;
    regs.CFGR[CFGR2] = (regs.CFGR[CFGR2] & 0x0F) | ((VOV << 4) & 0xF0) ;
    regs.CFGR[CFGR3] = (VOV >> 4) & 0xFF;
}

/*!*******************************************************************************************************
Reads configuration registers of a LTC6804
Giving additional Debug infos via Serial
*********************************************************************************************************/

void LTC68041::cfgDebugOutput()
{
    cfgRead();         //Read the configuration data of all ICs on the daisy chain into

    Serial.print("gelesene Config: ");
    for (const auto &element : regs.CFGR)
    {
        Serial.print(element, HEX);
        Serial.print("\t");
    }
    Serial.print("\n");
}

/*!******************************************************************************************************
Sets  the configuration array for cell balancing
  1. Reset all Discharge Pins
  2. Calculate adcv cmd PEC and load pec into cmd array
  Discharge this cell (1-12), disable all other, IF -1 then all off
  TODO: refactor
*********************************************************************************************************/
void LTC68041::cfgSetDCC(std::bitset<12> dcc)
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

/**
 * @brief Set ADC filter Mode out of the six possible modes.
 *        Handles setting of MD bitsin command and ADCOPT bit in CFGR0
 * 
 * @param mode ADC mode as enum value of type ADCFilterMode
 */
void LTC68041::cfgSetADCMode(ADCFilterMode mode)
{
    switch(mode)
    {
        case ADCFilterMode::FAST:
        case ADCFilterMode::BANDWIDTH_27KHZ:
            md = MD_FAST;
            regs.CFGR[CFGR0] &= 0xFE;
            break;
        case ADCFilterMode::NORMAL:
        case ADCFilterMode::BANDWIDTH_7KHZ:
            md = MD_NORMAL;
            regs.CFGR[CFGR0] &= 0xFE;
            break;
        case ADCFilterMode::FILTERED:
        case ADCFilterMode::BANDWIDTH_26HZ:
            md = MD_FILTERED;
            regs.CFGR[CFGR0] &= 0xFE;
            break;
        case ADCFilterMode::BANDWIDTH_14KHZ:
            md = MD_FAST;
            regs.CFGR[CFGR0] |= 0x1;
            break;
        case ADCFilterMode::BANDWIDTH_3KHZ:
            md = MD_NORMAL;
            regs.CFGR[CFGR0] |= 0x1;
            break;
        case ADCFilterMode::BANDWIDTH_2KHZ:
            md = MD_FILTERED;
            regs.CFGR[CFGR0] |= 0x1;
            break;
        default:
            break;
    }
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
void LTC68041::cfgWrite()//A two dimensional array of the configuration data that will be written
{
    std::uint16_t cmd = static_cast<std::uint16_t>(WRCFG);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);

    SPI.transfer16(cmd);
    SPI.transfer16(calcPEC15(cmd););

    for(const auto &element: regs.CFGR)
    {
        SPI.transfer(element);
    }

    SPI.transfer16(calcPEC15(regs.CFGR));

    digitalWrite(pinCS, HIGH);
    SPI.endTransaction();
/*
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
*/
}

/*!*******************************************************************************************************
Perform LUT lookup of cell SOC based on cell open circuit voltage.
SOC based on OCV lookup table.
SOC as a function of open cell voltage is non=linear, a lookup table seems to
be the best way to map between these two values.  
voc: Cell open circuit voltage, Volts.
return Function returns SOC from 0-1.

Funktion ist kaputt , muss ich mal fixen
TODO: refactor
*********************************************************************************************************/
float LTC68041::cellComputeSOC(float voc) {
    
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
void LTC68041::cmdCLRAUX()
{
    //3
    wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
    //4
    spi_write_cmd(static_cast<std::uint16_t>(CLRAUX));
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
bool LTC68041::readCells(const unsigned int groupCount) // Array of the parsed cell codes                  
{
    switch(groupCount)
    {
        case 4:
            if(!spi_read_cmd(static_cast<std::uint16_t>(RDCVD), regs.CVDR))
                return false;

            parseCellVoltages(4, regs.CVDR, cellVoltage);
            [[fallthrough]]
        case 3:
            if(!spi_read_cmd(static_cast<std::uint16_t>(RDCVC), regs.CVCR))
                return false;

            parseCellVoltages(3, regs.CVCR, cellVoltage);
            [[fallthrough]]
        case 2:
            if(!spi_read_cmd(static_cast<std::uint16_t>(RDCVB), regs.CVBR))
                return false;

            parseCellVoltages(2, regs.CVBR, cellVoltage);
            [[fallthrough]]
        case 1:
            if(!spi_read_cmd(static_cast<std::uint16_t>(RDCVA), regs.CVAR))
                return false;

            parseCellVoltages(1, regs.CVAR, cellVoltage);
            break;
        default:
            return false;
            break;
    }

    return true;
}

/**
 * @brief Helper function to calculate voltages in volt from register values
 * 
 * @param group Register group to convert
 * @param regs Data of register group as array
 * @param data Array for target values
 */
template<std::size_t N>
inline void LTC68041::parseVoltages(const unsigned int group, const std::array<std::uint8_t, SIZEREG> &regs, std::array<std::float, N> &data)
{
    unsigned int index = (group - 1) * 3;

    for(int i = 0; i < (regs.size() - 1); i++)
    {
        data[index++] = static_cast<float>(regs[i] | (regs[++i] << 8)) * 100E-6;
    }
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
bool LTC68041::cfgRead()                
{
    return spi_read_cmd(static_cast<std::uint16_t>(RDCFG), regs.CFGR);
}

/*!*******************************************************************************************************
 The command clears the cell voltage registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.

  1. Load clrcell command into cmd array
  2. Calculate clrcell cmd PEC and load pec into cmd array
  3. send broadcast clrcell command to LTC6804
*********************************************************************************************************/
void LTC68041::cmdCLRCELL()
{
    //3
    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

    //4
    spi_write_cmd(static_cast<std::uint16_t>(CLRCELL));
}

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
bool LTC68041::checkSPI(const bool dbgOut)
{
    std::array<std::uint8_t, 6> response = {};

    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    if(dgbOut)
        digitalWrite(LED_BUILTIN, HIGH);

    bool ret = spi_write_cmd(static_cast<std::uint16_t>(RDCFG), response));

    if(dgbOut)
    {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("");
        Serial.print("RSP: ");

        for(const auto &element : reponse)
        {
            Serial.print(element, HEX);
            Serial.print(" ");
        }

        Serial.println("");
    }

    if(ret)
    {
        if(dgbOut)
            Serial.println("PEC was correct");

        return true;
    }
    else
    {
        if(dgbOut)
            Serial.println("PEC was NOT correct, check Hardware");

        return false;
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
bool LTC68041::readAUX(const unsigned int group)
{
    switch(group)
    {
        case 0xA:
            wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.
            if(!spi_read_cmd(static_cast<std::uint16_t>(RDAUXA), regs.AVAR))
                return false;

            parseVoltages(1, regs.AVAR, gpioVoltage);
            break;
        case 0xB:
            wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.
            if(!spi_read_cmd(static_cast<std::uint16_t>(RDAUXB), regs.AVBR))
                return false;

            parseVoltages(2, regs.AVBR, gpioVoltage);
            break;
        default:
            return false;
    }
    
    return true;
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
bool LTC68041::readStatus(const unsigned int group)                
{
    switch(group)
    {
        case 0xA:
            wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.
            return spi_read_cmd(static_cast<std::uint16_t>(RDSTATA), regs.STAR);
        case 0xB:
            wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.
            return spi_read_cmd(static_cast<std::uint16_t>(RDSTATB), regs.STBR);
        default:
            return false;
    }
}

/*!******************************************************************************************************
Kind of debug function, a lot information are printed via Serial
*********************************************************************************************************/
void LTC68041::readStatusDbg()
{
    cmdADSTAT();
    delay(10); //wait for conversion

    readStatus(0xA);

    Serial.println("");
    Serial.print("RSP Status A: ");

    for(const auto &element : regs.STAR)
    {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    readStatus(0xB);

    Serial.println("");
    Serial.print("RSP Status B: ");

    for(const auto &element : regs.STBR)
    {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    ITMP = regs.STAR[STAR2] | ((uint16_t)regs.STAR[STAR3]) << 8;

    Serial.println("");
    Serial.print("ITMP:");
    Serial.println(ITMP);

    //16-Bit ADC Measurement Value of Internal Die Temperature Temperature Measurement (�C) = ITMP � 100�V/7.5mV/�C � 273�C
    Serial.print("InternalTemp:");
    Serial.println(parseTemp(10));
    Serial.println("");
}

/*!*******************************************************************************************************
converts the complete Status register with all values included
*********************************************************************************************************/
void LTC68041::cnvStatus() 
{
    SOC=(uint16_t)STAR[0];
    SOC=SOC+(((uint16_t)STAR[1]) << 8);
    SumCellVoltages=SOC*20*100E-6;		//Sum of All Cells Voltage = SOC � 100�V � 20
    for(int i=0;i<CELLNUM;i++)
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
  Starts cell voltage ADC conversions of the LTC6804 Cpin inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables:  
  MD   Determines the filter corner of the ADC
  CH   Determines which cell channels are converted
  DCP  Determines if Discharge is Permitted
*********************************************************************************************************/
void LTC68041::cmdADCV(DischargeCtrl dcp, CellChannel ch)
{

    std::uint16_t cmd = static_cast<std::uint16_t>(ADCV);
    cmd |= static_cast<std::uint16_t>(md);
    cmd |= static_cast<std::uint16_t>(dcp);
    cmd |= static_cast<std::uint16_t>(ch);

    //3
    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    //4
    spi_write_cmd(cmd);

}

/*!******************************************************************************************************
Starts cell voltage conversion with test values from selftest 2
*********************************************************************************************************/
void LTC68041::cmdCVST(SelfTestMode st)
{

    std::uint16_t cmd = static_cast<std::uint16_t>(CVST);
    cmd |= static_cast<std::uint16_t>(md);
    cmd |= static_cast<std::uint16_t>(st);

    //3
    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

    //4
    spi_write_cmd(cmd);
}

/*!*******************************************************************************************************
  Starts an ADC conversions of the LTC6804 GPIO inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables.
  1. Load adax command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
void LTC68041::cmdADAX(AuxChannel chg)
{
    std::uint16_t cmd = static_cast<std::uint16_t>(ADAX);
    cmd |= static_cast<std::uint16_t>(md);
    cmd |= static_cast<std::uint16_t>(chg);

    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    spi_write_cmd(cmd);
}


/*!*******************************************************************************************************
  Starts an ADC conversions of all cell voltages and the LTC6804 GPIO inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables.
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
void LTC68041::cmdADCVAX(DischargeCtrl dcp)
{
    std::uint16_t cmd = static_cast<std::uint16_t>(ADCVAX);
    cmd |= static_cast<std::uint16_t>(md);
    cmd |= static_cast<std::uint16_t>(dcp);

    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    spi_write_cmd(cmd);
}


/*!*******************************************************************************************************
  Starts an ADC conversions of the status values
  The type of ADC conversion executed can be changed by setting the associated global variables.
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
void LTC68041::cmdADSTAT(StatusGroup chst)
{
    std::uint16_t cmd = static_cast<std::uint16_t>(ADSTAT);
    cmd |= static_cast<std::uint16_t>(md);
    cmd |= static_cast<std::uint16_t>(chst);

    wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    spi_write_cmd(cmd);
}

/*!*******************************************************************************************************
  Starts an ADC conversions of the open wire check with pullup
  The type of ADC conversion executed can be changed by the command value
  1. Load command into cmd array
  2. Calculate adax cmd PEC and load pec into cmd array
  3. send broadcast adax command to LTC6804
*********************************************************************************************************/
void LTC68041::cmdADOW(PUPCtrl pup, DischargeCtrl dcp, CellChannel ch)
{
    std::uint16_t cmd = static_cast<std::uint16_t>(ADOW);
    cmd |= static_cast<std::uint16_t>(md);
    cmd |= static_cast<std::uint16_t>(pup);
    cmd |= static_cast<std::uint16_t>(dcp);
    cmd |= static_cast<std::uint16_t>(ch);

    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    spi_write_cmd(cmd);
}

/*!*******************************************************************************************************
Converts the raw temperature data into internal temperature
*********************************************************************************************************/
float LTC68041::parseTemp(float offset)
{
    float InternalTemp;
    uint16_t ITMP;
    ITMP = regs.STAR[STAR2] | ((uint16_t)regs.STAR[STAR3]) << 8;
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
void LTC68041::readCellsDbg() // Array of the parsed cell codes
{
    if(readCells();)
        Serial.print("Read failed!");

    Serial.print("Cell Voltages: ");

    for (const auto &element : cellVoltage)
    {
        Serial.print(element);
        Serial.print("\t");
    }

    Serial.println();
}

inline void serialPrint(uint8_t data)
{
    Serial.print(data, HEX);
}

inline void serialPrint(bool data)
{
    Serial.print(data);
}

inline void serialPrint(float data)
{
    Serial.print(data);
}

template<typename T, std::size_t N>
void printArray(std::array<T, N> &arr)
{
    Serial.print("\nArray Content | ");
    for(const auto &element : arr)
    {
        serialPrint(element);
        Serial.print("\t");
    }
    Serial.print(" |END \n");
}