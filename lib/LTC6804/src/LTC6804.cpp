/************************************************************

This library is based on the LTC68041.cpp by linear technology.
http://www.linear.com/product/LTC6804-1

I modified it make it compatible with the ESP8622

https://github.com/jontubs/EasyBMS
***********************************************************/


#include <stdint.h>
#include "LTC6804.h"


#include <cmath>


/**
 * @brief Creating of the object LTC68041
 * 
 * @param pCS Pin used as chip select
 */
LTC68041::LTC68041(byte pCS)
  : cellVoltage({}), auxVoltage({}), offsetTemp(0.0), md(MD_NORMAL), pinCS(pCS), regs({})
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
    SPI.end();
}

/**
 * @brief Wake isoSPI up from idle state
 *        Generic wakeup commannd to wake isoSPI up out of idle 
 */
void LTC68041::wakeup_idle() const
{
    digitalWrite(pinCS, LOW);
    delayMicroseconds(2); //Guarantees the isoSPI will be in ready mode
    digitalWrite(pinCS, HIGH);
}

/*!******************************************************************************************************
Calculates the CRC sum of some data bytes given by the array "data"
*********************************************************************************************************/
constexpr std::uint16_t LTC68041::calcPEC15(const std::uint16_t data) const
{
    std::uint16_t remainder = 16, addr = 0;//initialize the PEC

    addr = ((remainder >> 7) ^ (data >> 8)) & 0xff;//calculate PEC table address
    remainder = (remainder << 8) ^ crc15Table[addr];

    addr = ((remainder >> 7) ^ (data & 0xff)) & 0xff;//calculate PEC table address
    remainder = (remainder << 8) ^ crc15Table[addr];

    return(remainder * 2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

/*!******************************************************************************************************
Calculates the CRC sum of some data bytes given by the array "data"
*********************************************************************************************************/
template<std::size_t N>
constexpr std::uint16_t LTC68041::calcPEC15(const std::array<std::uint8_t, N> &data) const
{
    std::uint16_t remainder = 16, addr = 0;//initialize the PEC

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
void LTC68041::spi_write_cmd(const std::uint16_t cmd) const
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
    unsigned int VUV = static_cast<unsigned int>(Undervoltage / (0.0001f * 16.0f)) - 1;		//calc bitpattern for UV

    //regs.CFGR[CFGR0] = 0xFE;
    regs.CFGR[CFGR1] = VUV & CFG1_VUV_MSK;        //0x4E1 ; // 2.0V
    regs.CFGR[CFGR2] = (regs.CFGR[CFGR2] & (~CFG2_VUV_MSK)) | ((VUV >> 8) & CFG2_VUV_MSK);
}

float LTC68041::cfgGetVUV() const
{
    unsigned int value;

    value = regs.CFGR[CFGR1] | (static_cast<unsigned int>(regs.CFGR[CFGR2] & CFG2_VUV_MSK) << 8);
    return (static_cast<float>(value + 1) * 16.0f * 0.001f);
}

/*!******************************************************************************************************
calculates the bitpattern in the config for Overvoltage detection  
the config has to be written to the chip after this!
*********************************************************************************************************/
void LTC68041::cfgSetVOV(const float Overvoltage)
{
    //float Undervoltage=3.123;
    //float Overvoltage=3.923;
    unsigned int VOV = static_cast<unsigned int>(Overvoltage / (0.0001f * 16.0f));		//Calc bitpattern for OV

    //regs.CFGR[CFGR0] = 0xFE;
    regs.CFGR[CFGR2] = (regs.CFGR[CFGR2] & (~CFG2_VOV_MSK)) | ((VOV << 4) & CFG2_VOV_MSK) ;
    regs.CFGR[CFGR3] = (VOV >> 4) & CFG3_VOV_MSK;
}

float LTC68041::cfgGetVOV() const
{
    unsigned int value;

    value = ((regs.CFGR[CFGR2] & CFG2_VOV_MSK) >> 4) | (static_cast<unsigned int>(regs.CFGR[CFGR3]) << 4);
    return (static_cast<float>(value) * 16.0f * 0.001f);
}

/*!*******************************************************************************************************
Reads configuration registers of a LTC6804
Giving additional Debug infos via Serial
*********************************************************************************************************/

void LTC68041::readCfgDbg()
{
    Serial.println();
    Serial.print("Gelesene Config: ");

    for (const auto &element : regs.CFGR)
    {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
}

/*!******************************************************************************************************
Sets  the configuration array for cell balancing
  1. Reset all Discharge Pins
  2. Calculate adcv cmd PEC and load pec into cmd array
  Discharge this cell (1-12), disable all other, IF -1 then all off
*********************************************************************************************************/
void LTC68041::cfgSetDCC(std::bitset<12> dcc)
{
    // assert 0x0fff
    regs.CFGR[CFGR4] = (dcc.to_ulong() & CFG4_DCC_MSK); // (regs.CFGRx[CFGR1] & CFG1_DCC_INVMSK) |
    regs.CFGR[CFGR5] = (regs.CFGR[CFGR5] & (~CFG5_DCC_MSK)) | ((dcc.to_ulong() >> 8) & CFG5_DCC_MSK);
}

std::bitset<12> LTC68041::cfgGetDCC() const
{
    return std::bitset<12>{regs.CFGR[CFGR4] | (static_cast<unsigned long long>(regs.CFGR[CFGR5] & CFG5_DCC_MSK) << 8)};
}

void LTC68041::cfgSetDischargeTimeout(DischargeTimeout timeout)
{
    regs.CFGR[CFGR5] = (regs.CFGR[CFGR5] & (~CFG5_DCTO_MSK)) | timeout;
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
        case ADCFilterMode::BANDWIDTH_27KHZ:
            md = MD_FAST;
            regs.CFGR[CFGR0] &= ~(1 << CFGR0_ADCOPT_Pos);
            break;
        case ADCFilterMode::BANDWIDTH_7KHZ:
            md = MD_NORMAL;
            regs.CFGR[CFGR0] &= ~(1 << CFGR0_ADCOPT_Pos);
            break;
        case ADCFilterMode::BANDWIDTH_26HZ:
            md = MD_FILTERED;
            regs.CFGR[CFGR0] &= ~(1 << CFGR0_ADCOPT_Pos);
            break;
        case ADCFilterMode::BANDWIDTH_14KHZ:
            md = MD_FAST;
            regs.CFGR[CFGR0] = (regs.CFGR[CFGR0] & (~CFG0_ADCOPT_MSK)) | (1 << CFGR0_ADCOPT_Pos);
            break;
        case ADCFilterMode::BANDWIDTH_3KHZ:
            md = MD_NORMAL;
            regs.CFGR[CFGR0] = (regs.CFGR[CFGR0] & (~CFG0_ADCOPT_MSK)) | (1 << CFGR0_ADCOPT_Pos);
            break;
        case ADCFilterMode::BANDWIDTH_2KHZ:
            md = MD_FILTERED;
            regs.CFGR[CFGR0] = (regs.CFGR[CFGR0] & (~CFG0_ADCOPT_MSK)) | (1 << CFGR0_ADCOPT_Pos);
            break;
        default:
            break;
    }
}

void LTC68041::cfgSetRefOn(const bool value)
{
    regs.CFGR[CFGR0] = (regs.CFGR[CFGR0] & (~CFG0_REFON_MSK)) | (value << CFGR0_REFON_Pos);
}

bool LTC68041::cfgGetRefOn()
{
    return (regs.CFGR[CFGR0] & CFG0_REFON_MSK);
}

bool LTC68041::cfgGetSWTENPin() const
{
    return (regs.CFGR[CFGR0] & CFG0_SWTRD_MSK);
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
    std::uint16_t cmd = WRCFG;

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);

    SPI.transfer16(cmd);
    SPI.transfer16(calcPEC15(cmd));

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
void LTC68041::cmdCLRAUX() const
{
    //3
    wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
    //4
    spi_write_cmd(CLRAUX);
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
template<std::size_t N>
bool LTC68041::getCellVoltages(std::array<float, N> &voltages, const CellChannel ch)
{
    switch(ch)
    {
        case CellChannel::CH_ALL:
        case CellChannel::CH_CELL_1_AND_7:
        case CellChannel::CH_CELL_2_AND_8:
        case CellChannel::CH_CELL_3_AND_9:
            if(!spi_read_cmd(RDCVA, regs.CVAR))
                return false;

            if(!spi_read_cmd(RDCVC, regs.CVCR))
                return false;

            parseVoltages(0, regs.CVAR, cellVoltage);
            parseVoltages(2, regs.CVCR, cellVoltage);

            if(ch != CellChannel::CH_ALL)
                break;
        case CellChannel::CH_CELL_4_AND_10:
        case CellChannel::CH_CELL_5_AND_11:
        case CellChannel::CH_CELL_6_AND_12:
            if(!spi_read_cmd(RDCVB, regs.CVBR))
                return false;

            if(!spi_read_cmd(RDCVD, regs.CVDR))
                return false;

            parseVoltages(1, regs.CVBR, cellVoltage);
            parseVoltages(3, regs.CVDR, cellVoltage);
            break;

        default:
            return false;
    }

    for(unsigned int i = 0; i < voltages.size(); i++)
    {
        voltages[i] = cellVoltage[i];
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
constexpr inline void LTC68041::parseVoltages(const unsigned int group, const std::array<std::uint8_t, SIZEREG> &regs, std::array<float, N> &data)
{
    unsigned int index = (group) * 3;

    for(unsigned int i = 0; i < (regs.size() - 1); i += 2)
    {
        data[index++] = static_cast<float>(regs[i] | (static_cast<unsigned int>(regs[i + 1]) << 8)) * 100E-6;
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
    return spi_read_cmd(RDCFG, regs.CFGR);
}

/*!*******************************************************************************************************
 The command clears the cell voltage registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.

  1. Load clrcell command into cmd array
  2. Calculate clrcell cmd PEC and load pec into cmd array
  3. send broadcast clrcell command to LTC6804
*********************************************************************************************************/
void LTC68041::cmdCLRCELL() const
{
    //3
    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

    //4
    spi_write_cmd(CLRCELL);
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
    if(dbgOut)
        digitalWrite(LED_BUILTIN, HIGH);

    bool ret = spi_read_cmd(RDCFG, response);

    if(dbgOut)
    {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println();
        Serial.print("RSP: ");

        for(const auto &element : response)
        {
            Serial.print(element, HEX);
            Serial.print(" ");
        }

        Serial.println();
    }

    if(ret)
    {
        if(dbgOut)
            Serial.println("PEC was correct");

        return true;
    }
    else
    {
        if(dbgOut)
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
float LTC68041::getAuxVoltage(const AuxChannel chg)
{
    Commands cmd;
    int index, parse_index;
    std::array<std::uint8_t, SIZEREG> *p_arr;

    wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.

    switch(chg)
    {
        case AuxChannel::CHG_GPIO1:
            cmd = RDAUXA;
            p_arr = &regs.AVAR;
            index = 0;
            parse_index = 0;
            break;
        case AuxChannel::CHG_GPIO2:
            cmd = RDAUXA;
            p_arr = &regs.AVAR;
            index = 1;
            parse_index = 0;
            break;
        case AuxChannel::CHG_GPIO3:
            cmd = RDAUXA;
            p_arr = &regs.AVAR;
            index = 2;
            parse_index = 0;
            break;
        case AuxChannel::CHG_GPIO4:
            cmd = RDAUXB;
            p_arr = &regs.AVBR;
            index = 3;
            parse_index = 1;
            break;
        case AuxChannel::CHG_GPIO5:
            cmd = RDAUXB;
            p_arr = &regs.AVBR;
            index = 4;
            parse_index = 1;
            break;
        case AuxChannel::CHG_VREF2:
            cmd = RDAUXB;
            p_arr = &regs.AVBR;
            index = 5;
            parse_index = 1;
            break;
        case AuxChannel::CHG_ALL:
            spi_read_cmd(RDAUXA, regs.AVAR);
            spi_read_cmd(RDAUXB, regs.AVBR);
            parseVoltages(0, regs.AVAR, auxVoltage);
            parseVoltages(1, regs.AVBR, auxVoltage);
            return NAN;
        default:
            return NAN;
    }

    if(!spi_read_cmd(cmd, *p_arr))
        return NAN;
        
    parseVoltages(parse_index, *p_arr, auxVoltage);

    return auxVoltage[index];
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
float LTC68041::getStatusVoltage(const StatusGroup chst)
{
    wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake, this command can be removed.

    switch(chst)
    {
        case StatusGroup::CHST_SOC:
            if(!spi_read_cmd(RDSTATA, regs.STAR))
                return NAN;
            //16-Bit ADC Measurement Value of Sum of all cell voltages Sum of all cell voltages = SOC * 100µV * 20
            return static_cast<float>(regs.STAR[STAR0] | (static_cast<unsigned int>(regs.STAR[STAR1])) << 8) * 100E-6 * 20.0;
        case StatusGroup::CHST_ITMP:
            if(!spi_read_cmd(RDSTATA, regs.STAR))
                return NAN;
            //16-Bit ADC Measurement Value of Internal Die Temperature Temperature Measurement (°C) = ITMP * 100µV / 7.5mV/°C - 273°C
            return (static_cast<float>(regs.STAR[STAR2] | (static_cast<unsigned int>(regs.STAR[STAR3])) << 8) * 100E-6 / 7.5E-3 - 273.0 ) + offsetTemp;
        case StatusGroup::CHST_VA:
            if(!spi_read_cmd(RDSTATA, regs.STAR))
                return NAN;
            //16-Bit ADC Measurement Value of Analog Power Supply Voltage Analog Power Supply Voltage = VA * 100µV Normal Range Is within 4.5V to 5.5V
            return static_cast<float>(regs.STAR[STAR4] | (static_cast<unsigned int>(regs.STAR[STAR5])) << 8) * 100E-6;
        case StatusGroup::CHST_VD:
            if(!spi_read_cmd(RDSTATB, regs.STBR))
                return NAN;
            //16-Bit ADC Measurement Value of Digital Power Supply Voltage Digital Power Supply Voltage = VA * 100µV Normal Range Is within 2.7V to 3.6V
            return static_cast<float>(regs.STBR[STBR0] | (static_cast<unsigned int>(regs.STBR[STBR1])) << 8) * 100E-6;
        case StatusGroup::CHST_ALL:
            spi_read_cmd(RDSTATA, regs.STAR);
            spi_read_cmd(RDSTATB, regs.STBR);
            return NAN;
        default:
            return NAN;
    }
}

bool LTC68041::getStatusMUXFail()
{
    if(!spi_read_cmd(RDSTATB, regs.STBR))
        return false;

    return (regs.STBR[STBR5] & STBR5_MUXFAIL_MSK);
}

bool LTC68041::getStatusThermalShutdown()
{
    if(!spi_read_cmd(RDSTATB, regs.STBR))
        return false;

    return (regs.STBR[STBR5] & STBR5_THSD_MSK);
}

//Cell x Overvoltage Flag x = 1 to 12 Cell Voltage Compared to VOV Comparison Voltage 0 -> Cell x Not Flagged for Overvoltage Condition. 1 -> Cell x Flagged
std::bitset<12> LTC68041::getStatusOverVoltageFlags()
{
    std::bitset<12> ret;

    if(!spi_read_cmd(RDSTATB, regs.STBR))
    {
        ret.reset();
        return ret;
    }

    ret[0] = bitRead(regs.STBR[STBR2], 1);
    ret[1] = bitRead(regs.STBR[STBR2], 3);
    ret[2] = bitRead(regs.STBR[STBR2], 5);
    ret[3] = bitRead(regs.STBR[STBR2], 7);
    ret[4] = bitRead(regs.STBR[STBR3], 1);
    ret[5] = bitRead(regs.STBR[STBR3], 3);
    ret[6] = bitRead(regs.STBR[STBR3], 5);
    ret[7] = bitRead(regs.STBR[STBR3], 7);
    ret[8] = bitRead(regs.STBR[STBR4], 1);
    ret[9] = bitRead(regs.STBR[STBR4], 3);
    ret[10] = bitRead(regs.STBR[STBR4], 5);
    ret[11] = bitRead(regs.STBR[STBR4], 7);

    return ret;
}

//Cell x Undervoltage Flag x = 1 to 12 Cell Voltage Compared to VUV Comparison Voltage 0 -> Cell x Not Flagged for Undervoltage Condition. 1 -> Cell x Flagged
std::bitset<12> LTC68041::getStatusUnderVoltageFlags()
{
    std::bitset<12> ret;

    if(!spi_read_cmd(RDSTATB, regs.STBR))
    {
        ret.reset();
        return ret;
    }

    ret[0] = bitRead(regs.STBR[STBR2], 0);
    ret[1] = bitRead(regs.STBR[STBR2], 2);
    ret[2] = bitRead(regs.STBR[STBR2], 4);
    ret[3] = bitRead(regs.STBR[STBR2], 6);
    ret[4] = bitRead(regs.STBR[STBR3], 0);
    ret[5] = bitRead(regs.STBR[STBR3], 2);
    ret[6] = bitRead(regs.STBR[STBR3], 4);
    ret[7] = bitRead(regs.STBR[STBR3], 6);
    ret[8] = bitRead(regs.STBR[STBR4], 0);
    ret[9] = bitRead(regs.STBR[STBR4], 2);
    ret[10] = bitRead(regs.STBR[STBR4], 4);
    ret[11] = bitRead(regs.STBR[STBR4], 6);

    return ret;
}

int LTC68041::getStatusRevision()
{
    return ((regs.STBR[STBR5] & STBR5_REV_MSK) >> 4);
}

/*!******************************************************************************************************
Kind of debug function, a lot information are printed via Serial
*********************************************************************************************************/
void LTC68041::readStatusDbg()
{
    Serial.println();
    Serial.print("RSP Status A: ");

    for(const auto &element : regs.STAR)
    {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial.print("RSP Status B: ");

    for(const auto &element : regs.STBR)
    {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    std::uint16_t ITMP = regs.STAR[STAR2] | (regs.STAR[STAR3] << 8);

    Serial.println();
    Serial.print("ITMP:");
    Serial.println(ITMP, HEX);

    //16-Bit ADC Measurement Value of Internal Die Temperature Temperature Measurement (�C) = ITMP � 100�V/7.5mV/�C � 273�C
    Serial.print("InternalTemp:");
    Serial.println(getStatusVoltage(StatusGroup::CHST_ITMP));
    Serial.println();
}

/*!*******************************************************************************************************
  Starts cell voltage ADC conversions of the LTC6804 Cpin inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables:  
  MD   Determines the filter corner of the ADC
  CH   Determines which cell channels are converted
  DCP  Determines if Discharge is Permitted
*********************************************************************************************************/
void LTC68041::cmdADCV(DischargeCtrl dcp, CellChannel ch) const
{

    std::uint16_t cmd = ADCV;
    cmd |= md;
    cmd |= dcp;
    cmd |= ch;

    //3
    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    //4
    spi_write_cmd(cmd);

}

/*!******************************************************************************************************
Starts cell voltage conversion with test values from selftest 2
*********************************************************************************************************/
void LTC68041::cmdCVST(SelfTestMode st) const
{

    std::uint16_t cmd = CVST;
    cmd |= md;
    cmd |= st;

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
void LTC68041::cmdADAX(AuxChannel chg) const
{
    std::uint16_t cmd = ADAX;
    cmd |= md;
    cmd |= chg;

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
void LTC68041::cmdADCVAX(DischargeCtrl dcp) const
{
    std::uint16_t cmd = ADCVAX;
    cmd |= md;
    cmd |= dcp;

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
void LTC68041::cmdADSTAT(StatusGroup chst) const
{
    std::uint16_t cmd = ADSTAT;
    cmd |= md;
    cmd |= chst;

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
void LTC68041::cmdADOW(PUPCtrl pup, DischargeCtrl dcp, CellChannel ch) const
{
    std::uint16_t cmd = ADOW;
    cmd |= md;
    cmd |= pup;
    cmd |= dcp;
    cmd |= ch;

    //wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
    spi_write_cmd(cmd);
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
    Serial.println();
    Serial.print("Cell Voltage A: ");

    for(const auto &element : regs.CVAR)
    {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial.print("Cell Voltage B: ");

    for(const auto &element : regs.CVBR)
    {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial.print("Cell Voltage C: ");

    for(const auto &element : regs.CVCR)
    {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial.print("Cell Voltage D: ");

    for(const auto &element : regs.CVDR)
    {
        Serial.print(element, HEX);
        Serial.print(" ");
    }

    Serial.println();
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
    Serial.println();
    Serial.print("Array Content | ");

    for(const auto &element : arr)
    {
        serialPrint(element);
        Serial.print("\t");
    }

    Serial.print(" |END \n");
}