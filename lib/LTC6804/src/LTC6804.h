/************************************************************

This library is based on the LTC68041.cpp by linear technology.
http://www.linear.com/product/LTC6804-1

I modified it to make it compatible with the ESP8622
https://github.com/jontubs/EasyBMS
***********************************************************/

#ifndef LTC68041_H
#define LTC68041_H


//#ifndef LTC6804_CS
//#define LTC6804_CS D8
//#endif








/*!

 |MD| Dec  | ADC Conversion Model|
 |--|------|---------------------|
 |01| 1    | Fast            |
 |10| 2    | Normal        |
 |11| 3    | Filtered          |
*/
#define MD_FAST 1
#define MD_NORMAL 2
#define MD_FILTERED 3


/*!
|CH | Dec  | Channels to convert |
|---|------|---------------------|
|000| 0    | All Cells       |
|001| 1    | Cell 1 and Cell 7   |
|010| 2    | Cell 2 and Cell 8   |
|011| 3    | Cell 3 and Cell 9   |
|100| 4    | Cell 4 and Cell 10  |
|101| 5    | Cell 5 and Cell 11  |
|110| 6    | Cell 6 and Cell 12  |
*/

#define CELL_CH_ALL 0
#define CELL_CH_1and7 1
#define CELL_CH_2and8 2
#define CELL_CH_3and9 3
#define CELL_CH_4and10 4
#define CELL_CH_5and11 5
#define CELL_CH_6and12 6



/*!

  |CHG | Dec  |Channels to convert   |
  |----|------|----------------------|
  |000 | 0    | All GPIOS and 2nd Ref|
  |001 | 1    | GPIO 1           |
  |010 | 2    | GPIO 2               |
  |011 | 3    | GPIO 3           |
  |100 | 4    | GPIO 4           |
  |101 | 5    | GPIO 5         |
  |110 | 6    | Vref2            |
*/

#define AUX_CH_ALL 0
#define AUX_CH_GPIO1 1
#define AUX_CH_GPIO2 2
#define AUX_CH_GPIO3 3
#define AUX_CH_GPIO4 4
#define AUX_CH_GPIO5 5
#define AUX_CH_VREF2 6

//uint8_t CHG = 0; //!< aux channels to be converted
/*!****************************************************
 \brief Controls if Discharging transitors are enabled
 or disabled during Cell conversions.

|DCP | Discharge Permitted During conversion |
|----|----------------------------------------|
|0   | No - discharge is not permitted         |
|1   | Yes - discharge is permitted           |

********************************************************/
#define DCP_DISABLED 0
#define DCP_ENABLED 1




class LTC68041
{
public:
    uint8_t dummy = 0x55;		//Just a dummy byte
    uint8_t SizeConfigReg = 6; //Len Conifiguration Register = 6
    uint8_t SizeStatusRegA = 6; //Len Conifiguration Register = 6
    uint8_t SizeStatusRegB = 6; //Len Conifiguration Register = 6
    uint8_t SizeReg = 6; 		//All registers have the same length
    uint8_t PEClen = 2;		//Len PEC Bytes = 2
    uint8_t CFGRw[8];		//Configuration Register Group Write
    uint8_t CFGRr[8];		//Configuration Register Group Read
    uint8_t	CVAR[8];		//Cell Voltage Register Group A
    uint8_t	CVBR[8];		//Cell Voltage Register Group B
    uint8_t	CVCR[8];		//Cell Voltage Register Group C
    uint8_t	CVDR[8];		//Cell Voltage Register Group D
    uint8_t	AVAR[8];		//Auxiliary Register Group A
    uint8_t	AVBR[8];		//Auxiliary Register Group B
    uint8_t	STAR[8];		//Status Register Group A
    uint8_t	STBR[8];		//Status Register Group B
    uint8_t REV;				//Revision Code Device Revision Code. See Revision Code and Reserved Bits in Operation Section.
    uint8_t RSVD;				//Reserved Bits See Revision Code and Reserved Bits in Operation Section.
    uint16_t cellCodes[CELLNUM];  //Raw values extracted from the voltage registers
    uint16_t AuxCodes[GPIONUM];	//Auxilury Raw Data
    uint16_t SOC;			//Sum of all cells Raw
    uint16_t ITMP;			//InternalTemperautr Raw
    uint16_t VA;			//Analog Power Supply Voltage 16-Bit ADC Measurement Value of Analog Power Supply Voltage Analog Power Supply Voltage = VA � 100�V Normal Range Is within 4.5V to 5.5V
    uint16_t VD;			//Digital Power Supply Voltage 16-Bit ADC Measurement Value of Digital Power Supply Voltage Digital Power Supply Voltage = VA � 100�V Normal Range Is within 2.7V to 3.6V
    bool CUV[CELLNUM];		//Cell x Overvoltage Flag x = 1 to 12 Cell Voltage Compared to VOV Comparison Voltage 0 -> Cell x Not Flagged for Overvoltage Condition. 1 -> Cell x Flagged
    bool COV[CELLNUM];		//Cell x Undervoltage Flag x = 1 to 12 Cell Voltage Compared to VUV Comparison Voltage 0 -> Cell x Not Flagged for Undervoltage Condition. 1 -> Cell x Flagged
    uint16_t Discharge;	//Array of bits, if 1=Discharge Active	if 0=Off
    bool MUXFAIL;			//Multiplexer Self-Test ResultRead: 0 -> Multiplexer Passed Self Test 1 -> Multiplexer Failed Self Test
    bool THSD;				//Thermal Shutdown Status Read: 0 -> Thermal Shutdown Has Not Occurred 1 -> Thermal Shutdown Has Occurred THSD Bit Cleared to 0 on Read of Status RegIster Group B

    float cellVoltage[CELLNUM];	//Cell voltage on volt
    float gpioVoltage[GPIONUM];	//Voltage on the GPIO pins
    float SumCellVoltages;	//Sum of all cell voltages
    float AnalogSupplyVoltage;	//16-Bit ADC Measurement Value of Analog Power Supply Voltage Analog Power Supply Voltage = VA � 100�V Normal Range Is within 4.5V to 5.5V
    float DigitalSupplyVoltage; //16-Bit ADC Measurement Value of Digital Power Supply Voltage Digital Power Supply Voltage = VA � 100�V Normal Range Is within 2.7V to 3.6V
    float InternalTemp;		//16-Bit ADC Measurement Value of Internal Die Temperature Temperature Measurement (�C) = ITMP � 100�V/7.5mV/�C � 273�C
    float OffsetTemp;		//Offset of temperaturemeasurement

    //Methods
    explicit LTC68041(byte pMOSI = MOSI, byte pMISO = MISO, byte pCLK = SCK, byte pCS = 10);
    void helloworld();
    void initialize();
    void wakeup_idle();
    void initCFGR();
    uint8_t rdcfg();
    void setVUV(float Undervoltage);
    void setVOV(float Overvoltage);
    void resetDischargeAll();
    bool setDischarge(int Discharge) ;
    int8_t rdcfg_debug(uint8_t r_config[8]);
    void balance_cfg(int cell, uint8_t cfg[6]);
    void wrcfg(uint8_t config[6]);
    float cell_compute_soc(float voc);
    void clraux();
    uint8_t rdcv();
    void rdcv_reg(uint8_t reg, uint8_t *data);
    void clrcell();
    void rdaux_reg(uint8_t reg, uint8_t *data);
    void checkSPI();
    bool checkSPI_mute();
    int8_t rdaux(uint8_t reg, uint16_t aux_codes[6]);
    void adcv();
    void adcv_test1();
    void adcv_test2();
    bool rdstatus_debug();
    float  rditemp_debug();
    void cnvCellVolt();
    void cnvStatus();
    void cnvAuxVolt();
    void cnvConfigRead();
    void cnvConfigWrite();
    void adax();
    float cnvITMP(float offset);
    uint8_t rdcv_debug(uint16_t cell_codes[cellNum]);
    uint8_t rdauxa();
    uint8_t rdauxb();
    uint8_t rdstata();
    uint8_t rdstatb();
    void adcvax();
    void adstat();
    void adowpu();
    void adowpd();


protected:

private:
    //internal variables
    byte index;

    byte pinCS;		//ChipSelectPin
    byte pinMOSI;	//Master Out Slave In Pin
    byte pinMISO;	//Master In Slave Out Pin
    byte pinCLK;	//Clock Pin
    static constexpr byte CELLNUM = 12;  //Number of cells checked by this Chip
    static constexpr byte GPIONUM = 5;  //Number of GPIOs
    static constexpr uint16_t crc15Table[256] = {0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
    0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
    0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
    0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
    0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
    0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
    0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
    0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
    0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
    0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
    0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
    0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
    0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
    0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
    0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
    0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
    0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
    0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
    0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
    0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
    0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
    0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
    0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095};		//static

    uint16_t pec15_calc(uint8_t len, uint8_t *data);
    void spi_write_read(uint8_t tx_Data[], uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len);
    void spi_write_array(uint8_t len, uint8_t data[]);

};

template<typename T, std::size_t N>
void printArray(std::array<T, N> &arr);
#endif
