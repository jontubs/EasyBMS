/************************************************************

This library is based on the LTC68041.cpp by linear technology.
http://www.linear.com/product/LTC6804-1

I modified it to make it compatible with the ESP8622
https://github.com/jontubs/EasyBMS
***********************************************************/

#ifndef LTC68041_H
#define LTC68041_H

#include <Arduino.h>
#include <SPI.h>

#include <array>
#include <bitset>

class LTC68041
{
private:

    static constexpr int DCTOPos = 4;
    static constexpr int MDPos = 7;
    static constexpr int DCPPos = 4;
    static constexpr int STPos = 5;
    static constexpr int PUPPos = 6;

public:

    /**
     * @brief Discharge timeouts in DCTO bits in CFGR5
     * 
     */
    enum DischargeTimeout : std::uint8_t {
        DISCHRG_TIMEOUT_DISABLED    = 0x0 << DCTOPos,
        DISCHRG_TIMEOUT_0MIN5       = 0x1 << DCTOPos,
        DISCHRG_TIMEOUT_1MIN        = 0x2 << DCTOPos,
        DISCHRG_TIMEOUT_2MIN        = 0x3 << DCTOPos,
        DISCHRG_TIMEOUT_3MIN        = 0x4 << DCTOPos,
        DISCHRG_TIMEOUT_4MIN        = 0x5 << DCTOPos,
        DISCHRG_TIMEOUT_5MIN        = 0x6 << DCTOPos,
        DISCHRG_TIMEOUT_10MIN       = 0x7 << DCTOPos,
        DISCHRG_TIMEOUT_15MIN       = 0x8 << DCTOPos,
        DISCHRG_TIMEOUT_20MIN       = 0x9 << DCTOPos,
        DISCHRG_TIMEOUT_30MIN       = 0xA << DCTOPos,
        DISCHRG_TIMEOUT_40MIN       = 0xB << DCTOPos,
        DISCHRG_TIMEOUT_60MIN       = 0xC << DCTOPos,
        DISCHRG_TIMEOUT_75MIN       = 0xD << DCTOPos,
        DISCHRG_TIMEOUT_90MIN       = 0xE << DCTOPos,
        DISCHRG_TIMEOUT_120MIN      = 0xF << DCTOPos,
    };

    /**
     * @brief Discharge time left values on DCTO read in CFGR5
     * 
     */
    enum DischargeTimeLeft : std::uint8_t {
        DISCHRG_TIME_LEFT_TIMEOUT_DISABLED  = 0x0 << DCTOPos,
        DISCHRG_TIME_LEFT_0MIN_TO_0MIN5     = 0x1 << DCTOPos,
        DISCHRG_TIME_LEFT_0MIN5_TO_1MIN     = 0x2 << DCTOPos,
        DISCHRG_TIME_LEFT_1MIN_TO_2MIN      = 0x3 << DCTOPos,
        DISCHRG_TIME_LEFT_2MIN_TO_3MIN      = 0x4 << DCTOPos,
        DISCHRG_TIME_LEFT_3MIN_TO_4MIN      = 0x5 << DCTOPos,
        DISCHRG_TIME_LEFT_4MIN_TO_5MIN      = 0x6 << DCTOPos,
        DISCHRG_TIME_LEFT_5MIN_TO_10MIN     = 0x7 << DCTOPos,
        DISCHRG_TIME_LEFT_10MIN_TO_15MIN    = 0x8 << DCTOPos,
        DISCHRG_TIME_LEFT_15MIN_TO_20MIN    = 0x9 << DCTOPos,
        DISCHRG_TIME_LEFT_20MIN_TO_30MIN    = 0xA << DCTOPos,
        DISCHRG_TIME_LEFT_30MIN_TO_40MIN    = 0xB << DCTOPos,
        DISCHRG_TIME_LEFT_40MIN_TO_60MIN    = 0xC << DCTOPos,
        DISCHRG_TIME_LEFT_60MIN_TO_75MIN5   = 0xD << DCTOPos,
        DISCHRG_TIME_LEFT_75MIN_TO_90MIN    = 0xE << DCTOPos,
        DISCHRG_TIME_LEFT_90MIN_TO_120MIN   = 0xF << DCTOPos,
    };

    /**
     * @brief ADC Conversion Mode
     *
     * | 27kHz Mode (Fast)           |
     * | 7kHz Mode (Normal)          |
     * | 26Hz Mode (Filtered)        |
     * | 14kHz Mode                  |
     * | 3kHz Mode                   |
     * | 2kHz Mode                   |
     */
    enum ADCFilterMode {
        FAST,
        NORMAL,
        FILTERED,
        BANDWIDTH_27KHZ = FAST,
        BANDWIDTH_7KHZ  = NORMAL,
        BANDWIDTH_26HZ  = FILTERED,
        BANDWIDTH_14KHZ,
        BANDWIDTH_3KHZ,
        BANDWIDTH_2KHZ,
    };

    /**
     * @brief Cell Channels to convert
     *
     * |CH | Dec  | Channels to convert |
     * |---|------|---------------------|
     * |000| 0    | All Cells           |
     * |001| 1    | Cell 1 and Cell 7   |
     * |010| 2    | Cell 2 and Cell 8   |
     * |011| 3    | Cell 3 and Cell 9   |
     * |100| 4    | Cell 4 and Cell 10  |
     * |101| 5    | Cell 5 and Cell 11  |
     * |110| 6    | Cell 6 and Cell 12  |
     */
    enum CellChannel : std::uint16_t {
        CH_ALL           = 0b000,
        CH_CELL_1_AND_7  = 0b001,
        CH_CELL_2_AND_8  = 0b010,
        CH_CELL_3_AND_9  = 0b011,
        CH_CELL_4_AND_10 = 0b100,
        CH_CELL_5_AND_11 = 0b101,
        CH_CELL_6_AND_12 = 0b110,
   };

    /**
     * @brief AUX Channels to convert
     *
     * |CHG | Dec  | Channels to convert  |
     * |----|------|----------------------|
     * |000 | 0    | All GPIOS and 2nd Ref|
     * |001 | 1    | GPIO 1               |
     * |010 | 2    | GPIO 2               |
     * |011 | 3    | GPIO 3               |
     * |100 | 4    | GPIO 4               |
     * |101 | 5    | GPIO 5               |
     * |110 | 6    | Vref2                |
     */
    enum AuxChannel : std::uint16_t {
        CHG_ALL   = 0b000,
        CHG_GPIO1 = 0b001,
        CHG_GPIO2 = 0b010,
        CHG_GPIO3 = 0b011,
        CHG_GPIO4 = 0b100,
        CHG_GPIO5 = 0b101,
        CHG_VREF2 = 0b110,
    };

    /**
     * @brief Status Group to select
     *
     * |CHST| Dec  |  Status Group    |
     * |----|------|------------------|
     * |000 | 0    | OC, ITMP, VA, VD |
     * |001 | 1    | SOC              |
     * |010 | 2    | ITMP             |
     * |011 | 3    | VA               |
     * |100 | 4    | VD               |
     */
    enum StatusGroup : std::uint16_t {
        CHST_ALL  = 0b000,
        CHST_SOC  = 0b001,
        CHST_ITMP = 0b010,
        CHST_VA   = 0b011,
        CHST_VD   = 0b100,
    };

    /**
     * @brief Self-Test mode selection
     *
     * |ST| Dec  | Self-Test Mode |
     * |--|------|----------------|
     * |01| 1    | Self-Test 1    |
     * |10| 2    | Self-Test 2    |
     */
    enum SelfTestMode : std::uint16_t {
        ST_SELF_TEST_1 = (0b01 << STPos),
        ST_SELF_TEST_2 = (0b10 << STPos),
    };

    /**
     * @brief Controls if Discharging transitors are enabled
     *        or disabled during Cell conversions.
     *
     * |DCP | Discharge Permitted During conversion  |
     * |----|----------------------------------------|
     * |0   | No - discharge is not permitted        |
     * |1   | Yes - discharge is permitted           |
     */
    enum DischargeCtrl : std::uint16_t {
        DCP_DISABLED = (0b0 << DCPPos),
        DCP_ENABLED  = (0b1 << DCPPos),
    };

    /**
     * @brief Pull up/Pull down selection for open wire test
     *
     * |PUP | Pull-Up/Pull-Down Current |
     * |    | for Open-Wire Conversions |
     * |----|---------------------------|
     * |0   | Pull-Down Current         |
     * |1   | Pull-Up Current           |
     */
    enum PUPCtrl : std::uint16_t {
        PUP_PULL_DOWN = (0b0 << PUPPos),
        PUP_PULL_UP   = (0b1 << PUPPos),
    };

    //Methods
    explicit LTC68041(byte pCS = 10, float tempOffset = 0.0);
    void initSPI(byte pinMOSI, byte pinMISO, byte pinCLK);
    void destroySPI();
    void wakeup_idle() const;
    bool cfgRead();
    void cfgWrite();
    void cfgSetVUV(const float Undervoltage);
    float cfgGetVUV() const;
    void cfgSetVOV(const float Overvoltage);
    float cfgGetVOV() const;
    void cfgSetDCC(std::bitset<12> dcc);
    std::bitset<12> cfgGetDCC() const;
    void cfgSetDischargeTimeout(DischargeTimeout timeout);
    DischargeTimeLeft cfgGetDischargeTimeLeft() const;
    void cfgSetRefOn(const bool value);
    bool cfgGetRefOn();
    bool cfgGetSWTENPin() const;
    void cfgSetADCMode(ADCFilterMode mode);
    ADCFilterMode cfgGetADCMode() const;

    // debug methods
    bool checkSPI(const bool dbgOut);
    void readCfgDbg();
    void readStatusDbg();
    void readAuxDbg();
    void readCellsDbg();

    template<std::size_t N>
    bool getCellVoltages(std::array<float, N> &voltages, const CellChannel ch = CH_ALL);
    float getAuxVoltage(const AuxChannel chg);
    float getStatusVoltage(const StatusGroup chst);
    bool getStatusMUXFail();
    bool getStatusThermalShutdown();
    std::bitset<12> getStatusOverVoltageFlags();
    std::bitset<12> getStatusUnderVoltageFlags();
    int getStatusRevision();

    float cellComputeSOC(float voc);

    void clrAuxRegs() const;
    void clrCellRegs() const;
    void startAuxConv(AuxChannel chg = AuxChannel::CHG_ALL) const;
    void startCellConv(DischargeCtrl dcp, CellChannel ch = CH_ALL) const;
    void startCellConvTest(SelfTestMode st) const;
    void startCellAuxConv(DischargeCtrl dcp) const;
    void startStatusConv(StatusGroup chst = StatusGroup::CHST_ALL) const;
    void startOpenWireCheck(PUPCtrl pup, DischargeCtrl dcp, CellChannel ch = CH_ALL) const;

private:

    static constexpr int CELLNUM = 12;  //Number of cells checked by this Chip
    static constexpr int AUXNUM = 6;    //Number of Auxiliary Voltages
    static constexpr int SIZEREG = 6; 	//All registers have the same length

    /**
     * @brief Position of config and status bits in corresponding registers
     * 
     */
    enum CfgBits {
        CFGR0_ADCOPT_Pos    = 0,
        CFGR0_REFON_Pos     = 2,
        CFGR0_GPIO1_Pos     = 3,
        CFGR0_GPIO2_Pos     = 4,
        CFGR0_GPIO3_Pos     = 5,
        CFGR0_GPIO4_Pos     = 6,
        CFGR0_GPIO5_Pos     = 7,
    };

    enum BitMasks : std::uint8_t {
        CFG0_ADCOPT_MSK     = 0x01,
        CFG0_SWTRD_MSK      = 0x02,
        CFG0_REFON_MSK      = 0x04,
        CFG0_GPIO1_MSK      = 0x08,
        CFG0_GPIO2_MSK      = 0x10,
        CFG0_GPIO3_MSK      = 0x20,
        CFG0_GPIO4_MSK      = 0x40,
        CFG0_GPIO5_MSK      = 0x80,
        CFG1_VUV_MSK        = 0xFF,
        CFG2_VUV_MSK        = 0x0F,
        CFG2_VOV_MSK        = 0xF0,
        CFG3_VOV_MSK        = 0xFF,

        /**
         * Configuration register 4 discharge cell bitmask.
         */
        CFG4_DCC_MSK        = 0xFF,

        /**
         * Configuration register 5 discharge cell bitmask.
         */
        CFG5_DCC_MSK        = 0x0F,

        CFG5_DCTO_MSK       = 0xF0,

        /**
         * Thermal Shutdown Status Read: 0 -> Thermal Shutdown Has Not Occurred 1 -> Thermal Shutdown Has Occurred THSD Bit Cleared to 0 on Read of Status RegIster Group B
         */
        STBR5_THSD_MSK      = 0x01,

        /**
         * Multiplexer Self-Test ResultRead: 0 -> Multiplexer Passed Self Test 1 -> Multiplexer Failed Self Test
         */
        STBR5_MUXFAIL_MSK   = 0x02,

        STBR5_REV_MSK       = 0xF0,
    };

    /**
     * @brief Register names in the different register groups with corresponding
     *        index in group array
     */
    enum RegNames {
        CFGR0 = 0,
        CFGR1 = 1,
        CFGR2 = 2,
        CFGR3 = 3,
        CFGR4 = 4,
        CFGR5 = 5,

        CVAR0 = 0,
        CVAR1 = 1,
        CVAR2 = 2,
        CVAR3 = 3,
        CVAR4 = 4,
        CVAR5 = 5,

        CVBR0 = 0,
        CVBR1 = 1,
        CVBR2 = 2,
        CVBR3 = 3,
        CVBR4 = 4,
        CVBR5 = 5,

        CVCR0 = 0,
        CVCR1 = 1,
        CVCR2 = 2,
        CVCR3 = 3,
        CVCR4 = 4,
        CVCR5 = 5,

        CVDR0 = 0,
        CVDR1 = 1,
        CVDR2 = 2,
        CVDR3 = 3,
        CVDR4 = 4,
        CVDR5 = 5,

        AVAR0 = 0,
        AVAR1 = 1,
        AVAR2 = 2,
        AVAR3 = 3,
        AVAR4 = 4,
        AVAR5 = 5,

        AVBR0 = 0,
        AVBR1 = 1,
        AVBR2 = 2,
        AVBR3 = 3,
        AVBR4 = 4,
        AVBR5 = 5,

        STAR0 = 0,
        STAR1 = 1,
        STAR2 = 2,
        STAR3 = 3,
        STAR4 = 4,
        STAR5 = 5,

        STBR0 = 0,
        STBR1 = 1,
        STBR2 = 2,
        STBR3 = 3,
        STBR4 = 4,
        STBR5 = 5,

        COMM0 = 0,
        COMM1 = 1,
        COMM2 = 2,
        COMM3 = 3,
        COMM4 = 4,
        COMM5 = 5,
    };

    /**
     * @brief Command codes, for commands that have option bits (see p. 50 in manual),
     *        these bits are here set to 0 and will be OR'ed in later before sending
     *        the command to the chips
     */
    enum Commands : std::uint16_t {
        WRCFG   = 0x0001,
        RDCFG   = 0x0002,
        RDCVA   = 0x0004,
        RDCVB   = 0x0006,
        RDCVC   = 0x0008,
        RDCVD   = 0x000A,
        RDAUXA  = 0x000C,
        RDAUXB  = 0x000E,
        RDSTATA = 0x0010,
        RDSTATB = 0x0012,
        ADCV    = 0x0260,
        ADOW    = 0x0228,
        CVST    = 0x0207,
        ADAX    = 0x0460,
        AXST    = 0x0407,
        ADSTAT  = 0x0468,
        STATST  = 0x040F,
        ADCVAX  = 0X046F,
        CLRCELL = 0x0711,
        CLRAUX  = 0x0712,
        CLRSTAT = 0x0713,
        PLADC   = 0x0714,
        DIAGN   = 0x0715,
        WRCOMM  = 0x0721,
        RDCOMM  = 0x0722,
        STCOMM  = 0x0723
    };

    /**
     * @brief ADC Conversion Mode
     *
     * |MD| Dec  |          ADC Conversion Mode                |
     * |--|------|---------------------|-----------------------|
     * |  |      | ADCOPT(CFGR0[0]) = 0| ADCOPT(CFGR0[0]) = 1  |
     * |--|------|---------------------|-----------------------|
     * |01| 1    | 27kHz Mode (Fast)   | 14kHz Mode            |
     * |10| 2    | 7kHz Mode (Normal)  | 3kHz Mode             |
     * |11| 3    | 26Hz Mode (Filtered)| 2kHz Mode             |
     */
    enum ADCMode : std::uint16_t {
        MD_FAST     = (0b01 << MDPos),
        MD_NORMAL   = (0b10 << MDPos),
        MD_FILTERED = (0b11 << MDPos),
    };

    /**
     * @brief Register map of internal register groups
     */
    struct Registers {
        std::array<std::uint8_t, SIZEREG> CFGR;     // Configuration Register Group
        std::array<std::uint8_t, SIZEREG> CVAR;	    // Cell Voltage Register Group A
        std::array<std::uint8_t, SIZEREG> CVBR;	    // Cell Voltage Register Group B
        std::array<std::uint8_t, SIZEREG> CVCR;	    // Cell Voltage Register Group C
        std::array<std::uint8_t, SIZEREG> CVDR;	    // Cell Voltage Register Group D
        std::array<std::uint8_t, SIZEREG> AVAR;	    // Auxiliary Register Group A
        std::array<std::uint8_t, SIZEREG> AVBR;	    // Auxiliary Register Group B
        std::array<std::uint8_t, SIZEREG> STAR;	    // Status Register Group A
        std::array<std::uint8_t, SIZEREG> STBR;	    // Status Register Group B
        std::array<std::uint8_t, SIZEREG> COMM;	    // COMM Register Group
    };
    
    float offsetTemp;		                //Offset of temperaturemeasurement

    ADCMode md;
    byte pinCS;		//ChipSelectPin
    Registers regs;

    static constexpr std::uint16_t crc15Table[256] = {
        0x0000, 0xc599, 0xceab, 0x0b32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
        0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
        0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
        0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
        0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
        0x2544, 0x02be, 0xc727, 0xcc15, 0x098c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
        0x3d6e, 0xf8f7, 0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x07c2, 0xc25b, 0xc969, 0x0cf0, 0xdf0d,
        0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
        0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
        0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
        0x4a88, 0x8f11, 0x057c, 0xc0e5, 0xcbd7, 0x0e4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
        0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
        0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
        0x85e9, 0x0f84, 0xca1d, 0xc12f, 0x04b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a,
        0x3528, 0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2,
        0xe46b, 0xef59, 0x2ac0, 0x0d3a, 0xc8a3, 0xc391, 0x0608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7,
        0x54aa, 0x9133, 0x9a01, 0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06,
        0xab9f, 0x7862, 0xbdfb, 0xb6c9, 0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80,
        0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a, 0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41,
        0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25, 0x2fbc, 0x0846, 0xcddf, 0xc6ed, 0x0374,
        0xd089, 0x1510, 0x1e22, 0xdbbb, 0x0af8, 0xcf61, 0xc453, 0x01ca, 0xd237, 0x17ae, 0x1c9c,
        0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b, 0x2d02, 0xa76f, 0x62f6,
        0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3, 0x585a, 0x8ba7,
        0x4e3e, 0x450c, 0x8095
    };

    template<std::size_t N>
    constexpr void parseVoltages(const unsigned int group, const std::array<std::uint8_t, SIZEREG> &regGroup, std::array<float, N> &data);

    static constexpr float parseVoltage(const std::array<std::uint8_t, SIZEREG> &regGroup, RegNames index);

    constexpr std::uint16_t calcPEC15(const std::uint16_t data) const;

    template<std::size_t N>
    constexpr std::uint16_t calcPEC15(const std::array<std::uint8_t, N> &data) const;

    template<std::size_t N>
    bool spi_read_cmd(const std::uint16_t cmd, std::array<std::uint8_t, N> &rx_data);

    void spi_write_cmd(const std::uint16_t cmd) const;

};

template<typename T, std::size_t N>
void printArray(std::array<T, N> &arr);
#endif
