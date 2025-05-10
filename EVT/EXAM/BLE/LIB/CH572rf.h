/********************************** (C) COPYRIGHT ******************************
* File Name         : CH572rf.h
* Author            : WCH
* Version           : V1.00
* Date              : 2024/11/07
* Description       : head file(ch572/ch570)
 *
 * Copyright (c) 2023 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/



/******************************************************************************/
#ifndef __CH572_RF_H
#define __CH572_RF_H

#ifdef __cplusplus
extern "C"
{
#endif
#include  "stdint.h"

#ifndef  bStatus_t
typedef uint8_t                 bStatus_t;
#endif

typedef uint32_t rfRole_States_t;
// Define function type that rfRole process callback
typedef void (*pfnRfRoleProcess)( rfRole_States_t status, uint8_t id );


/*********************************************************************
 * GLOBAL MACROS
 */
#define  VER_RF_FILE            "CH57x_RF_BASIC_LIB_V1.0"
extern const uint8_t VER_RF_LIB[];  // LIB version

/* takes a byte out of a uint32_t : var - uint32_t,  ByteNum - byte to take out (0 - 3) */
#define BREAK_UINT32( var, ByteNum ) (uint8_t)((uint32_t)(((var) >>((ByteNum) * 8)) & 0x00FF))
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)
#define HI_UINT8(a)  (((a) >> 4) & 0x0F)
#define LO_UINT8(a)  ((a) & 0x0F)
#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32_t)(((uint32_t)(Byte0) & 0x00FF) \
          + (((uint32_t)(Byte1) & 0x00FF) << 8) \
          + (((uint32_t)(Byte2) & 0x00FF) << 16) \
          + (((uint32_t)(Byte3) & 0x00FF) << 24)))
#define BUILD_UINT16(loByte, hiByte) ((uint16_t)(((loByte) & 0x00FF)|(((hiByte) & 0x00FF)<<8)))

#define ACTIVE_LOW        !
#define ACTIVE_HIGH       !!            // double negation forces result to be '1'

#ifndef BV
#define BV(n)      (1 << (n))
#endif

#ifndef BF
#define BF(x,b,s)  (((x) & (b)) >> (s))
#endif

#ifndef MIN
#define MIN(n,m)   (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
#define MAX(n,m)   (((n) < (m)) ? (m) : (n))
#endif

#ifndef ABS
#define ABS(n)     (((n) < 0) ? -(n) : (n))
#endif

/* TxPower define(Accuracy:¡À2dBm) */
#define LL_TX_POWEER_MINUS_25_DBM       0x01
#define LL_TX_POWEER_MINUS_20_DBM       0x02
#define LL_TX_POWEER_MINUS_15_DBM       0x03
#define LL_TX_POWEER_MINUS_10_DBM       0x05
#define LL_TX_POWEER_MINUS_8_DBM        0x07
#define LL_TX_POWEER_MINUS_5_DBM        0x0A
#define LL_TX_POWEER_MINUS_3_DBM        0x0C
#define LL_TX_POWEER_MINUS_1_DBM        0x10
#define LL_TX_POWEER_0_DBM              0x12
#define LL_TX_POWEER_1_DBM              0x15
#define LL_TX_POWEER_2_DBM              0x18
#define LL_TX_POWEER_3_DBM              0x1B
#define LL_TX_POWEER_4_DBM              0x1F
#define LL_TX_POWEER_5_DBM              0x25
#define LL_TX_POWEER_6_DBM              0x2D
#define LL_TX_POWEER_7_DBM              0x3B


/*********************************************************************
 * GLOBAL MACROS
 */
#define  RF_MAX_DATA_LEN              251

// defined for properties
// whitening off enable
#define  BB_WHITENING_OFF             (1<<0)
#define  BB_WHITENING_CH              (1<<2)

// PHY_MODE_TYPE  @PHY AUX TYPE(2b)
#define PHY_MODE_MASK                 (0x30)
#define  PHY_MODE_PHY_1M              (0<<4)
#define  PHY_MODE_PHY_2M              (1<<4)
#define  PHY_MODE_2G4                 (3<<4)

/* rfRole config structure definition */
typedef struct
{
    pfnRfRoleProcess rfProcessCB;
    uint32_t processMask;
#define    RF_STATE_RX             (1<<0)
#define    RF_STATE_TIMEOUT        (1<<2)
#define    RF_STATE_RX_CRCERR      (1<<3)
#define    RF_STATE_TX_FINISH      (1<<4)
} rfRoleConfig_t;


typedef union {
    struct {
        uint32_t whitOff               : 1;   //0       whitening off enable
        uint32_t resv0                 : 1;   //1
        uint32_t whitChannel           : 1;   //2       set the channel index of Data whitening enable
        uint32_t resv1                 : 1;   //3
        uint32_t modePHY               : 2;   //4-5     00-1M  01-2M  11-2.4G
        uint32_t resv2                 :10;   //6-15
        // 2.4G CONFIG
        uint32_t lengthCrc             : 2;   //16-17   00:0 bytes, 01:1 bytes, 10:2 bytes
        uint32_t ctlFiled              : 2;   //18-19   00:no control field,  01:9 bits field, 10:10 bits field
        uint32_t lengthAA              : 2;   //20-21   Access length, 00:resv, 01:4 bytes, 10:5 bytes
        uint32_t lengthPreamble        : 2;   //22-23   Preamble length, 01:1 byte, 11:3 bytes
        uint32_t dplEnable             : 1;   //24      0:nondpl, 1: dpl
        uint32_t mode2G4               : 2;   //25-26   00: 2M, 01: 1M
#define  PHY_2G4_1M   1
#define  PHY_2G4_2M   0
        uint32_t bitOrderData          : 1;   //27  0: MSB first, 1: LSB first
        uint32_t resv3                 : 1;
        uint32_t resv4                 : 1;
        uint32_t crcXOREnable          : 1;   //30  the CRC result inverted enable
        uint32_t resv5                 : 1;
    };
    uint32_t cfgVal;
} TPROPERTIES_CFG;

/* rfip tx parameter */
typedef struct
{
    uint32_t accessAddress;       //!< access address,32bit PHY address
    uint8_t accessAddressEx;
    uint32_t crcInit;             //!< crc initial value
    uint32_t frequency;           //!< rf frequency (2400000kHz-2483500kHz)
    uint32_t properties;          //!< bit0: 0-whitening on 1-whitening off
                                  //!< bit2: 1-set the channel index of Data whitening
                                  //!< BIT[5:4]    MODE_PHY        2b'00-1M  01-2M  11-2.4G
                                  //!< BIT[10:8]   MODE_INDEX      2b'00:0.5  01:0.375  10:0.3125  11:0.25
                                  //!< BIT[17:16]  CRC_LEN         2b'00:0 bytes, 01:1 bytes, 10:2 bytes
                                  //!< BIT[19:18]  CTL_FILED       2b'00:no control field,  01:9 bits field, 10:10 bits field
                                  //!< BIT[21:20]  AA_LEN          2b'00:resv, 01:4 bytes, 10:5 bytes
                                  //!< BIT[23:22]  PRE_LEN         2b'01:1 byte, 11:3 bytes
                                  //!< BIT24       DPL_EN          1b'0:nondpl, 1: dpl
                                  //!< BIT[26:25]  MODE_2G4        2b'00:2M, 01: 1M
                                  //!< BIT27       DATA_BIT_ORDER  1b'0: MSB first, 1: LSB first
                                  //!< BIT29       AA_CRC_EN       1b'1:access address included in the CRC
                                  //!< BIT30       CRC_XOR_EN      1b'1:the CRC result inverted
    uint32_t txDMA;               //!< Tx DMA begin address
    uint8_t whiteChannel;         //!< whitening channel
    uint8_t txLen;                //!< 2.4G nondpl mode: The length of Tx data  other mode: resv
    uint8_t waitTime;             //!< wait tx PLL lock time
    int8_t txPowerVal;            //!< Tx power value
    uint32_t crcPoly;             //!< crc poly value
} rfipTx_t;

/* rfip rx  parameter */
typedef struct
{
    uint32_t accessAddress;       //!< access address,32bit PHY address
    uint8_t accessAddressEx;
    uint32_t crcInit;             //!< crc initial value
    uint32_t frequency;           //!< rf frequency (2400000kHz-2483500kHz)
    uint32_t properties;          //!< bit0: 0-whitening on 1-whitening off
                                  //!< bit2: 1-set the channel index of Data whitening
                                  //!< BIT[5:4]    PHY_MODE        2b'00-1M  01-2M  11-2.4G
                                  //!< BIT[17:16]  CRC length      2b'00:0 bytes, 01:1 bytes, 10:2 bytes
                                  //!< BIT[19:18]  Control filed   2b'00:no control field,  01:9 bits field, 10:10 bits field
                                  //!< BIT[21:20]  Access_code     2b'00:resv, 01:4 bytes, 10:5 bytes
                                  //!< BIT[23:22]  Preamble length 2b'01:1 byte, 11:3 bytes
                                  //!< BIT24       DPL enable      1b'0:nondpl, 1: dpl
                                  //!< BIT[26:25]  2G4_MODE        2b'00:2M, 01: 1M
                                  //!< BIT27       DATA_BIT_ORDER  0: MSB first      1: LSB first
                                  //!< BIT28       HEAD_BIT_ORDER  0: MSB first      1: LSB first
                                  //!< BIT29       AA_CRC_EN       1b'1:access address included in the CRC
                                  //!< BIT30       CRC_XOR_EN      1b'1:the CRC result inverted
    uint32_t rxDMA;               //!< Rx DMA address
    uint8_t whiteChannel;         //!< white channel(properties bit2 = 1)
    uint8_t rxMaxLen;             //!< 2.4G nondpl mode: The length of Rx data  other mode: The maximum length of Rx data
    uint16_t timeOut;             //!< Rx wait timeout,0:No timeout others: N*0.5us
    uint32_t crcPoly;             //!< crc poly value
} rfipRx_t;

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 * @brief   lle interruput
 *
 * @param   None.
 *
 * @return  None.
 */
void LLE_LibIRQHandler( void );

/**
 * @brief   bb interruput
 *
 * @param   None.
 *
 * @return  None.
 */
void BB_LibIRQHandler( void );

/**
 * @brief   read crc state
 *
 * @param   None.
 *
 * @return  the value of crc state.
 */
uint8_t RFIP_ReadCrc( void );

/**
 * @brief   used to stop
 *
 * @param   None
 *
 * @return  0-success.
 */
bStatus_t RFRole_Stop( void );

/**
 * @brief   used to power off rf
 *
 * @param   None
 *
 * @return  0-success.
 */
bStatus_t RFRole_Shut(void );

/**
 * @brief   rf calibration
 *
 * @param   None 
 *
 * @return  None
 */
void RFIP_Calibration( void );

/**
 * @brief   reinitialize the rfip register after sleep wake-up
 *
 * @param   None
 *
 * @return  None
 */
void RFIP_WakeUpRegInit( void );

/**
 * @brief   Basic mode init
 *
 * @param	
 *
 * @return  0-success.
 */
bStatus_t RFRole_BasicInit( rfRoleConfig_t *pConf );

/**
 * @brief  set tx parameter and start rf
 *
 * @param  pParm - rfip tx parameter
 *
 * @return  0-success.
 */
bStatus_t RFIP_StartTx( rfipTx_t *pParm  );

/**
 * @brief   set rx parameter and start rf
 *
 * @param   pParm - rfip rx parameter
 *
 * @return  0-success.
 */
bStatus_t RFIP_SetRx( rfipRx_t *pParm );

/*
 * END @ API
 */
/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif
