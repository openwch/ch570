/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH57x_flash.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : source file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

#define FLASH_ROM_SIZE		0x0003C000				// 240KB
#define FLASH_ADDRESS		0x00000000				// flash base address
#define BOOTLD_ADDRESS		0x0003C000				// 8KB
#define CONFIG_ADDRESS		0x0003E000				// 8KB

/* System: flash ROM control register */
//#define R32_FLASH_DATA	(*((volatile uint32_t *)0x40001800))	// RO/WO, flash ROM data
//#define R32_FLASH_CONTROL	(*((volatile uint32_t *)0x40001804))	// RW, flash ROM control
//#define R8_FLASH_DATA		(*((volatile uint8_t *)0x40001804))	// RO/WO, flash ROM data buffer
//#define R8_FLASH_CTRL		(*((volatile uint8_t *)0x40001806))	// RW, flash ROM control
#define RB_FLASH_EN			0x01						// RW, flash ROM enable
#define RB_FLASH_RD_ACT		0x04						// RW, flash ROM read for action enable
#define RB_FLASH_SHIFT		0x10						// WO, shift flash ROM data, auto clear
#define RB_FLASH_PON_DLY	0x60						// RW, flash ROM power up delay time: 00=disable, 01=2x, 10=4x, 11=6x
#define RB_FLASH_ACT		0x80						// RO, flash ROM data action status
//#define R8_FLASH_CFG		(*((volatile uint8_t *)0x40001807))	// RW, flash ROM config, SAM
#define RB_FLASH_SCK_HIGH	0x03						// RW, SCK high level width: 01=2 unit, 10=3 unit, 11=4 unit
#define RB_FLASH_SCK_LOW	0x04						// RW, SCK low level width: 0=same with high level, 1=longer 1 unit than high level
#define RB_FLASH_SCS_FAST	0x08						// RW, SCS inactive width: 0=3 SCK cycle, 1=2 SCK cycle
#define RB_FLASH_SEL_ADDR	0x30						// RW, select AHB address input sample point: 00=normal, 01=slow, 1x=fast
#define RB_FLASH_SEL_RDY	0xC0						// RW, select AHB ready output poing: 00=normal, 01=slow, 10=fast, 11=faster

//#define R8_GLOB_ROM_CFG	(*((volatile uint8_t *)0x40001044))	// RWA, flash ROM configuration, SAM

#define R32_PFIC_ISR0		(*((volatile uint32_t *)0xE000E000))
#define R32_PFIC_ISR1		(*((volatile uint32_t *)0xE000E004))
#define R32_PFIC_IENR0		(*((volatile uint32_t *)0xE000E100))
#define R32_PFIC_IENR1		(*((volatile uint32_t *)0xE000E104))
#define R32_PFIC_IRER0		(*((volatile uint32_t *)0xE000E180))
#define R32_PFIC_IRER1		(*((volatile uint32_t *)0xE000E184))

register volatile uint8_t * pR8_SYS_SFR asm ("s0");
#define DEF_ADDR_SYS_SFR	0x40002000
#define pR32_FLASH_DATA		(volatile uint32_t *)(pR8_SYS_SFR-(DEF_ADDR_SYS_SFR-(uint32_t)(&R32_FLASH_DATA)))
#define pR8_FLASH_DATA		(pR8_SYS_SFR-(DEF_ADDR_SYS_SFR-(uint32_t)(&R8_FLASH_DATA)))
#define pR8_FLASH_CTRL		(pR8_SYS_SFR-(DEF_ADDR_SYS_SFR-(uint32_t)(&R8_FLASH_CTRL)))
#define IS_FLASH_ACT		( *pR8_FLASH_CTRL & RB_FLASH_ACT )
#define WAIT_FLASH_ACT		{ int8_t d; asm volatile ( ".loop%=:\n\t" "lb %0,-2042(s0)\n\t" "bltz %0,.loop%=\n\t" : "=r"(d) ); }

/* RESET_EN */
#define RESET_Enable           0x00000008
#define RESET_Disable          0xFFFFFFF7

/* LOCKUP_RST_EN */
#define UART_NO_KEY_Enable     0x00000100
#define UART_NO_KEY_Disable    0xFFFFFEFF

/* BOOT_PIN */
#define BOOT_PIN_PB22          0x00000200
#define BOOT_PIN_PB11          0xFFFFFDFF

/* FLASH_WRProt */
#define FLASH_WRProt           0xFFF003FF

/* Flash operation lock flag, default value is unlocked for compatibility */
volatile uint16_t FlashLockFlag = FLASH_UNLOCK_KEY;

/*********************************************************************
 * @fn      FlashLock
 *
 * @brief   Lock Flash operation, FLASH_EEPROM_CMD will be rejected
 *
 * @return  none
 */
void FlashLock(void)
{
    FlashLockFlag = 0;
}

/*********************************************************************
 * @fn      FlashUnlock
 *
 * @brief   Unlock Flash operation
 *
 * @return  none
 */
void FlashUnlock(void)
{
    FlashLockFlag = FLASH_UNLOCK_KEY;
}

/*********************************************************************
 * @fn      FLASH_ROM_READ
 *
 * @brief   Read Flash
 *
 * @param   StartAddr   - read address
 * @param   Buffer      - read buffer
 * @param   len         - read len
 *
 * @return  none
 */
void FLASH_ROM_READ(uint32_t StartAddr, void *Buffer, uint32_t len)
{
    uint32_t  i, Length = (len + 3) >> 2;
    uint32_t *pCode = (uint32_t *)StartAddr;
    uint32_t *pBuf = (uint32_t *)Buffer;

    for(i = 0; i < Length; i++)
    {
        *pBuf++ = *pCode++;
    }
}

/*********************************************************************
 * @fn      UserOptionByteConfig
 *
 * @brief   Configure User Option Byte.需在调用用户配置字生效函数后生效,且每次烧录后只能修改一次
 *          (使用该函数，必须使用官方提供的.S文件，同时调用该函数后，两次上电后，两线调试接口默认关闭)
 *
 * @param   RESET_EN        - 外部复位引脚使能
 * @param   UART_NO_KEY_EN  - 串口免按键下载使能
 * @param   FLASHProt_Size  - 写保护大小(单位4K)
 *
 * @return  0-Success, 1-Err
 */
uint8_t UserOptionByteConfig(FunctionalState RESET_EN, FunctionalState UART_NO_KEY_EN,
                           uint32_t FLASHProt_Size)
{
    uint32_t s, t;

    FLASH_ROM_READ(0x14, &s, 4);

    if(s == 0xF3F9BDA9)
    {
        s = 0;
        FLASH_EEPROM_CMD(CMD_GET_ROM_INFO, 0x3EFFC, &s, 4);
        s &= 0xFF;

        if(RESET_EN == ENABLE)
            s |= RESET_Enable;
        else
            s &= RESET_Disable;

        /* bit[7:0]-bit[31-24] */
        s |= ((~(s << 24)) & 0xFF000000); //高8位 配置信息取反；

        if(UART_NO_KEY_EN == ENABLE)
            s |= UART_NO_KEY_Enable;

        /* bit[23-10] */
        s &= 0xFF0003FF;
        s |= ((FLASHProt_Size << 10) | (5 << 20)) & 0x00FFFC00;

        /*Write user option byte*/
        FLASH_ROM_WRITE(0x14, &s, 4);

        /* Verify user option byte */
        FLASH_ROM_READ(0x14, &t, 4);

        if(s == t)
            return 0;
        else
            return 1;
    }

    return 1;
}

/*********************************************************************
 * @fn      UserOptionByteClose_SWD
 *
 * @brief   关两线调试接口
 *
 * @return  0-Success, 1-Err
 */
uint8_t UserOptionByteClose_SWD(void)
{
    uint32_t s, t;

    return 1;
}

/*********************************************************************
 * @fn      UserOptionByte_Active
 *
 * @brief   用户配置字生效函数，执行后自动复位
 *
 * @return  0-Success, 1-Err
 */
void UserOptionByte_Active(void)
{
    FLASH_ROM_SW_RESET();
    sys_safe_access_enable();
    R16_INT_LSI_TUNE = 0xFFFF;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    sys_safe_access_disable();
    while(1);
}

/*********************************************************************
 * @fn      GET_UNIQUE_ID
 *
 * @brief   get 64 bit unique ID
 *
 * @param   Buffer      - Pointer to the buffer where data should be stored, Must be aligned to 4 bytes.
 *
 * @return  0-SUCCESS  (!0)-FAILURE
 */
void GET_UNIQUE_ID(uint8_t *Buffer)
{
    uint16_t temp;
    FLASH_EEPROM_CMD( CMD_GET_ROM_INFO, ROM_CFG_MAC_ADDR, Buffer, 0 );
    temp = (Buffer[0]|(Buffer[1]<<8)) + (Buffer[2]|(Buffer[3]<<8)) + (Buffer[4]|(Buffer[5]<<8));
    Buffer[6] = temp&0xFF;
    Buffer[7] = (temp>>8)&0xFF;
}



/*********************************************************************
 * @fn      FLASH_ROM_BEG_FF
 *
 * @brief   Start FlashROM I/O and output one byte twice, used before
 *          the command sent from FlashROM
 *
 * @param   Dout    - data to output
 *
 * @return  none
 */
__attribute__((section(".highcode")))
VOID FLASH_ROM_BEG_FF( uint8_t Dout )
{
    *pR8_FLASH_CTRL = 0;
    *pR8_FLASH_CTRL = RB_FLASH_EN | RB_FLASH_RD_ACT;
    asm ( "c.nop" );
    asm ( "c.nop" );
    *pR8_FLASH_DATA = Dout;
    while ( IS_FLASH_ACT );
    *pR8_FLASH_DATA = Dout;
    while ( IS_FLASH_ACT );
}

/*********************************************************************
 * @fn      FLASH_ROM_BEG
 *
 * @brief   Start FlashROM I/O and output one byte
 *
 * @param   Dout    - data to output
 *
 * @return  none
 */
__attribute__((section(".highcode")))
VOID FLASH_ROM_BEG( uint8_t Dout )
{
	*pR8_FLASH_CTRL = 0;
	*pR8_FLASH_CTRL = RB_FLASH_EN | RB_FLASH_RD_ACT;
    asm ( "c.nop" );
    asm ( "c.nop" );
	*pR8_FLASH_DATA = Dout;
}

/*********************************************************************
 * @fn      FLASH_ROM_END
 *
 * @brief   End FlashROM I/O operation
 *
 * @return  none
 */
__attribute__((section(".highcode")))
VOID FLASH_ROM_END( VOID )
{
//	while ( IS_FLASH_ACT );
	WAIT_FLASH_ACT;
	*pR8_FLASH_CTRL = 0;
}

/*********************************************************************
 * @fn      FLASH_ROM_IN
 *
 * @brief   Read one byte from FlashROM
 *
 * @return  data read from FlashROM
 */
__attribute__((section(".highcode")))
uint8_t FLASH_ROM_IN( VOID )
{
//	while ( IS_FLASH_ACT );
	WAIT_FLASH_ACT;
	return( *pR8_FLASH_DATA );
}

/*********************************************************************
 * @fn      FLASH_ROM_OUT
 *
 * @brief   Output one byte to FlashROM
 *
 * @param   Dout    - data to output
 *
 * @return  none
 */
__attribute__((section(".highcode")))
VOID FLASH_ROM_OUT( uint8_t Dout )
{
//	while ( IS_FLASH_ACT );
	WAIT_FLASH_ACT;
	*pR8_FLASH_DATA = Dout;
}

/*********************************************************************
 * @fn      FLASH_ROM_ADDR
 *
 * @brief   Send command and address to FlashROM
 *
 * @param   Cmd     - FlashROM command
 * @param   Addr    - address
 *
 * @return  none
 */
__attribute__((section(".highcode")))
VOID FLASH_ROM_ADDR( uint8_t Cmd, uint32_t Addr )
{
	uint32_t	i;
	i = 3;
	if ( Cmd == 0x0B || Cmd == 0x4B ) i = 5;
	else {
		FLASH_ROM_BEG( 0x06 );	// WR_EN
		FLASH_ROM_END( );
	}
	FLASH_ROM_BEG( Cmd );
	while ( i -- ) {
		FLASH_ROM_OUT( Addr >> 16 );
		Addr <<= 8;
	}
}

/*********************************************************************
 * @fn      FLASH_ROM_WAIT
 *
 * @brief   Wait for FlashROM operation to finish
 *
 * @return  0-timeout, other-FlashROM status
 */
__attribute__((section(".highcode")))
uint32_t FLASH_ROM_WAIT( VOID )
{
	uint32_t	s;
	uint32_t	i;
	FLASH_ROM_END( );
	for ( i = 0; i < 0x80000; i ++ ) {
		FLASH_ROM_BEG( 0x05 );
		FLASH_ROM_IN( );
		s = FLASH_ROM_IN( );
		FLASH_ROM_END( );
		if ( ( s & 0x01 ) == 0x00 ) return( s | 1 );
	}
	return( 0 );
}

/*********************************************************************
 * @fn      FLASH_START
 *
 * @brief   Enable FlashROM code write/control and start FlashROM I/O
 *
 * @return  none
 */
__attribute__((section(".highcode")))
VOID FLASH_START( VOID )
{
    R8_SAFE_ACCESS_SIG = 0x57;
    R8_SAFE_ACCESS_SIG = 0xA8;
    R8_GLOB_ROM_CFG |= RB_ROM_CODE_WE | RB_ROM_CTRL_EN;
    *pR8_FLASH_CTRL = RB_FLASH_RD_ACT;  // clear
    FLASH_ROM_BEG_FF( 0xFF );   // start FlashROM I/O before command from FlashROM
    FLASH_ROM_END( );
}

/*********************************************************************
 * @fn      FLASH_CMD_ROM_WRITE
 *
 * @brief   Write data to FlashROM
 *
 * @param   StartAddr   - write start address
 * @param   Buffer      - write data buffer
 * @param   Length      - write length, unit: word(4 bytes)
 *
 * @return  0-Success, -1-Err
 */
__attribute__((section(".highcode")))
uint32_t FLASH_CMD_ROM_WRITE( uint32_t StartAddr, void * Buffer, uint32_t Length )
{
    uint32_t  s;
    uint32_t  int32_0, int32_1;
    uint8_t  *pBuf1, *pBuf2, *pBuf3;
    uint32_t *pBufx1, *pBufx2, *pBufx3, *pBufx4;
    uint32_t  msk, sz;
    uint32_t  endaddr;
    uint32_t  d0;
    uint8_t   flagword;
    int     i1, j1, j2;
    FLASH_START();
    pBufx2 = Buffer;
    while ( Length ) {
        FLASH_ROM_ADDR( 0x02, StartAddr );
        do {
            *pR32_FLASH_DATA = *pBufx2 ++;
            for ( j1 = 4; j1 != 0; j1 -- ) {
//                      while ( IS_FLASH_ACT );
                WAIT_FLASH_ACT;
                *pR8_FLASH_CTRL = RB_FLASH_EN | RB_FLASH_RD_ACT | RB_FLASH_SHIFT;
            }
            StartAddr += 4;
            if ( ( -- Length ) == 0 ) break;
        } while ( (uint8_t)StartAddr );
        s = FLASH_ROM_WAIT( );
        if ( s == 0 )
        {
            s = -1;
            return( s );
        }
    }
    FLASH_ROM_END( );
    return( 0 );
}

/*********************************************************************
 * @fn      FLASH_CMD_ROM_ERASE
 *
 * @brief   Erase FlashROM, support 64K/4K/256 byte erase
 *
 * @param   StartAddr   - erase start address
 * @param   Buffer      - unused
 * @param   Length      - erase length, unit: byte
 *
 * @return  0-Success, -1-Err
 */
__attribute__((section(".highcode")))
uint32_t FLASH_CMD_ROM_ERASE( uint32_t StartAddr, void * Buffer, uint32_t Length )
{
    uint32_t  s;
    uint32_t  int32_0, int32_1;
    uint8_t  *pBuf1, *pBuf2, *pBuf3;
    uint32_t *pBufx1, *pBufx2, *pBufx3, *pBufx4;
    uint32_t  msk, sz;
    uint32_t  endaddr;
    uint32_t  d0;
    uint8_t   flagword;
    int     i1, j1, j2;
    FLASH_START();
    sz = 0x10000;
    do {    // 64K,4K,256
        if ( ( StartAddr & ( sz - 1 ) ) == 0 ) {
            while ( Length >= sz ) {
                FLASH_ROM_ADDR( sz == 0x10000 ? 0xD8 : ( sz == 0x1000 ? 0x20 : 0x81 ), StartAddr );
                s = FLASH_ROM_WAIT( );
                if ( s == 0 ) {
                    s = -1;
                    return( s );
                }
                StartAddr += sz;
                Length -= sz;
            }
        }
        sz >>= 4;
    } while ( sz > 0x10 );

    FLASH_ROM_END( );
    return( 0 );
}

/*********************************************************************
 * @fn      FLASH_CMD_ROM_VERIFY
 *
 * @brief   Verify data in FlashROM
 *
 * @param   StartAddr   - verify start address
 * @param   Buffer      - data buffer to compare
 * @param   Length      - verify length, unit: byte
 *
 * @return  0-Success, other-verify failed length
 */
__attribute__((section(".highcode")))
uint32_t FLASH_CMD_ROM_VERIFY( uint32_t StartAddr, void * Buffer, uint32_t Length )
{
    uint32_t  s;
    uint32_t  int32_0, int32_1;
    uint8_t  *pBuf1, *pBuf2, *pBuf3;
    uint32_t *pBufx1, *pBufx2, *pBufx3;
    uint32_t  d0;
    uint8_t   flagword;
    int     i1, j1, j2;
    FLASH_START();
    FLASH_ROM_ADDR( 0x0B, StartAddr );
    pBufx3 = Buffer;
    while ( Length -- ) {
        FLASH_ROM_IN( );
        if ( ( Length & 3 ) == 0 ) {
            if ( *pR32_FLASH_DATA != *pBufx3 ++ ) break;
        }
    }
    FLASH_ROM_END( );
    return( Length );
}

/*********************************************************************
 * @fn      FLASH_CMD_ROM_PWR
 *
 * @brief   FlashROM power up/down control
 *
 * @param   ctrl    - power control command, 0xB9-power down, 0xAB-power up
 *
 * @return  none
 */
__attribute__((section(".highcode")))
VOID FLASH_CMD_ROM_PWR( uint8_t ctrl )
{
    FLASH_START();
    FLASH_ROM_BEG( ctrl );
    FLASH_ROM_END( );
}

/*********************************************************************
 * @fn      FLASH_CMD_GET_ROM_INFO
 *
 * @brief   Get information from FlashROM, such as MAC/BOOT info
 *
 * @param   StartAddr   - information address
 * @param   Buffer      - buffer to store information
 * @param   Length      - buffer length
 *
 * @return  none
 */
__attribute__((section(".highcode")))
VOID FLASH_CMD_GET_ROM_INFO( uint32_t StartAddr, void * Buffer, uint32_t Length )
{
    uint32_t *pBufx1, *pBufx4;
    uint32_t  msk, sz;
    uint32_t  endaddr;
    uint32_t  d0;
    uint8_t   flagword;
    int     i1, j1, j2;
    FLASH_START();
    FLASH_ROM_ADDR( 0x0B, StartAddr);
    pBufx1 = Buffer;
    for ( j2 = 0; j2 != 8; j2 ++ ) {
        FLASH_ROM_IN( );
        if ( j2 == 3 ) *pBufx1 = *pR32_FLASH_DATA;
    }
    d0 = *pR32_FLASH_DATA;
    if ( StartAddr & 0x2000 ) *(uint16_t *)(pBufx1+1) = (uint16_t)d0;    // MAC: 6 bytes
    else *(pBufx1+1) = d0;  // BOOT: 8 bytes
    FLASH_ROM_END( );
}

/*********************************************************************
 * @fn      FLASH_CMD_GET_UNIQUE_ID
 *
 * @brief   Get 64 bit unique ID from FlashROM
 *
 * @param   StartAddr   - unused
 * @param   Buffer      - buffer to store unique ID
 * @param   Length      - unused
 *
 * @return  none
 */
__attribute__((section(".highcode")))
VOID FLASH_CMD_GET_UNIQUE_ID( uint32_t StartAddr, void * Buffer, uint32_t Length )
{
    uint8_t  *pBuf3;
    int     i1;
    FLASH_START();
    FLASH_ROM_ADDR( 0x4B, 0x00000000 );
    pBuf3 = Buffer;
    i1 = 15;
    do {
        *( pBuf3 + ( i1 & 0x07 ) ) ^= FLASH_ROM_IN( );
    } while ( i1 -- );
    FLASH_ROM_END( );
}

/*********************************************************************
 * @fn      FLASH_CMD_ROM_SW_RESET
 *
 * @brief   FlashROM software reset
 *
 * @return  none
 */
__attribute__((section(".highcode")))
VOID FLASH_CMD_ROM_SW_RESET( VOID )
{
    FLASH_START();
    FLASH_ROM_BEG( 0x66 );  // enable software reset
    FLASH_ROM_END( );
    FLASH_ROM_BEG( 0x99 );  // software reset
    FLASH_ROM_END( );
}

/*********************************************************************
 * @fn      FLASH_EEPROM_CMD
 *
 * @brief   Execute Flash/EEPROM command, can be called from FlashROM or RAM
 *
 * @param   cmd         - command, refer to CMD_xxx definition
 * @param   StartAddr   - operation start address
 * @param   Buffer      - data buffer
 * @param   Length      - data length
 *
 * @return  0-Success, negative-Err
 */
uint32_t FLASH_EEPROM_CMD( uint8_t cmd, uint32_t StartAddr, void * Buffer, uint32_t Length )	// execute Flash/EEPROM command, caller from FlashROM or RAM
{
	uint32_t	s;
	uint32_t	int32_0, int32_1;
	uint32_t	*pBufx4;
	uint32_t	msk;
	uint32_t	endaddr;
	int		j2;
	uint32_t	saveS0;

	if ( FlashLockFlag != FLASH_UNLOCK_KEY ) return( -5 );	// Flash operation is locked
	asm volatile ( "sw %0,%1\n\t" : "=r"(pR8_SYS_SFR) : "o"(saveS0) );
//	pR8_SYS_SFR = (volatile uint8_t *)DEF_ADDR_SYS_SFR;
	asm volatile ( "li %0,0x40002000" : "=r"(pR8_SYS_SFR) );
	s = 0;
	int32_0 = R32_PFIC_ISR0;
	int32_1 = R32_PFIC_ISR1;
	R32_PFIC_IRER0 = 0xFFFFFFFF;	// disable interrupt
	R32_PFIC_IRER1 = 0xFFFFFFFF;
    if ( cmd == CMD_FLASH_ROM_ERASE || cmd == CMD_FLASH_ROM_WRITE || cmd == CMD_FLASH_ROM_VERIFY ) {
//      StartAddr &= 0x0FFFFFFC;
        StartAddr += FLASH_ADDRESS;
        endaddr = R8_GLOB_CFG_INFO & RB_BOOT_LOADER ? FLASH_ROM_SIZE : BOOTLD_ADDRESS;
        if ( StartAddr >= endaddr ) { s = -2; goto FLASH_EEPROM_exit; }  // start address
        if ( Length > endaddr - StartAddr ) { s = -2; goto FLASH_EEPROM_exit; }  // length
        if ( cmd == CMD_FLASH_ROM_WRITE ) {
            Length >>= 2;
            s =  FLASH_CMD_ROM_WRITE( StartAddr, Buffer, Length );
        }
        else if ( cmd == CMD_FLASH_ROM_ERASE ) {
            msk = 0x0FFF;   // sector erase
            Length += StartAddr & msk;
            Length += msk;
            Length &= ~ msk;
            StartAddr &= ~ msk;
            s =  FLASH_CMD_ROM_ERASE( StartAddr, Buffer, Length );
        }
        else {

            Length = FLASH_CMD_ROM_VERIFY( StartAddr, Buffer, Length );
            s = Length + 1;
//          FLASH_ROM_END( );
        }
    }
    else if ( cmd == CMD_FLASH_ROM_PWR_DOWN ) {
        FLASH_CMD_ROM_PWR(0xB9);
    }
    else if ( cmd == CMD_FLASH_ROM_PWR_UP ) {
        FLASH_CMD_ROM_PWR(0xAB);
    }
    else if ( cmd == CMD_GET_ROM_INFO ) {   // get information from FlashROM

        FLASH_CMD_GET_ROM_INFO( StartAddr, Buffer, Length );
    }
    else if ( cmd == CMD_GET_UNIQUE_ID ) {
        pBufx4 = Buffer;
        pBufx4[1] = pBufx4[0] = 0x00000000;
        FLASH_CMD_GET_UNIQUE_ID( StartAddr, Buffer, Length );

    }
    else if ( cmd == CMD_FLASH_ROM_SW_RESET ) {
        FLASH_CMD_ROM_SW_RESET();
    }
    else {  // unknown command
        if ( cmd != CMD_FLASH_ROM_START_IO ) s = -4;    // start FlashROM I/O, release 4xIO
    }

FLASH_EEPROM_exit: 
	R8_SAFE_ACCESS_SIG = 0x57;
	R8_SAFE_ACCESS_SIG = 0xA8;
//	R8_GLOB_ROM_CFG &= ~ ( RB_ROM_DATA_WE | RB_ROM_CODE_WE | RB_ROM_CTRL_EN );
	R8_GLOB_ROM_CFG &= RB_ROM_CODE_OFS;
	R32_PFIC_IENR0 = int32_0;	// enable interrupt
	R32_PFIC_IENR1 = int32_1;
	asm volatile ( "lw %0,%1\n\t" : "=r"(pR8_SYS_SFR) : "o"(saveS0) );
	return( s );
}

