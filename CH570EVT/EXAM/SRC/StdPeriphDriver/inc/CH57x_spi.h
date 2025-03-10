/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH57x_SPI.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH57x_SPI_H__
#define __CH57x_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  SPI interrupt bit define
 */
#define SPI_IT_FST_BYTE    RB_SPI_IE_FST_BYTE  // 从机模式的首字节命令模式下，接收到首字节中断
#define SPI_IT_FIFO_OV     RB_SPI_IE_FIFO_OV   // FIFO 溢出
#define SPI_IT_DMA_END     RB_SPI_IE_DMA_END   // DMA 传输结束
#define SPI_IT_FIFO_HF     RB_SPI_IE_FIFO_HF   // FIFO 使用过半
#define SPI_IT_BYTE_END    RB_SPI_IE_BYTE_END  // 单字节传输完成
#define SPI_IT_CNT_END     RB_SPI_IE_CNT_END   // 全部字节传输完成

/**
 * @brief  Configuration data mode
 */
typedef enum
{
    Mode0_LowBitINFront = 0, // 模式0，低位在前
    Mode0_HighBitINFront,    // 模式0，高位在前
    Mode3_LowBitINFront,     // 模式3，低位在前
    Mode3_HighBitINFront,    // 模式3，高位在前
} ModeBitOrderTypeDef;

/**
 * @brief  Configuration SPI slave mode
 */
typedef enum
{
    Mode_DataStream = 0, // 数据流模式
    Mose_FirstCmd,       // 首字节命令模式
} Slave_ModeTypeDef;

/**
 * @brief   主机模式默认初始化：模式0+3线全双工+8MHz
 */
void SPI_MasterDefInit(void);

/**
 * @brief   主机2线发送模式初始化：模式1+2线半双工+8MHz
 */
void SPI_2WIRE_MasterOutputInit(void);

/**
 * @brief   主机2线接收模式初始化：模式1+2线半双工+8MHz
 */
void SPI_2WIRE_MasterReceiveInit(void);

/**
 * @brief   从机2线接收模式初始化
 */
void SPI_2WIRE_SlaveInputInit(void);

/**
 * @brief   从机2线发送模式初始化
 */
void SPI_2WIRE_SlaveOutputInit(void);

/**
 * @brief   SPI 基准时钟配置，= d*Tsys
 *
 * @param   c       - 时钟分频系数
 */
void SPI_CLKCfg(uint8_t c);

/**
 * @brief   设置数据流模式
 *
 * @param   m       - 数据流模式 refer to ModeBitOrderTypeDef
 */
void SPI_DataMode(ModeBitOrderTypeDef m);

/**
 * @brief   发送单字节 (buffer)
 *
 * @param   d       - 发送字节
 */
void SPI_MasterSendByte(uint8_t d);

/**
 * @brief   接收单字节 (buffer)
 *
 * @param   none
 */
uint8_t SPI_MasterRecvByte(void);

/**
 * @brief   使用FIFO连续发送多字节
 *
 * @param   pbuf    - 待发送的数据内容首地址
 * @param   len     - 请求发送的数据长度，最大4095
 */
void SPI_MasterTrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   使用FIFO连续接收多字节
 *
 * @param   pbuf    - 待接收的数据首地址
 * @param   len     - 待接收的数据长度，最大4095
 */
void SPI_MasterRecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   DMA方式连续发送数据
 *
 * @param   pbuf    - 待发送数据起始地址,需要四字节对其
 * @param   len     - 待发送数据长度
 */
void SPI_MasterDMATrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   DMA方式连续接收数据
 *
 * @param   pbuf    - 待接收数据存放起始地址,需要四字节对其
 * @param   len     - 待接收数据长度
 */
void SPI_MasterDMARecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   加载首字节数据内容
 *
 * @param   d       - 首字节数据内容
 */
#define SetFirstData(d)    (R8_SPI_SLAVE_PRE = d)

/**
 * @brief   从机模式初始化
 */
void SPI_SlaveInit(void);

/**
 * @brief   从机2线模式初始化
 */
void SPI_2WIRE_SlaveInit(void);

/**
 * @brief   从机模式，发送一字节数据
 *
 * @param   d       - 待发送数据
 */
void SPI_SlaveSendByte(uint8_t d);

/**
 * @brief   从机模式，接收一字节数据
 *
 * @return  接收到数据
 */
uint8_t SPI_SlaveRecvByte(void);

/**
 * @brief   从机模式，发送多字节数据
 *
 * @param   pbuf    - 待发送的数据内容首地址
 * @param   len     - 请求发送的数据长度，最大4095
 */
void SPI_SlaveTrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   从机模式，接收多字节数据
 *
 * @param   pbuf    - 接收收数据存放起始地址
 * @param   len     - 请求接收数据长度
 */
void SPI_SlaveRecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   DMA方式连续发送数据
 *
 * @param   pbuf    - 待发送数据起始地址,需要四字节对其
 * @param   len     - 待发送数据长度
 */
void SPI_SlaveDMATrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   DMA方式连续接收数据
 *
 * @param   pbuf    - 待接收数据存放起始地址,需要四字节对其
 * @param   len     - 待接收数据长度
 */
void SPI_SlaveDMARecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   配置SPI中断
 *
 * @param   s       - 使能/关闭
 * @param   f       - refer to SPI interrupt bit define
 */
#define SPI_ITCfg(s, f)       ((s) ? (R8_SPI_INTER_EN |= f) : (R8_SPI_INTER_EN &= ~f))

/**
 * @brief   获取中断标志状态，0-未置位，(!0)-触发
 *
 * @param   f       - refer to SPI interrupt bit define
 */
#define SPI_GetITFlag(f)      (R8_SPI_INT_FLAG & f)

/**
 * @brief   清除当前中断标志
 *
 * @param   f       - refer to SPI interrupt bit define
 */
#define SPI_ClearITFlag(f)    (R8_SPI_INT_FLAG = f)

/**
 * @brief   关闭SPI
 */
#define SPI_Disable()         (R8_SPI_CTRL_MOD &= ~(RB_SPI_MOSI_OE | RB_SPI_SCK_OE | RB_SPI_MISO_OE))

#ifdef __cplusplus
}
#endif

#endif // __CH57x_SPI_H__
