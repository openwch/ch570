/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"
#include "usb_uart.h"
#include "rf_uart_rx.h"

#define THIS_ENDP0_SIZE         64
#define MAX_PACKET_SIZE         64

#define USB_CDC_MODE      0
#define USB_VENDOR_MODE   1

/* USB工作模式 */
#define  USB_WORK_MODE     USB_VENDOR_MODE



/* CDC相关参数 */
/* 设备描述符 */
const uint8_t TAB_USB_CDC_DEV_DES[18] =
{
  0x12,
  0x01,
  0x10,
  0x01,
  0x02,
  0x00,
  0x00,
  0x40,
  0x86, 0x1a,
  0x40, 0x80,
  0x00, 0x30,
  0x01,                 //制造者的字符串描述符的索引值
  0x02,                 //产品的字符串描述符的索引值
  0x03,                 //序号的字符串描述符的索引值
  0x01                  //可能配置的数目
};


/* 配置描述符 */
const uint8_t TAB_USB_CDC_CFG_DES[ ] =
{
  0x09,0x02,0x43,0x00,0x02,0x01,0x00,0x80,0x30,

  //以下为接口0（CDC接口）描述符
  0x09, 0x04,0x00,0x00,0x01,0x02,0x02,0x01,0x00,

  0x05,0x24,0x00,0x10,0x01,
  0x04,0x24,0x02,0x02,
  0x05,0x24,0x06,0x00,0x01,
  0x05,0x24,0x01,0x01,0x00,

  0x07,0x05,0x84,0x03,0x08,0x00,0x01,                       //中断上传端点描述符

  //以下为接口1（数据接口）描述符
  0x09,0x04,0x01,0x00,0x02,0x0a,0x00,0x00,0x00,

  0x07,0x05,0x01,0x02,0x40,0x00,0x00,                       //端点描述符
  0x07,0x05,0x81,0x02,0x40,0x00,0x00,                       //端点描述符
};

/* 设备限定描述符 */
const uint8_t My_QueDescr[ ] = { 0x0A, 0x06, 0x00, 0x02, 0xFF, 0x00, 0xFF, 0x40, 0x01, 0x00 };

uint8_t TAB_CDC_LINE_CODING[ ]  =
{
  0x85, /* baud rate*/
  0x20,
  0x00,
  0x00,
  0x00,   /* stop bits-1*/
  0x00,   /* parity - none*/
  0x08    /* no. of bits 8*/
};


#define DEF_IC_PRG_VER                 0x31
/* 7523 */
/* 设备描述符 *///0x00, DEF_IC_PRG_VER, //BCD 设备版本号
const uint8_t TAB_USB_VEN_DEV_DES[] =
{
  0x12,
  0x01,
  0x10, 0x01,
  0xff, 0x00, 0x02, 0x40,//0x40,
  0x86, 0x1a, 0x23, 0x75,
  0x00, DEF_IC_PRG_VER,

  0x01,                 //制造者的字符串描述符的索引值
  0x02,                 //产品的字符串描述符的索引值
  0x03,                 //序号的字符串描述符的索引值

  0x01
};

const uint8_t TAB_USB_VEN_CFG_DES[39] =
{
  0x09, 0x02, 0x27, 0x00, 0x01, 0x01, 0x00, 0x80, 0x30,   //配置描述符
  0x09, 0x04, 0x00, 0x00, 0x03, 0xff, 0x01, 0x02, 0x00,   //接口
  0x07, 0x05, 0x82, 0x02, 0x20, 0x00, 0x00,               //端点 IN2   批量
  0x07, 0x05, 0x02, 0x02, 0x20, 0x00, 0x00,               //端点 OUT2  批量
  0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x01        //端点 IN1   中断
};

// 语言描述符
const uint8_t TAB_USB_LID_STR_DES[ ] = { 0x04, 0x03, 0x09, 0x04 };
// 厂家信息
const uint8_t TAB_USB_VEN_STR_DES[ ] = { 0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0 };
const uint8_t TAB_USB_PRD_STR_DES[ ] = {
  0x1e,0x03,0x55,0x00,0x53,0x00,0x42,0x00,0x20,0x00,0x43,0x00,0x44,0x00,0x43,0x00,
  0x2d,0x00,0x53,0x00,0x65,0x00,0x72,0x00,0x69,0x00,0x61,0x00,0x6c,0x00
};


const uint8_t USB_DEV_PARA_CDC_SERIAL_STR[]=     "WCH121212TS1";
const uint8_t USB_DEV_PARA_CDC_PRODUCT_STR[]=    "USB2.0 To Serial Port";
const uint8_t USB_DEV_PARA_CDC_MANUFACTURE_STR[]= "wch.cn";

const uint8_t USB_DEV_PARA_VEN_SERIAL_STR[]=     "WCH454545TS2";
const uint8_t USB_DEV_PARA_VEN_PRODUCT_STR[]=   "USB2.0 To Serial Port";
const uint8_t USB_DEV_PARA_VEN_MANUFACTURE_STR[]= "wch.cn";


typedef struct DevInfo
{
  uint8_t UsbConfig;      // USB配置标志
  uint8_t UsbAddress;     // USB设备地址
  uint8_t gSetupReq;        /* USB控制传输命令码 */
  uint8_t gSetupLen;          /* USB控制传输传输长度 */
  uint8_t gUsbInterCfg;       /* USB设备接口配置 */
  uint8_t gUsbFlag;       /* USB设备各种操作标志,位0=总线复位,位1=获取设备描述符,位2=设置地址,位3=获取配置描述符,位4=设置配置 */
}DevInfo_Parm;

/* 设备信息 */
DevInfo_Parm  devinf;
uint8_t SetupReqCode, SetupLen;

/* 端点硬件和软件操作的缓存区 */
__aligned(4) uint8_t  Ep0Buffer[MAX_PACKET_SIZE];     //端点0 收发共用  端点4 OUT & IN

//端点4的上传地址
__aligned(4) uint8_t  Ep1Buffer[MAX_PACKET_SIZE];     //IN
__aligned(4) uint8_t  Ep2Buffer[2*MAX_PACKET_SIZE];   //OUT & IN
__aligned(4) uint8_t  Ep3Buffer[2*MAX_PACKET_SIZE];   //OUT & IN


/* 两个串口相关数据 */
LINE_CODE Uart0Para;

#define CH341_REG_NUM     10
uint8_t CH341_Reg_Add[CH341_REG_NUM];
uint8_t CH341_Reg_val[CH341_REG_NUM];


/* 厂商模式下串口参数变化 */
uint8_t VENSer0ParaChange = 0;

/* 厂商模式下串口发送数据标志 */
uint8_t VENSer0SendFlag = 0;

/* 厂商模式下modem信号检测 */
uint8_t UART0_RTS_Val = 0; //输出 表示DTE请求DCE发送数据
uint8_t UART0_DTR_Val = 0; //输出 数据终端就绪
uint8_t UART0_OUT_Val = 0; //自定义modem信号（CH340手册）

uint8_t UART_Status;

uint8_t UART0_DCD_Val = 0;
uint8_t UART0_DCD_Change = 0;

uint8_t UART0_RI_Val = 0;
uint8_t UART0_RI_Change = 0;

uint8_t UART0_DSR_Val = 0;
uint8_t UART0_DSR_Change = 0;

uint8_t UART0_CTS_Val = 0;
uint8_t UART0_CTS_Change = 0;

/* CDC设置的串口 */
uint8_t CDCSetSerIdx = 0;
uint8_t CDCSer0ParaChange = 0;

typedef struct _USB_SETUP_REQ_ {
    uint8_t bRequestType;
    uint8_t bRequest;
    uint8_t wValueL;
    uint8_t wValueH;
    uint8_t wIndexL;
    uint8_t wIndexH;
    uint8_t wLengthL;
    uint8_t wLengthH;
} USB_SETUP_REQ_t;

#define UsbSetupBuf     ((USB_SETUP_REQ_t *)Ep0Buffer) //USB_SETUP_REQ_t USB_SETUP_REQ_t

uint32_t gUartRxCount;


/* USB缓存定义，所有端点全部定义 */
/* 端点1 -- IN状态 */
uint8_t Ep1DataINFlag = 0;

/* 端点1下传数据 */
uint8_t Ep1DataOUTFlag = 0;
uint8_t Ep1DataOUTLen = 0;
__aligned(4) uint8_t Ep1OUTDataBuf[MAX_PACKET_SIZE];


/* 端点2下传数据 */
uint8_t Ep2DataOUTFlag = 0;
uint8_t Ep2DataOUTLen = 0;
__aligned(4) uint8_t Ep2OUTDataBuf[MAX_PACKET_SIZE];

/* 保存USB中断的状态 ->改成几组的操作方式 */
#define USB_IRQ_FLAG_NUM     4

uint8_t usb_irq_w_idx = 0;
uint8_t usb_irq_r_idx = 0;

volatile uint8_t usb_irq_len[USB_IRQ_FLAG_NUM];
volatile uint8_t usb_irq_pid[USB_IRQ_FLAG_NUM];
volatile uint8_t usb_irq_flag[USB_IRQ_FLAG_NUM];

uint8_t cdc_uart_sta_trans_step = 0;
uint8_t ven_ep1_trans_step = 0;

/* 端点0枚举上传帧处理 */
uint8_t ep0_send_buf[256];


/**********************************************************/
uint8_t DevConfig;
UINT16 SetupReqLen;
const uint8_t *pDescr;


/* Character Size */
#define HAL_UART_5_BITS_PER_CHAR             5
#define HAL_UART_6_BITS_PER_CHAR             6
#define HAL_UART_7_BITS_PER_CHAR             7
#define HAL_UART_8_BITS_PER_CHAR             8

/* Stop Bits */
#define HAL_UART_ONE_STOP_BIT                1
#define HAL_UART_TWO_STOP_BITS               2

/* Parity settings */
#define HAL_UART_NO_PARITY                   0x00                              //无校验
#define HAL_UART_ODD_PARITY                  0x01                              //奇校验
#define HAL_UART_EVEN_PARITY                 0x02                              //偶校验
#define HAL_UART_MARK_PARITY                 0x03                              //置1 mark
#define HAL_UART_SPACE_PARITY                0x04                              //空白位 space


/* 端点状态设置函数 */
void USBDevEPnINSetStatus(uint8_t ep_num, uint8_t type, uint8_t sta);

/*******************************************************************************
* Function Name  : CH341RegWrite
* Description    : 写入 CH341的寄存器
* Input          : reg_add：写入寄存器地址
                   reg_val：写入寄存器的值
* Output         : None
* Return         : None
*******************************************************************************/
void CH341RegWrite(uint8_t reg_add,uint8_t reg_val)
{
  uint8_t find_idx;
  uint8_t find_flag;
  uint8_t i;

  find_flag = 0;
  find_idx = 0;
  for(i=0; i<CH341_REG_NUM; i++)
  {
    if(CH341_Reg_Add[i] == reg_add)
    {
      find_flag = 1;
      break;
    }
    if(CH341_Reg_Add[i] == 0xff)
    {
      find_flag = 0;
      break;
    }
  }
  find_idx = i;
  if(find_flag)
  {
    CH341_Reg_val[find_idx] = reg_val;
  }
  else
  {
    CH341_Reg_Add[find_idx] = reg_add;
    CH341_Reg_val[find_idx] = reg_val;
  }

  switch(reg_add)
  {
    case 0x06:break; //IO
    case 0x07:break; //IO
    case 0x18: //SFR_UART_CTRL -->串口的参数寄存器
    {
      uint8_t reg_uart_ctrl;
      uint8_t data_bit_val;
      uint8_t stop_bit_val;
      uint8_t parity_val;
      uint8_t break_en;

      reg_uart_ctrl = reg_val;
      /* break位 */
      break_en = (reg_uart_ctrl & 0x40)?(0):(1);
//      SetUART0BreakENStatus(break_en);

      data_bit_val = reg_uart_ctrl & 0x03;
      if   (data_bit_val == 0x00)   data_bit_val = HAL_UART_5_BITS_PER_CHAR;
      else if(data_bit_val == 0x01) data_bit_val = HAL_UART_6_BITS_PER_CHAR;
      else if(data_bit_val == 0x02) data_bit_val = HAL_UART_7_BITS_PER_CHAR;
      else if(data_bit_val == 0x03) data_bit_val = HAL_UART_8_BITS_PER_CHAR;

      stop_bit_val = reg_uart_ctrl & 0x04;
      if(stop_bit_val) stop_bit_val = HAL_UART_TWO_STOP_BITS;
      else             stop_bit_val = HAL_UART_ONE_STOP_BIT;

      parity_val = reg_uart_ctrl & (0x38);
      if(parity_val == 0x00)      parity_val = HAL_UART_NO_PARITY;
      else if(parity_val == 0x08) parity_val = HAL_UART_ODD_PARITY;
      else if(parity_val == 0x18) parity_val = HAL_UART_EVEN_PARITY;
      else if(parity_val == 0x28) parity_val = HAL_UART_MARK_PARITY;
      else if(parity_val == 0x38) parity_val = HAL_UART_SPACE_PARITY;

      //Uart0Para.BaudRate;
      Uart0Para.StopBits = stop_bit_val;
      Uart0Para.ParityType = parity_val;
      Uart0Para.DataBits = data_bit_val;

      PRINT("CH341 set para:%d %d %d break:%02x\r\n",data_bit_val,(int)stop_bit_val,parity_val,break_en);

      //直接设置寄存器
      VENSer0ParaChange = 1;
      break;
    }
    case 0x25: break;
    case 0x27:
    {
      PRINT("modem set:%02x\r\n",reg_val);
//      SetUART0ModemVendorSta(reg_val);
      break;
    }
  }
}

/*******************************************************************************
* Function Name  : CH341RegRead
* Description    : 读取 CH341的寄存器
* Input          : reg_add：读取的寄存器地址
                   reg_val：读取的寄存器的值存放指针
* Output         : None
* Return         : 寄存器存在
*******************************************************************************/
uint8_t CH341RegRead(uint8_t reg_add,uint8_t *reg_val)
{
  uint8_t find_flag;
  uint8_t i;

  find_flag = 0;
  *reg_val = 0;
  for(i=0; i<CH341_REG_NUM; i++)
  {
    if(CH341_Reg_Add[i] == reg_add)   //找到相同地址的寄存器
    {
      find_flag = 1;
      *reg_val = CH341_Reg_val[i];
      break;
    }
    if(CH341_Reg_Add[i] == 0xff)      //找到当前第一个空
    {
      find_flag = 0;
      *reg_val = 0x00;
      break;
    }
  }

  switch(reg_add)
  {
    case 0x06:
    {
      uint8_t  reg_pb_val = 0;
      *reg_val = reg_pb_val;
      break;
    }
    case 0x07:
    {
      uint8_t  reg_pc_val = 0;
      *reg_val = reg_pc_val;
      break;
    }
    case 0x18:   //SFR_UART_CTRL -->串口的参数寄存器
    {
      uint8_t  reg_uart_ctrl_val;
      uint8_t  ram_uart_ctrl_val;

      reg_uart_ctrl_val = R8_UART_LCR;
      //保留break位
      ram_uart_ctrl_val = *reg_val;
      reg_uart_ctrl_val |= (ram_uart_ctrl_val & 0x40);
      *reg_val = reg_uart_ctrl_val;

      break;
    }
    case 0x25:  break;
  }

  return find_flag;
}

/* endpoints enumeration */
#define ENDP0                           0x00
#define ENDP1                           0x01
#define ENDP2                           0x02
#define ENDP3                           0x03
#define ENDP4                           0x04

/* ENDP x Type */
#define ENDP_TYPE_IN                    0x00                                    /* ENDP is IN Type */
#define ENDP_TYPE_OUT                   0x01                                    /* ENDP is OUT Type */

/* 端点应答状态定义 */
/* OUT */
#define OUT_ACK                         0
#define OUT_TIMOUT                      1
#define OUT_NAK                         2
#define OUT_STALL                       3
/* IN */
#define IN_ACK                          0
#define IN_NORSP                        1
#define IN_NAK                          2
#define IN_STALL                        3

/* USB设备各种标志位定义 */
#define DEF_BIT_USB_RESET               0x01                                    /* 总线复位标志 */
#define DEF_BIT_USB_DEV_DESC            0x02                                    /* 获取过设备描述符标志 */
#define DEF_BIT_USB_ADDRESS             0x04                                    /* 设置过地址标志 */
#define DEF_BIT_USB_CFG_DESC            0x08                                    /* 获取过配置描述符标志 */
#define DEF_BIT_USB_SET_CFG             0x10                                    /* 设置过配置值标志 */
#define DEF_BIT_USB_WAKE                0x20                                    /* USB唤醒标志 */
#define DEF_BIT_USB_SUPD                0x40                                    /* USB总线挂起标志 */
#define DEF_BIT_USB_HS                  0x80                                    /* USB高速、全速标志 */


/* CH341相关的命令帧 */
#define DEF_VEN_DEBUG_READ              0X95         /* 读两组寄存器 */
#define DEF_VEN_DEBUG_WRITE             0X9A         /* 写两组寄存器 */
#define DEF_VEN_UART_INIT               0XA1         /* 初始化串口 */
#define DEF_VEN_UART_M_OUT              0XA4         /* 设置MODEM信号输出 */
#define DEF_VEN_BUF_CLEAR               0XB2         /* 清除未完成的数据 */
#define DEF_VEN_I2C_CMD_X               0X54         /* 发出I2C接口的命令,立即执行 */
#define DEF_VEN_DELAY_MS                0X5E         /* 以亳秒为单位延时指定时间 */
#define DEF_VEN_GET_VER                 0X5F         /* 获取芯片版本 */

/* 类请求 */
//  3.1 Requests---Abstract Control Model
#define DEF_SEND_ENCAPSULATED_COMMAND  0x00
#define DEF_GET_ENCAPSULATED_RESPONSE  0x01
#define DEF_SET_COMM_FEATURE           0x02
#define DEF_GET_COMM_FEATURE           0x03
#define DEF_CLEAR_COMM_FEATURE         0x04
#define DEF_SET_LINE_CODING          0x20   // Configures DTE rate, stop-bits, parity, and number-of-character
#define DEF_GET_LINE_CODING          0x21   // This request allows the host to find out the currently configured line coding.
//#define DEF_SET_CTL_LINE_STE         0X22   // This request generates RS-232/V.24 style control signals.
#define DEF_SET_CONTROL_LINE_STATE     0x22
#define DEF_SEND_BREAK                 0x23


//  3.2 Notifications---Abstract Control Model
#define DEF_NETWORK_CONNECTION         0x00
#define DEF_RESPONSE_AVAILABLE         0x01
#define DEF_SERIAL_STATE               0x20
/* 中断处理函数 */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void USB_IRQHandler(void)
{
  uint8_t   i;
  uint8_t   j;

  if(R8_USB_INT_FG & RB_UIF_TRANSFER)
  {
    /* 除setup包处理 */
    if((R8_USB_INT_ST & MASK_UIS_TOKEN) != MASK_UIS_TOKEN){     // 非空闲
      /* 直接写入 */
      usb_irq_flag[usb_irq_w_idx] = 1;
      usb_irq_pid[usb_irq_w_idx]  = R8_USB_INT_ST;  //& 0x3f;//(0x30 | 0x0F);
      usb_irq_len[usb_irq_w_idx]  = R8_USB_RX_LEN;

      switch(usb_irq_pid[usb_irq_w_idx]& 0x3f){   //分析当前的端点
        case UIS_TOKEN_OUT | 2:
        {
          if( R8_USB_INT_FG & RB_U_TOG_OK ){   //不同步的数据包将丢弃
            R8_UEP2_CTRL ^=  RB_UEP_R_TOG;
            R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK; //OUT_NAK
            /* 保存数据 */
            for(j=0; j<(MAX_PACKET_SIZE/4); j++)
            {
                ((UINT32 *)Ep2OUTDataBuf)[j] = ((UINT32 *)Ep2Buffer)[j];
            }
          }
          else
          {
              usb_irq_flag[usb_irq_w_idx] = 0;
          }
        }break;
        case UIS_TOKEN_IN | 2:{ //endpoint 2# 批量端点上传完成
          R8_UEP2_CTRL ^=  RB_UEP_T_TOG;
          R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | IN_NAK; //IN_NAK
        }break;
        case UIS_TOKEN_OUT | 1:{
          if( R8_USB_INT_FG & RB_U_TOG_OK ){   //不同步的数据包将丢弃
            R8_UEP1_CTRL ^=  RB_UEP_R_TOG;
            R8_UEP1_CTRL = (R8_UEP1_CTRL & 0xf3) | 0x08; //OUT_NAK
            /* 保存数据 */
            for(j=0; j<(MAX_PACKET_SIZE/4); j++)
              ((UINT32 *)Ep1OUTDataBuf)[j] = ((UINT32 *)Ep1Buffer)[j];
          }
          else usb_irq_flag[usb_irq_w_idx] = 0;
        }break;
        case UIS_TOKEN_IN | 1:{  //endpoint 1# 批量端点上传完成
          R8_UEP1_CTRL ^=  RB_UEP_T_TOG;
          R8_UEP1_CTRL = (R8_UEP1_CTRL & 0xfc) | IN_NAK; //IN_NAK
        }break;
        case UIS_TOKEN_OUT | 0:{    // endpoint 0
          if( R8_USB_INT_FG & RB_U_TOG_OK )   //不同步的数据包将丢弃
            R8_UEP0_CTRL = (R8_UEP0_CTRL & 0xf3) | 0x08; //OUT_NAK
          else usb_irq_flag[usb_irq_w_idx] = 0;
        }break;
        case UIS_TOKEN_IN | 0:{  //endpoint 0
          R8_UEP0_CTRL = (R8_UEP0_CTRL & 0xfc) | IN_NAK; //IN_NAK
        }break;
      }

      /* 切到下一个写地址 */
      if(usb_irq_flag[usb_irq_w_idx]){
        usb_irq_w_idx++;
        if(usb_irq_w_idx >= USB_IRQ_FLAG_NUM) usb_irq_w_idx = 0;
      }

      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }

    /* setup包处理 */
    if(R8_USB_INT_ST & RB_UIS_SETUP_ACT)
    {
      /* 与之前的处理接轨 -- UIS_TOKEN_SETUP */
      /* 直接写入 */
      usb_irq_flag[usb_irq_w_idx] = 1;
      usb_irq_pid[usb_irq_w_idx]  = UIS_TOKEN_SETUP | 0;  //保留之前的标志
      usb_irq_len[usb_irq_w_idx]  = 8;
      /* 切到下一个写地址 */
      usb_irq_w_idx++;
      if(usb_irq_w_idx >= USB_IRQ_FLAG_NUM) usb_irq_w_idx = 0;

      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }
  }
  else if ( R8_USB_INT_FG & RB_UIF_BUS_RST )  // USB总线复位
  {
#if ( USB_WORK_MODE== USB_VENDOR_MODE)
    {
      R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
      R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
      R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    }
#else
    {
      R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
      R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
      R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
      R8_UEP3_CTRL = UEP_T_RES_NAK;
      R8_UEP4_CTRL = UEP_T_RES_NAK;
    }
#endif
    cdc_uart_sta_trans_step = 0;
    ven_ep1_trans_step = 0;

    R8_USB_DEV_AD = 0x00;
    devinf.UsbAddress = 0;

    R8_USB_INT_FG = RB_UIF_BUS_RST;             // 清中断标志
  }
  else if (  R8_USB_INT_FG & RB_UIF_SUSPEND )  // USB总线挂起/唤醒完成
  {
    if ( R8_USB_MIS_ST & RB_UMS_SUSPEND )    //挂起
    {
#if ( USB_WORK_MODE== USB_VENDOR_MODE)
      {
        VENSer0ParaChange = 1;
      }
#else
      {
        CDCSer0ParaChange = 1;
      }
#endif
      Ep1DataINFlag = 0;
    }
    else                                     //唤醒
    {
      Ep1DataINFlag = 1;
    }

    cdc_uart_sta_trans_step = 0;
    ven_ep1_trans_step = 0;

    R8_USB_INT_FG = RB_UIF_SUSPEND;
  }
}

void USB_IRQProcessHandler( void )   /* USB中断服务程序 */
{
    static  PUINT8  pDescr;
    static  uint8_t irq_idx = 0;
    uint8_t len;
    uint8_t   buf[8];
    uint8_t   data_dir = 0;   //数据方向
    uint8_t   ep_idx, ep_pid;
    uint8_t   i;
    uint8_t   ep_sta;
    uint32_t  bps;

    {
        i = usb_irq_r_idx;
      usb_irq_r_idx++;
      if(usb_irq_r_idx >= USB_IRQ_FLAG_NUM) usb_irq_r_idx = 0;
      switch ( usb_irq_pid[i] & 0x3f )   // 分析操作令牌和端点号
      {
        case UIS_TOKEN_IN | 4:  //endpoint 4# 批量端点上传完成
        {
          break;
        }
        case UIS_TOKEN_IN | 3:  //endpoint 3# 批量端点上传完成
        {
          break;
        }
        case UIS_TOKEN_OUT | 2:    // endpoint 2# 批量端点下传完成
        {
            Ep2DataOUTLen = usb_irq_len[i];
            gUartRxCount += Ep2DataOUTLen;
            Ep2DataOUTFlag = 1;
            break;
        }
        case UIS_TOKEN_IN | 2:  //endpoint 2# 批量端点上传完成
        {
          break;
        }
        case UIS_TOKEN_OUT | 1:    // endpoint 1# 批量端点下传完成
        {
          PRINT("usb_out1\n");
          len = usb_irq_len[i];
          //Ep1OUTDataBuf
          for(int i = 0;i<len;i++)
          PRINT("%02x  ",Ep1OUTDataBuf[i]);
          PRINT("\n");

          //CH341的数据下发
          Ep1DataOUTFlag = 1;
          Ep1DataOUTLen = len;
          VENSer0SendFlag = 1;
          PFIC_DisableIRQ(USB_IRQn);
          R8_UEP1_CTRL = R8_UEP1_CTRL & 0xf3; //OUT_ACK
          PFIC_EnableIRQ(USB_IRQn);
          break;
        }
        case UIS_TOKEN_IN | 1:   // endpoint 1# 中断端点上传完成
        {
          Ep1DataINFlag = 1;
          break;
        }
        case UIS_TOKEN_SETUP | 0:    // endpoint 0# SETUP
        {
          len = usb_irq_len[i];
          if(len == sizeof(USB_SETUP_REQ))
          {
            SetupLen = UsbSetupBuf->wLengthL;
            if(UsbSetupBuf->wLengthH) SetupLen = 0xff;

            len = 0;                                                 // 默认为成功并且上传0长度
            SetupReqCode = UsbSetupBuf->bRequest;

            /* 数据方向 */
            data_dir = USB_REQ_TYP_OUT;
            if(UsbSetupBuf->bRequestType & USB_REQ_TYP_IN) data_dir = USB_REQ_TYP_IN;

            /* 厂商请求 */
            if( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_VENDOR )
            {
              switch(SetupReqCode)
              {
                case DEF_VEN_DEBUG_WRITE:  //写两组 0X9A
                {
                  uint8_t write_reg_add1;
                  uint8_t write_reg_add2;
                  uint8_t write_reg_val1;
                  uint8_t write_reg_val2;

                  len = 0;

                  write_reg_add1 = Ep0Buffer[2];
                  write_reg_add2 = Ep0Buffer[3];
                  write_reg_val1 = Ep0Buffer[4];
                  write_reg_val2 = Ep0Buffer[5];

                  /* 该组是设置波特率的寄存器 */
                  if((write_reg_add1 == 0x12)&&(write_reg_add2 == 0x13))
                  {
                    /* 波特率处理采用计算值 */
                    if((UsbSetupBuf->wIndexL==0x87)&&(UsbSetupBuf->wIndexH==0xf3))
                    {
                      bps = 921600;  //13 * 921600 = 11980800
                    }
                    else if((UsbSetupBuf->wIndexL==0x87)&&(UsbSetupBuf->wIndexH==0xd9))
                    {
                      bps = 307200;  //39 * 307200 = 11980800
                    }

                    //系统主频： 36923077
                    else if( UsbSetupBuf->wIndexL == 0x88 )
                    {
                      UINT32 CalClock;
                      uint8_t CalDiv;

                      CalClock = 36923077 / 8;
                      CalDiv = 0 - UsbSetupBuf->wIndexH;
                      bps = CalClock / CalDiv;
                    }
                    else if( UsbSetupBuf->wIndexL == 0x89 )
                    {
                      UINT32 CalClock;
                      uint8_t CalDiv;

                      CalClock = 36923077 / 8 / 256;
                      CalDiv = 0 - UsbSetupBuf->wIndexH;
                      bps = CalClock / CalDiv;
                    }
                    //系统主频：  32000000
                    else if( UsbSetupBuf->wIndexL == 0x8A )
                    {
                      UINT32 CalClock;
                      uint8_t CalDiv;

                      CalClock = 32000000 / 8;
                      CalDiv = 0 - UsbSetupBuf->wIndexH;
                      bps = CalClock / CalDiv;
                    }
                    else if( UsbSetupBuf->wIndexL == 0x8B )
                    {
                      UINT32 CalClock;
                      uint8_t CalDiv;

                      CalClock = 32000000 / 8 / 256;
                      CalDiv = 0 - UsbSetupBuf->wIndexH;
                      bps = CalClock / CalDiv;
                    }
                    else  //340
                    {
                      UINT32 CalClock;
                      uint8_t CalDiv;

                      //115384
                      if((UsbSetupBuf->wIndexL & 0x7f) == 3)
                      {
                        CalClock = 6000000;
                        CalDiv = 0 - UsbSetupBuf->wIndexH;
                        bps = CalClock / CalDiv;
                      }
                      else if((UsbSetupBuf->wIndexL & 0x7f) == 2)
                      {
                        CalClock = 750000;  //6000000 / 8
                        CalDiv = 0 - UsbSetupBuf->wIndexH;
                        bps = CalClock / CalDiv;
                      }
                      else if((UsbSetupBuf->wIndexL & 0x7f) == 1)
                      {
                        CalClock = 93750; //64 分频
                        CalDiv = 0 - UsbSetupBuf->wIndexH;
                        bps = CalClock / CalDiv;
                      }
                      else if((UsbSetupBuf->wIndexL & 0x7f) == 0)
                      {
                        CalClock = 11719;  //约512
                        CalDiv = 0 - UsbSetupBuf->wIndexH;
                        bps = CalClock / CalDiv;
                      }
                      else
                      {
                        bps = 115200;
                      }
                    }
                    Uart0Para.BaudRate = bps;
                    UART_Status = 1;
                    PRINT("CH341 set bps:%d\r\n",(int)bps);
                  }
                  else
                  {
                    CH341RegWrite(write_reg_add1,write_reg_val1);
                    CH341RegWrite(write_reg_add2,write_reg_val2);
                  }

                  break;
                }
                case DEF_VEN_DEBUG_READ:   //需要回传数据 0X95  /* 读两组寄存器 */
                {
                  uint8_t read_reg_add1;
                  uint8_t read_reg_add2;
                  uint8_t read_reg_val1;
                  uint8_t read_reg_val2;

                  read_reg_add1 = UsbSetupBuf->wValueL;
                  read_reg_add2 = UsbSetupBuf->wValueH;

                  CH341RegRead(read_reg_add1,&read_reg_val1);
                  CH341RegRead(read_reg_add2,&read_reg_val2);

                  len = 2;
                  pDescr = buf;
                  buf[0] = read_reg_val1;
                  buf[1] = read_reg_val2;
                  SetupLen = len;
                  memcpy(Ep0Buffer, pDescr, len);

                  break;
                }
                //A1命令也是要初始化串口的
                case DEF_VEN_UART_INIT:  //初始化串口 0XA1
                {
                  uint8_t reg_uart_ctrl;
                  uint8_t  parity_val;
                  uint8_t  data_bit_val;
                  uint8_t  stop_bit_val;
                  uint8_t  uart_reg1_val;
                  uint8_t  uart_reg2_val;
                  uint8_t  uart_set_m;

                  len = 0;

                  if(Ep0Buffer[2] & 0x80)
                  {
                    reg_uart_ctrl = Ep0Buffer[3];

                    data_bit_val = reg_uart_ctrl & 0x03;
                    if   (data_bit_val == 0x00) data_bit_val = HAL_UART_5_BITS_PER_CHAR;
                    else if(data_bit_val == 0x01) data_bit_val = HAL_UART_6_BITS_PER_CHAR;
                    else if(data_bit_val == 0x02) data_bit_val = HAL_UART_7_BITS_PER_CHAR;
                    else if(data_bit_val == 0x03) data_bit_val = HAL_UART_8_BITS_PER_CHAR;

                    stop_bit_val = reg_uart_ctrl & 0x04;
                    if(stop_bit_val) stop_bit_val = HAL_UART_TWO_STOP_BITS;
                    else stop_bit_val = HAL_UART_ONE_STOP_BIT;

                    parity_val = reg_uart_ctrl & (0x38);
                    if(parity_val == 0x00) parity_val = HAL_UART_NO_PARITY;
                    else if(parity_val == 0x08) parity_val = HAL_UART_ODD_PARITY;
                    else if(parity_val == 0x18) parity_val = HAL_UART_EVEN_PARITY;
                    else if(parity_val == 0x28) parity_val = HAL_UART_MARK_PARITY;
                    else if(parity_val == 0x38) parity_val = HAL_UART_SPACE_PARITY;

                    //Uart0Para.BaudRate;
                    Uart0Para.StopBits = stop_bit_val;
                    Uart0Para.ParityType = parity_val;
                    Uart0Para.DataBits = data_bit_val;

                    //直接设置寄存器
                   // UART0ParaSet(data_bit_val, stop_bit_val,parity_val);

                    uart_set_m = 0;
                    uart_reg1_val = UsbSetupBuf->wIndexL;
                    uart_reg2_val = UsbSetupBuf->wIndexH;

                    if(uart_reg1_val & (1<<6))  //判断第六位
                    {
                      uart_set_m = 1;
                    }
                    else
                    {
                      uart_set_m = 1;
                      uart_reg1_val = uart_reg1_val & 0xC7;
                    }

                    if(uart_set_m)
                    {
                      /* 波特率处理采用计算值 */
                      if((uart_reg1_val == 0x87)&&(uart_reg2_val == 0xf3))
                      {
                        bps = 921600;  //13 * 921600 = 11980800
                      }
                      else if((uart_reg1_val == 0x87)&&(uart_reg2_val == 0xd9))
                      {
                        bps = 307200;  //39 * 307200 = 11980800
                      }

                      //系统主频： 36923077
                      else if( uart_reg1_val == 0xC8 )
                      {
                        UINT32 CalClock;
                        uint8_t CalDiv;

                        CalClock = 36923077 / 8;
                        CalDiv = 0 - uart_reg2_val;
                        bps = CalClock / CalDiv;
                      }
                      else if( uart_reg1_val == 0xC9 )
                      {
                        UINT32 CalClock;
                        uint8_t CalDiv;

                        CalClock = 36923077 / 8 / 256;
                        CalDiv = 0 - uart_reg2_val;
                        bps = CalClock / CalDiv;
                      }
                      //系统主频：  32000000
                      else if( uart_reg1_val == 0xCA )
                      {
                        UINT32 CalClock;
                        uint8_t CalDiv;

                        CalClock = 32000000 / 8;
                        CalDiv = 0 - uart_reg2_val;
                        bps = CalClock / CalDiv;
                      }
                      else if( uart_reg1_val == 0xCB )
                      {
                        UINT32 CalClock;
                        uint8_t CalDiv;

                        CalClock = 32000000 / 8 / 256;
                        CalDiv = 0 - uart_reg2_val;
                        bps = CalClock / CalDiv;
                      }
                      else  //340
                      {
                        UINT32 CalClock;
                        uint8_t CalDiv;

                        //115384
                        if((uart_reg1_val & 0x7f) == 3)
                        {
                          CalClock = 6000000;
                          CalDiv = 0 - uart_reg2_val;
                          bps = CalClock / CalDiv;
                        }
                        else if((uart_reg1_val & 0x7f) == 2)
                        {
                          CalClock = 750000;  //6000000 / 8
                          CalDiv = 0 - uart_reg2_val;
                          bps = CalClock / CalDiv;
                        }
                        else if((uart_reg1_val & 0x7f) == 1)
                        {
                          CalClock = 93750; //64 分频
                          CalDiv = 0 - uart_reg2_val;
                          bps = CalClock / CalDiv;
                        }
                        else if((uart_reg1_val & 0x7f) == 0)
                        {
                          CalClock = 11719;  //约512
                          CalDiv = 0 - uart_reg2_val;
                          bps = CalClock / CalDiv;
                        }
                        else
                        {
                          bps = 115200;
                        }
                      }
                      Uart0Para.BaudRate = bps;
                      UART_Status = 1;
                      PRINT("set bps:%d\r\n",(int)bps);
                    }
                  }
                  break;
                }
                case DEF_VEN_UART_M_OUT:  //设置MODEM信号输出 0XA4
                {
                  len = 0;
                  Uart0Para.ioStaus = Ep0Buffer[2];
                  UART_Status = 1;
                  break;
                }
                case DEF_VEN_BUF_CLEAR: //0XB2  /* 清除未完成的数据 */
                {
                  len = 0;

                  //可以重新初始化
                  VENSer0ParaChange = 1; // 重新初始化即可清除所有的数据
                  break;
                }
                case DEF_VEN_I2C_CMD_X:  //0X54  发出I2C接口的命令,立即执行
                {
                  len = 0;
                  break;
                }
                case DEF_VEN_DELAY_MS:  //0X5E  以亳秒为单位延时指定时间
                {
                  len = 0;
                  break;
                }
                case DEF_VEN_GET_VER:   //0X5E   获取芯片版本 //需要回传数据-->版本号
                {
                  len = 2;
                  pDescr = buf;
                  buf[0] = 0x30;
                  buf[1] = 0x00;
                  SetupLen = len;
                  memcpy( Ep0Buffer, pDescr, len );

                  break;
                }
                default:
                  //len = 0xFF;
                  len = 0;
                  break;
              }
            }
            // 标准请求
            else if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD)
            {
              switch( SetupReqCode )  // 请求码
              {
                case USB_GET_DESCRIPTOR:  //获取描述符
                {
                  switch( UsbSetupBuf->wValueH )
                  {
                    case 1: // 设备描述符
                    {
#if ( USB_WORK_MODE== USB_VENDOR_MODE)
                      {
                        memcpy(ep0_send_buf,
                               &TAB_USB_VEN_DEV_DES[0],
                               sizeof( TAB_USB_VEN_DEV_DES ));
                        pDescr = ep0_send_buf;
                        len = sizeof( TAB_USB_VEN_DEV_DES );
                      }
#else                    //CDC类
                      {
                        memcpy(ep0_send_buf,
                               &TAB_USB_CDC_DEV_DES[0],
                               sizeof( TAB_USB_CDC_DEV_DES ));

                        pDescr = ep0_send_buf;
                        len = sizeof( TAB_USB_CDC_DEV_DES );
                      }
#endif
                      break;
                    }
                    case 2:  // 配置描述符
                    {
#if ( USB_WORK_MODE== USB_VENDOR_MODE)
                      {
                        memcpy(ep0_send_buf,
                               &TAB_USB_VEN_CFG_DES[0],
                               sizeof( TAB_USB_VEN_CFG_DES ));
                        pDescr = ep0_send_buf;
                        len = sizeof( TAB_USB_VEN_CFG_DES );
                      }
#else                    //CDC类
                      {
                        memcpy(ep0_send_buf,
                               &TAB_USB_CDC_CFG_DES[0],
                               sizeof( TAB_USB_CDC_CFG_DES ));
                        pDescr = ep0_send_buf;
                        len = sizeof( TAB_USB_CDC_CFG_DES );
                      }
#endif
                      break;
                    }
                    case 3:  // 字符串描述符
                    {
                      PRINT("str %d\r\n",UsbSetupBuf->wValueL);
#if ( USB_WORK_MODE== USB_VENDOR_MODE)
                      {
                        PRINT("str %d\r\n",UsbSetupBuf->wValueL);
                        switch(UsbSetupBuf->wValueL)
                        {
                          case 0:  //语言描述符
                          {
                            pDescr = (PUINT8)( &TAB_USB_LID_STR_DES[0] );
                            len = sizeof( TAB_USB_LID_STR_DES );

                            break;
                          }
                          case 1:  //iManufacturer
                          case 2:   //iProduct
                          case 3:   //iSerialNumber
                          {
                            uint8_t ep0_str_len;
                            uint8_t *p_send;
                            uint8_t *manu_str;
                            uint8_t tmp;

                            /* 取长度 */
                            if(UsbSetupBuf->wValueL == 1)
                              manu_str = (uint8_t *)USB_DEV_PARA_VEN_MANUFACTURE_STR;
                            else if(UsbSetupBuf->wValueL == 2)
                              manu_str = (uint8_t *)USB_DEV_PARA_VEN_PRODUCT_STR;
                            else if(UsbSetupBuf->wValueL == 3)
                              manu_str = (uint8_t *)USB_DEV_PARA_VEN_SERIAL_STR;

                            ep0_str_len = (uint8_t)strlen((char *)manu_str);
                            p_send = ep0_send_buf;
                            *p_send++ = ep0_str_len*2 + 2;
                            *p_send++ = 0x03;
                            for(tmp = 0; tmp<ep0_str_len; tmp++)
                            {
                              *p_send++ = manu_str[tmp];
                              *p_send++ = 0x00;
                            }

                            pDescr = ep0_send_buf;
                            len = ep0_send_buf[0];
                            break;
                          }
                          default:
                            len = 0xFF;    // 不支持的描述符类型
                            break;
                        }
                      }
#else   //CDC模式
                      {
                        PRINT("str %d\r\n",UsbSetupBuf->wValueL);
                        switch(UsbSetupBuf->wValueL)
                        {
                          case 0:  //语言描述符
                          {
                            pDescr = (PUINT8)( &TAB_USB_LID_STR_DES[0] );
                            len = sizeof( TAB_USB_LID_STR_DES );

                            break;
                          }
                          case 1:  //iManufacturer
                          case 2:   //iProduct
                          case 3:   //iSerialNumber
                          {
                            uint8_t ep0_str_len;
                            uint8_t *p_send;
                            uint8_t *manu_str;
                            uint8_t tmp;

                            /* 取长度 */
                            if(UsbSetupBuf->wValueL == 1)
                              manu_str = (uint8_t *)USB_DEV_PARA_CDC_MANUFACTURE_STR;
                            else if(UsbSetupBuf->wValueL == 2)
                              manu_str = (uint8_t *)USB_DEV_PARA_CDC_PRODUCT_STR;
                            else if(UsbSetupBuf->wValueL == 3)
                              manu_str = (uint8_t *)USB_DEV_PARA_CDC_SERIAL_STR;
                            ep0_str_len = (uint8_t)strlen((char *)manu_str);
                            p_send = ep0_send_buf;
                            *p_send++ = ep0_str_len*2 + 2;
                            *p_send++ = 0x03;
                            for(tmp = 0; tmp<ep0_str_len; tmp++)
                            {
                              *p_send++ = manu_str[tmp];
                              *p_send++ = 0x00;
                            }

                            pDescr = ep0_send_buf;
                            len = ep0_send_buf[0];

                            break;
                          }
                          default:
                            len = 0xFF;    // 不支持的描述符类型
                            break;
                        }
                      }
#endif
                      break;
                    }
                    case 6:  //设备限定描述符
                    {
                      pDescr = (PUINT8)( &My_QueDescr[0] );
                      len = sizeof( My_QueDescr );
                      break;
                    }
                    default:
                      len = 0xFF;                                  // 不支持的描述符类型
                      break;
                  }
                  if ( SetupLen > len ) SetupLen = len;            // 限制总长度
                  len = (SetupLen >= THIS_ENDP0_SIZE) ? THIS_ENDP0_SIZE : SetupLen;  // 本次传输长度
                  memcpy( Ep0Buffer, pDescr, len );                 /* 加载上传数据 */
                  SetupLen -= len;
                  pDescr += len;

                  break;
                }
                case USB_SET_ADDRESS:  //设置地址
                {
                  PRINT("SET_ADDRESS:%d\r\n",UsbSetupBuf->wValueL);
                  devinf.gUsbFlag |= DEF_BIT_USB_ADDRESS;
                  devinf.UsbAddress = UsbSetupBuf->wValueL;    // 暂存USB设备地址

                  break;
                }
                case USB_GET_CONFIGURATION:
                {
                  PRINT("GET_CONFIGURATION\r\n");
                  Ep0Buffer[0] = devinf.UsbConfig;
                  if ( SetupLen >= 1 ) len = 1;

                  break;
                }
                case USB_SET_CONFIGURATION:
                {
                  PRINT("SET_CONFIGURATION\r\n");
                  devinf.gUsbFlag |= DEF_BIT_USB_SET_CFG;
                  devinf.UsbConfig = UsbSetupBuf->wValueL;
                  break;
                }
                case USB_CLEAR_FEATURE:
                {
                  PRINT("CLEAR_FEATURE\r\n");
                  len = 0;
                  /* 清除设备 */
                  if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                  {
                    R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
                    R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
                    R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
                    R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;

                    //状态变量复位
                    Ep1DataINFlag = 1;
                    Ep1DataOUTFlag = 0;

                    cdc_uart_sta_trans_step = 0;
                    ven_ep1_trans_step = 0;
                  }
                  else if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )  // 端点
                  {
                    switch( UsbSetupBuf->wIndexL )   //判断端点
                    {
                      case 0x84: R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
                      case 0x04: R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
                      case 0x83: R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
                      case 0x03: R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
                      case 0x82: R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
                      case 0x02: R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
                      case 0x81: R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
                      case 0x01: R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
                      default: len = 0xFF;  break;
                    }
                  }
                  else len = 0xFF;                                  // 不是端点不支持

                  break;
                }
                case USB_GET_INTERFACE:
                {
                  PRINT("GET_INTERFACE\r\n");
                  Ep0Buffer[0] = 0x00;
                  if ( SetupLen >= 1 ) len = 1;
                  break;
                }
                case USB_GET_STATUS:
                {
                  PRINT("GET_STATUS\r\n");
                  Ep0Buffer[0] = 0x00;
                  Ep0Buffer[1] = 0x00;
                  if ( SetupLen >= 2 ) len = 2;
                  else len = SetupLen;
                  break;
                }
                default:
                  len = 0xFF;                                       // 操作失败
                  break;
              }
            }
            /* 类请求 */
            else if( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS )
            {
              /* 主机下传 */
              if(data_dir == USB_REQ_TYP_OUT)
              {
                switch( SetupReqCode )  // 请求码
                {
                  case DEF_SET_LINE_CODING: /* SET_LINE_CODING */
                  {
                    uint8_t i;
                    PRINT("SET_LINE_CODING\r\n");
                    for(i=0; i<8; i++)
                    {
                      PRINT("%02x ",Ep0Buffer[i]);
                    }
                    PRINT("\r\n");
                    if( Ep0Buffer[ 4 ] == 0x00 )
                    {
                      CDCSetSerIdx = 0;
                      len = 0x00;
                    }
                    else if( Ep0Buffer[ 4 ] == 0x02 )
                    {
                      CDCSetSerIdx = 1;
                      len = 0x00;
                    }
                    else len = 0xFF;
                    break;
                  }
                  case DEF_SET_CONTROL_LINE_STATE:  /* SET_CONTROL_LINE_STATE */
                  {
                    uint8_t  carrier_sta;
                    uint8_t  present_sta;
                    /* 线路状态 */
                    PRINT("ctl %02x %02x\r\n",Ep0Buffer[2],Ep0Buffer[3]);
                    carrier_sta = Ep0Buffer[2] & (1<<1);   //RTS状态
                    present_sta = Ep0Buffer[2] & (1<<0);   //DTR状态
                    len = 0;
                    break;
                  }
                  default:
                  {
                    PRINT("CDC ReqCode%x\r\n",SetupReqCode);
                    len = 0xFF;                                       // 操作失败
                    break;
                  }
                }
              }
              /* 设备上传 */
              else
              {
                switch( SetupReqCode )  // 请求码
                {
                  case DEF_GET_LINE_CODING: /* GET_LINE_CODING */
                  {
                    PRINT("GET_LINE_CODING:%d\r\n",Ep0Buffer[ 4 ]);
                    pDescr = Ep0Buffer;
                    len = sizeof( LINE_CODE );
                    ( ( PLINE_CODE )Ep0Buffer )->BaudRate   = Uart0Para.BaudRate;
                    ( ( PLINE_CODE )Ep0Buffer )->StopBits   = Uart0Para.StopBits;
                    ( ( PLINE_CODE )Ep0Buffer )->ParityType = Uart0Para.ParityType;
                    ( ( PLINE_CODE )Ep0Buffer )->DataBits   = Uart0Para.DataBits;
                    break;
                  }
                  case DEF_SERIAL_STATE:
                  {
                    PRINT("GET_SERIAL_STATE:%d\r\n",Ep0Buffer[ 4 ]);
                    //SetupLen 判断总长度
                    len = 2;
                    CDCSetSerIdx = 0;
                    Ep0Buffer[0] = 0;
                    Ep0Buffer[1] = 0;
                    break;
                  }
                  default:
                  {
                    PRINT("CDC ReqCode%x\r\n",SetupReqCode);
                    len = 0xFF;                                       // 操作失败
                    break;
                  }
                }
              }
            }

            else len = 0xFF;   /* 失败 */
          }
          else
          {
            len = 0xFF; // SETUP包长度错误
          }
          if ( len == 0xFF )  // 操作失败
          {
            SetupReqCode = 0xFF;
            PFIC_DisableIRQ(USB_IRQn);
            R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;  // STALL
            PFIC_EnableIRQ(USB_IRQn);
          }
          else if ( len <= THIS_ENDP0_SIZE )  // 上传数据或者状态阶段返回0长度包
          {
            if( SetupReqCode ==  USB_SET_ADDRESS)  //设置地址 0x05
            {
//              PRINT("add in:%d\r\n",len);
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //默认数据包是DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  USB_SET_CONFIGURATION)  //设置配置值 0x09
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //默认数据包是DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  USB_GET_DESCRIPTOR)  //获取描述符  0x06
            {
              R8_UEP0_T_LEN = len;
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;  //默认数据包是DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  DEF_VEN_UART_INIT )  //0XA1 初始化串口
            {
              R8_UEP0_T_LEN = len;
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //默认数据包是DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  DEF_VEN_DEBUG_WRITE )  //0X9A
            {
              R8_UEP0_T_LEN = len;
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //默认数据包是DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  DEF_VEN_UART_M_OUT )  //0XA4
            {
              R8_UEP0_T_LEN = len;
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //默认数据包是DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  DEF_SET_CONTROL_LINE_STATE )  //0x22
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //默认数据包是DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  USB_CLEAR_FEATURE )  //0x01
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //默认数据包是DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else
            {
              if(data_dir == USB_REQ_TYP_IN)   //当前需要上传
              {
                PFIC_DisableIRQ(USB_IRQn);
                R8_UEP0_T_LEN = len;
                R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //默认数据包是DATA1
                PFIC_EnableIRQ(USB_IRQn);
              }
              else                            //当前需要下传
              {
                PFIC_DisableIRQ(USB_IRQn);
                R8_UEP0_T_LEN = len;
                R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;  //默认数据包是DATA1
                PFIC_EnableIRQ(USB_IRQn);
              }
            }
          }
          else  // 下传数据或其它
          {
            //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
            R8_UEP0_T_LEN = 0;
            PFIC_DisableIRQ(USB_IRQn);
            R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;  // 默认数据包是DATA1
            PFIC_EnableIRQ(USB_IRQn);
          }
          break;
        }
        case UIS_TOKEN_IN | 0:      // endpoint 0# IN  UIS_TOKEN_IN
        {
          switch( SetupReqCode )
          {
            /* 简单的处理SETUP的命令 */
            case USB_GET_DESCRIPTOR:  //0x06  获取描述符
            {
              len = (SetupLen >= THIS_ENDP0_SIZE) ? THIS_ENDP0_SIZE : SetupLen;  // 本次传输长度
              memcpy( Ep0Buffer, pDescr, len );                    /* 加载上传数据 */
              SetupLen -= len;
              pDescr += len;

              if(len)
              {
                R8_UEP0_T_LEN = len;
                PFIC_DisableIRQ(USB_IRQn);
                R8_UEP0_CTRL ^=  RB_UEP_T_TOG;
                USBDevEPnINSetStatus(ENDP0, ENDP_TYPE_IN, IN_ACK);
                PFIC_EnableIRQ(USB_IRQn);
              }
              else
              {
                R8_UEP0_T_LEN = len;
                PFIC_DisableIRQ(USB_IRQn);
                R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
                PFIC_EnableIRQ(USB_IRQn);
              }
              break;
            }
            case USB_SET_ADDRESS:   //0x05
            {
              R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | (devinf.UsbAddress);
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
              PFIC_EnableIRQ(USB_IRQn);
//              PRINT("add in deal\r\n");

              break;
            }
            //厂商读取
            case DEF_VEN_DEBUG_READ:     //0X95
            case DEF_VEN_GET_VER:         //0X5F
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
              PFIC_EnableIRQ(USB_IRQn);

              break;
            }
            case DEF_GET_LINE_CODING:  //0x21
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
              PFIC_EnableIRQ(USB_IRQn);

              break;
            }
            case DEF_SET_LINE_CODING:   //0x20
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
              PFIC_EnableIRQ(USB_IRQn);
              break;
            }
            default:
            {
              R8_UEP0_T_LEN = 0;                                      // 状态阶段完成中断或者是强制上传0长度数据包结束控制传输
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
              PFIC_EnableIRQ(USB_IRQn);

              break;
            }
          }
          break;
        }
        case UIS_TOKEN_OUT | 0:      // endpoint 0# OUT
        {
          len = usb_irq_len[i];
          if(len)
          {
#if ( USB_WORK_MODE== USB_VENDOR_MODE)
            {
              switch(SetupReqCode)
              {
                /* 设置串口 */
                case DEF_SET_LINE_CODING:
                {
                  UINT32 set_bps;
                  uint8_t  data_bit;
                  uint8_t  stop_bit;
                  uint8_t  ver_bit;
                  uint8_t  set_stop_bit;

                  memcpy(&set_bps,Ep0Buffer,4);
                  stop_bit = Ep0Buffer[4];
                  ver_bit = Ep0Buffer[5];
                  data_bit = Ep0Buffer[6];

                  PRINT("LINE_CODING %d %d %d %d %d\r\n",CDCSetSerIdx
                                       ,(int)set_bps
                                       ,data_bit
                                       ,stop_bit
                                       ,ver_bit);

                    Uart0Para.BaudRate = set_bps;
                    Uart0Para.StopBits = stop_bit;
                    Uart0Para.ParityType = ver_bit;
                    Uart0Para.DataBits = data_bit;
                    CDCSer0ParaChange = 1;

                  PFIC_DisableIRQ(USB_IRQn);
                  R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK|UEP_T_RES_ACK;
                  PFIC_EnableIRQ(USB_IRQn);
                  break;
                }
                default:
                  PFIC_DisableIRQ(USB_IRQn);
                  R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
                  PFIC_EnableIRQ(USB_IRQn);
                  break;
              }
            }
#else
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
              PFIC_EnableIRQ(USB_IRQn);
            }
#endif
          }
          else
          {
            PFIC_DisableIRQ(USB_IRQn);
            R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK|UEP_T_RES_NAK;
            PFIC_EnableIRQ(USB_IRQn);
          }
          break;
        }
        default:
          ep_idx = 0xff;
          break;
      }
    }
}

/*******************************************************************************
* Function Name  : USBDevEPnINSetStatus
* Description    : 端点状态设置函数
* Input          : ep_num：端点号
                   type：端点传输类型
                   sta：切换的端点状态
* Output         : None
* Return         : None
*******************************************************************************/
void USBDevEPnINSetStatus(uint8_t ep_num, uint8_t type, uint8_t sta)
{
  uint8_t *p_UEPn_CTRL;

  p_UEPn_CTRL = (uint8_t *)(USB_BASE_ADDR + 0x22 + ep_num * 4);
  if(type == ENDP_TYPE_IN) *((PUINT8V)p_UEPn_CTRL) = (*((PUINT8V)p_UEPn_CTRL) & (~(0x03))) | sta;
  else *((PUINT8V)p_UEPn_CTRL) = (*((PUINT8V)p_UEPn_CTRL) & (~(0x03<<2))) | (sta<<2);
}

/*******************************************************************************
* Function Name  : USBParaInit
* Description    : USB参数初始化，缓存和标志
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBParaInit(void)
{
    Ep1DataINFlag = 1;
    Ep1DataOUTFlag = 0;
}


/*******************************************************************************
* Function Name  : InitCDCDevice
* Description    : 初始化CDC设备
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitCDCDevice(void)
{
  /* 初始化缓存 */
  USBParaInit();

  R8_USB_CTRL = 0x00;                                                 // 先设定模式

//  1.端点分配：
//  端点0
//  端点1：IN和OUT  （数据接口）
//  端点2：IN和OUT  （数据接口）
//  端点3：IN       （接口23组合，中断上传）
//  端点4：IN       （接口01组合，中断上传）

  R8_UEP4_1_MOD = RB_UEP4_TX_EN|RB_UEP1_TX_EN|RB_UEP1_RX_EN;

  /* 单 64 字节接收缓冲区(OUT)，单 64 字节发送缓冲区（IN） */
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_TX_EN;

  R16_UEP0_DMA = (UINT32)&Ep0Buffer[0];
  R16_UEP1_DMA = (UINT32)&Ep1Buffer[0];
  R16_UEP2_DMA = (UINT32)&Ep2Buffer[0];
  R16_UEP3_DMA = (UINT32)&Ep3Buffer[0];
  //R16_UEP4_DMA = (UINT16)(UINT32)&Ep2Buffer[0];

  /* 端点0状态：OUT--ACK IN--NAK */
  R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;

  /* 端点1状态：OUT--ACK IN--NAK 自动翻转 */
  R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  /* 端点2状态：OUT--ACK IN--NAK 自动翻转 */
  R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  /* 端点3状态：IN--NAK 自动翻转 */
  R8_UEP3_CTRL = UEP_T_RES_NAK;

  /* 端点4状态：IN--NAK 手动翻转 */
  R8_UEP4_CTRL = UEP_T_RES_NAK;

  /* 设备地址 */
  R8_USB_DEV_AD = 0x00;

  //禁止DP/DM下拉电阻s
  R8_UDEV_CTRL = RB_UD_PD_DIS;

  //启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;

  //清中断标志
  R8_USB_INT_FG = 0xFF;

  //程序统一查询？
  //开启中断          挂起            传输完成         总线复位
  R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_TRANSFER | RB_UIE_BUS_RST;
//  R8_USB_INT_EN = RB_UIE_TRANSFER ;
  PFIC_EnableIRQ(USB_IRQn);

  //使能USB端口
  R8_UDEV_CTRL |= RB_UD_PORT_EN;

  devinf.UsbConfig = 0;
  devinf.UsbAddress = 0;
}

/*******************************************************************************
* Function Name  : InitVendorDevice
* Description    : 初始化厂商USB设备
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitVendorDevice(void)
{
  /* 初始化缓存 */
  USBParaInit();

  R8_USB_CTRL = 0x00;                                                 // 先设定模式

//  1.端点分配：
//  端点0
//  端点1：IN和OUT  （配置接口）
//  端点2：IN和OUT  （数据接口）

  /* 单 64 字节接收缓冲区(OUT)，单 64 字节发送缓冲区（IN） */
  R8_UEP4_1_MOD = RB_UEP1_TX_EN | RB_UEP1_RX_EN;

  /* 单 64 字节接收缓冲区(OUT)，单 64 字节发送缓冲区（IN） */
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN;

  R16_UEP0_DMA = (UINT32)&Ep0Buffer[0];
  R16_UEP1_DMA = (UINT32)&Ep1Buffer[0];
  R16_UEP2_DMA = (UINT32)&Ep2Buffer[0];

  /* 端点0状态：OUT--ACK IN--NAK */
  R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;

  /* 端点1状态：OUT--ACK IN--NAK 自动翻转 */
  R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  /* 端点2状态：OUT--ACK IN--NAK 自动翻转 */
  R8_UEP2_CTRL =  UEP_R_RES_ACK | UEP_T_RES_NAK;

  /* 设备地址 */
  R8_USB_DEV_AD = 0x00;

  //禁止DP/DM下拉电阻s
  R8_UDEV_CTRL = RB_UD_PD_DIS;

  //启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;

  //清中断标志
  R8_USB_INT_FG = 0xFF;

  //开启中断          挂起            传输完成         总线复位
  R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_TRANSFER | RB_UIE_BUS_RST;
  PFIC_EnableIRQ(USB_IRQn);

  //使能USB端口
  R8_UDEV_CTRL |= RB_UD_PORT_EN;

  devinf.UsbConfig = 0;
  devinf.UsbAddress = 0;
}

/*******************************************************************************
* Function Name  : InitUSBDevPara
* Description    : USB相关的变量初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitUSBDevPara(void)
{
  uint8_t i;

  Uart0Para.BaudRate = 115200;
  Uart0Para.DataBits = HAL_UART_8_BITS_PER_CHAR;
  Uart0Para.ParityType = HAL_UART_NO_PARITY;
  Uart0Para.StopBits = HAL_UART_ONE_STOP_BIT;
  UART_Status = 1;
  VENSer0ParaChange = 0;
  VENSer0SendFlag = 0;
  CDCSetSerIdx = 0;
  CDCSer0ParaChange = 0;

  for(i=0; i<CH341_REG_NUM; i++)
  {
    CH341_Reg_Add[i] = 0xff;
    CH341_Reg_val[i] = 0x00;
  }

  UART0_DCD_Val = 0;
  UART0_RI_Val = 0;
  UART0_DSR_Val = 0;
  UART0_CTS_Val = 0;

  UART0_RTS_Val = 0; //输出 表示DTE请求DCE发送数据
  UART0_DTR_Val = 0; //输出 数据终端就绪

  for(i=0; i<USB_IRQ_FLAG_NUM; i++)
  {
    usb_irq_flag[i] = 0;
  }
}

/*******************************************************************************
* Function Name  : InitUSBDevice
* Description    : 初始化USB
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitUSBDevice(void)
{
#if ( USB_WORK_MODE== USB_VENDOR_MODE)
    InitVendorDevice();
#else
    InitCDCDevice();
#endif
}

/*******************************************************************************
 * @fn      USB_StatusQuery
 *
 * @brief
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
void USB_StatusQuery( void )
{
    typeBufSize len;
    uint8_t i;

    i = usb_irq_r_idx;
    if(usb_irq_flag[i])
    {
        USB_IRQProcessHandler( );
        usb_irq_flag[i] = 0;
    }
    if( devinf.UsbAddress )
    {
#if ( USB_WORK_MODE== USB_VENDOR_MODE)
        {
            if( (R8_UEP2_CTRL&MASK_UEP_T_RES) == UEP_T_RES_NAK )
            {
                len  = MAX_PACKET_SIZE/2;
                if( RF_RxQuery( &Ep2Buffer[64], &len ) )
                {
                    if( len > 32 )
                    {
                        PRINT("!!! error usb in l=%d %x\n",len,R8_UEP2_CTRL);
                    }
                    R8_UEP2_T_LEN = (uint8_t)len;
                    PFIC_DisableIRQ(USB_IRQn);
                    R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK; //IN_ACK
                    PFIC_EnableIRQ(USB_IRQn);
                }
            }
        }
        /* CDC模式处理 */
#else
        {
            if( Ep1DataINFlag )
            {
                len  = MAX_PACKET_SIZE;
                if( RF_RxQuery( &Ep1Buffer[0], &len ) )
                {
                    /* 直接发送数据 */
                    Ep1DataINFlag = 0;
                    R8_UEP1_T_LEN = (uint8_t)len;
                    PFIC_DisableIRQ(USB_IRQn);
                    R8_UEP1_CTRL = R8_UEP1_CTRL & 0xfc; //IN_ACK
                    PFIC_EnableIRQ(USB_IRQn);
                }
            }
        }
#endif
    }
 }

/*******************************************************************************
 * @fn      USB_RxQuery
 *
 * @brief
 *
 * @param   None.
 *
 * @return  None.
 */
/************************************************************************
 * @fn      USB_RxQuery
 *
 * @brief   查询USB是否接收到数据
 *
 * @return  0：有数据   0x80 状态或者命令  0xFF 无事件
 */
__HIGH_CODE
uint8_t USB_RxQuery( void *buf, typeBufSize *len )
{
    uint8_t *p;

    if( Ep2DataOUTFlag )
    {
        *len = Ep2DataOUTLen;
        __MCPY( buf,Ep2OUTDataBuf,&Ep2OUTDataBuf[Ep2DataOUTLen] );
        PFIC_DisableIRQ(USB_IRQn);
        Ep2DataOUTFlag = 0;
        R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_R_RES)|UEP_R_RES_ACK; //OUT_ACK
        PFIC_EnableIRQ(USB_IRQn);
        return 0x0;
    }
    else if( devinf.UsbAddress )
    {
        if( UART_Status )
        {
            UART_Status = 0;
            *len = 0;
            return 0x80;
        }
    }
    *len = 0;
    return 0xFF;
}

/*******************************************************************************
 * @fn      USB_Init
 *
 * @brief
 *
 * @param   None.
 *
 * @return  None.
 */
void USB_Init( void )
{
    // 关闭仿真调试接口
    R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;
    InitUSBDevPara();
    InitUSBDevice();
    PFIC_EnableIRQ( USB_IRQn );
}


