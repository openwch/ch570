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

/* USB����ģʽ */
#define  USB_WORK_MODE     USB_VENDOR_MODE



/* CDC��ز��� */
/* �豸������ */
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
  0x01,                 //�����ߵ��ַ���������������ֵ
  0x02,                 //��Ʒ���ַ���������������ֵ
  0x03,                 //��ŵ��ַ���������������ֵ
  0x01                  //�������õ���Ŀ
};


/* ���������� */
const uint8_t TAB_USB_CDC_CFG_DES[ ] =
{
  0x09,0x02,0x43,0x00,0x02,0x01,0x00,0x80,0x30,

  //����Ϊ�ӿ�0��CDC�ӿڣ�������
  0x09, 0x04,0x00,0x00,0x01,0x02,0x02,0x01,0x00,

  0x05,0x24,0x00,0x10,0x01,
  0x04,0x24,0x02,0x02,
  0x05,0x24,0x06,0x00,0x01,
  0x05,0x24,0x01,0x01,0x00,

  0x07,0x05,0x84,0x03,0x08,0x00,0x01,                       //�ж��ϴ��˵�������

  //����Ϊ�ӿ�1�����ݽӿڣ�������
  0x09,0x04,0x01,0x00,0x02,0x0a,0x00,0x00,0x00,

  0x07,0x05,0x01,0x02,0x40,0x00,0x00,                       //�˵�������
  0x07,0x05,0x81,0x02,0x40,0x00,0x00,                       //�˵�������
};

/* �豸�޶������� */
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
/* �豸������ *///0x00, DEF_IC_PRG_VER, //BCD �豸�汾��
const uint8_t TAB_USB_VEN_DEV_DES[] =
{
  0x12,
  0x01,
  0x10, 0x01,
  0xff, 0x00, 0x02, 0x40,//0x40,
  0x86, 0x1a, 0x23, 0x75,
  0x00, DEF_IC_PRG_VER,

  0x01,                 //�����ߵ��ַ���������������ֵ
  0x02,                 //��Ʒ���ַ���������������ֵ
  0x03,                 //��ŵ��ַ���������������ֵ

  0x01
};

const uint8_t TAB_USB_VEN_CFG_DES[39] =
{
  0x09, 0x02, 0x27, 0x00, 0x01, 0x01, 0x00, 0x80, 0x30,   //����������
  0x09, 0x04, 0x00, 0x00, 0x03, 0xff, 0x01, 0x02, 0x00,   //�ӿ�
  0x07, 0x05, 0x82, 0x02, 0x20, 0x00, 0x00,               //�˵� IN2   ����
  0x07, 0x05, 0x02, 0x02, 0x20, 0x00, 0x00,               //�˵� OUT2  ����
  0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x01        //�˵� IN1   �ж�
};

// ����������
const uint8_t TAB_USB_LID_STR_DES[ ] = { 0x04, 0x03, 0x09, 0x04 };
// ������Ϣ
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
  uint8_t UsbConfig;      // USB���ñ�־
  uint8_t UsbAddress;     // USB�豸��ַ
  uint8_t gSetupReq;        /* USB���ƴ��������� */
  uint8_t gSetupLen;          /* USB���ƴ��䴫�䳤�� */
  uint8_t gUsbInterCfg;       /* USB�豸�ӿ����� */
  uint8_t gUsbFlag;       /* USB�豸���ֲ�����־,λ0=���߸�λ,λ1=��ȡ�豸������,λ2=���õ�ַ,λ3=��ȡ����������,λ4=�������� */
}DevInfo_Parm;

/* �豸��Ϣ */
DevInfo_Parm  devinf;
uint8_t SetupReqCode, SetupLen;

/* �˵�Ӳ������������Ļ����� */
__aligned(4) uint8_t  Ep0Buffer[MAX_PACKET_SIZE];     //�˵�0 �շ�����  �˵�4 OUT & IN

//�˵�4���ϴ���ַ
__aligned(4) uint8_t  Ep1Buffer[MAX_PACKET_SIZE];     //IN
__aligned(4) uint8_t  Ep2Buffer[2*MAX_PACKET_SIZE];   //OUT & IN
__aligned(4) uint8_t  Ep3Buffer[2*MAX_PACKET_SIZE];   //OUT & IN


/* ��������������� */
LINE_CODE Uart0Para;

#define CH341_REG_NUM     10
uint8_t CH341_Reg_Add[CH341_REG_NUM];
uint8_t CH341_Reg_val[CH341_REG_NUM];


/* ����ģʽ�´��ڲ����仯 */
uint8_t VENSer0ParaChange = 0;

/* ����ģʽ�´��ڷ������ݱ�־ */
uint8_t VENSer0SendFlag = 0;

/* ����ģʽ��modem�źż�� */
uint8_t UART0_RTS_Val = 0; //��� ��ʾDTE����DCE��������
uint8_t UART0_DTR_Val = 0; //��� �����ն˾���
uint8_t UART0_OUT_Val = 0; //�Զ���modem�źţ�CH340�ֲᣩ

uint8_t UART_Status;

uint8_t UART0_DCD_Val = 0;
uint8_t UART0_DCD_Change = 0;

uint8_t UART0_RI_Val = 0;
uint8_t UART0_RI_Change = 0;

uint8_t UART0_DSR_Val = 0;
uint8_t UART0_DSR_Change = 0;

uint8_t UART0_CTS_Val = 0;
uint8_t UART0_CTS_Change = 0;

/* CDC���õĴ��� */
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


/* USB���涨�壬���ж˵�ȫ������ */
/* �˵�1 -- IN״̬ */
uint8_t Ep1DataINFlag = 0;

/* �˵�1�´����� */
uint8_t Ep1DataOUTFlag = 0;
uint8_t Ep1DataOUTLen = 0;
__aligned(4) uint8_t Ep1OUTDataBuf[MAX_PACKET_SIZE];


/* �˵�2�´����� */
uint8_t Ep2DataOUTFlag = 0;
uint8_t Ep2DataOUTLen = 0;
__aligned(4) uint8_t Ep2OUTDataBuf[MAX_PACKET_SIZE];

/* ����USB�жϵ�״̬ ->�ĳɼ���Ĳ�����ʽ */
#define USB_IRQ_FLAG_NUM     4

uint8_t usb_irq_w_idx = 0;
uint8_t usb_irq_r_idx = 0;

volatile uint8_t usb_irq_len[USB_IRQ_FLAG_NUM];
volatile uint8_t usb_irq_pid[USB_IRQ_FLAG_NUM];
volatile uint8_t usb_irq_flag[USB_IRQ_FLAG_NUM];

uint8_t cdc_uart_sta_trans_step = 0;
uint8_t ven_ep1_trans_step = 0;

/* �˵�0ö���ϴ�֡���� */
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
#define HAL_UART_NO_PARITY                   0x00                              //��У��
#define HAL_UART_ODD_PARITY                  0x01                              //��У��
#define HAL_UART_EVEN_PARITY                 0x02                              //żУ��
#define HAL_UART_MARK_PARITY                 0x03                              //��1 mark
#define HAL_UART_SPACE_PARITY                0x04                              //�հ�λ space


/* �˵�״̬���ú��� */
void USBDevEPnINSetStatus(uint8_t ep_num, uint8_t type, uint8_t sta);

/*******************************************************************************
* Function Name  : CH341RegWrite
* Description    : д�� CH341�ļĴ���
* Input          : reg_add��д��Ĵ�����ַ
                   reg_val��д��Ĵ�����ֵ
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
    case 0x18: //SFR_UART_CTRL -->���ڵĲ����Ĵ���
    {
      uint8_t reg_uart_ctrl;
      uint8_t data_bit_val;
      uint8_t stop_bit_val;
      uint8_t parity_val;
      uint8_t break_en;

      reg_uart_ctrl = reg_val;
      /* breakλ */
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

      //ֱ�����üĴ���
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
* Description    : ��ȡ CH341�ļĴ���
* Input          : reg_add����ȡ�ļĴ�����ַ
                   reg_val����ȡ�ļĴ�����ֵ���ָ��
* Output         : None
* Return         : �Ĵ�������
*******************************************************************************/
uint8_t CH341RegRead(uint8_t reg_add,uint8_t *reg_val)
{
  uint8_t find_flag;
  uint8_t i;

  find_flag = 0;
  *reg_val = 0;
  for(i=0; i<CH341_REG_NUM; i++)
  {
    if(CH341_Reg_Add[i] == reg_add)   //�ҵ���ͬ��ַ�ļĴ���
    {
      find_flag = 1;
      *reg_val = CH341_Reg_val[i];
      break;
    }
    if(CH341_Reg_Add[i] == 0xff)      //�ҵ���ǰ��һ����
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
    case 0x18:   //SFR_UART_CTRL -->���ڵĲ����Ĵ���
    {
      uint8_t  reg_uart_ctrl_val;
      uint8_t  ram_uart_ctrl_val;

      reg_uart_ctrl_val = R8_UART_LCR;
      //����breakλ
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

/* �˵�Ӧ��״̬���� */
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

/* USB�豸���ֱ�־λ���� */
#define DEF_BIT_USB_RESET               0x01                                    /* ���߸�λ��־ */
#define DEF_BIT_USB_DEV_DESC            0x02                                    /* ��ȡ���豸��������־ */
#define DEF_BIT_USB_ADDRESS             0x04                                    /* ���ù���ַ��־ */
#define DEF_BIT_USB_CFG_DESC            0x08                                    /* ��ȡ��������������־ */
#define DEF_BIT_USB_SET_CFG             0x10                                    /* ���ù�����ֵ��־ */
#define DEF_BIT_USB_WAKE                0x20                                    /* USB���ѱ�־ */
#define DEF_BIT_USB_SUPD                0x40                                    /* USB���߹����־ */
#define DEF_BIT_USB_HS                  0x80                                    /* USB���١�ȫ�ٱ�־ */


/* CH341��ص�����֡ */
#define DEF_VEN_DEBUG_READ              0X95         /* ������Ĵ��� */
#define DEF_VEN_DEBUG_WRITE             0X9A         /* д����Ĵ��� */
#define DEF_VEN_UART_INIT               0XA1         /* ��ʼ������ */
#define DEF_VEN_UART_M_OUT              0XA4         /* ����MODEM�ź���� */
#define DEF_VEN_BUF_CLEAR               0XB2         /* ���δ��ɵ����� */
#define DEF_VEN_I2C_CMD_X               0X54         /* ����I2C�ӿڵ�����,����ִ�� */
#define DEF_VEN_DELAY_MS                0X5E         /* ������Ϊ��λ��ʱָ��ʱ�� */
#define DEF_VEN_GET_VER                 0X5F         /* ��ȡоƬ�汾 */

/* ������ */
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
/* �жϴ����� */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void USB_IRQHandler(void)
{
  uint8_t   i;
  uint8_t   j;

  if(R8_USB_INT_FG & RB_UIF_TRANSFER)
  {
    /* ��setup������ */
    if((R8_USB_INT_ST & MASK_UIS_TOKEN) != MASK_UIS_TOKEN){     // �ǿ���
      /* ֱ��д�� */
      usb_irq_flag[usb_irq_w_idx] = 1;
      usb_irq_pid[usb_irq_w_idx]  = R8_USB_INT_ST;  //& 0x3f;//(0x30 | 0x0F);
      usb_irq_len[usb_irq_w_idx]  = R8_USB_RX_LEN;

      switch(usb_irq_pid[usb_irq_w_idx]& 0x3f){   //������ǰ�Ķ˵�
        case UIS_TOKEN_OUT | 2:
        {
          if( R8_USB_INT_FG & RB_U_TOG_OK ){   //��ͬ�������ݰ�������
            R8_UEP2_CTRL ^=  RB_UEP_R_TOG;
            R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK; //OUT_NAK
            /* �������� */
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
        case UIS_TOKEN_IN | 2:{ //endpoint 2# �����˵��ϴ����
          R8_UEP2_CTRL ^=  RB_UEP_T_TOG;
          R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | IN_NAK; //IN_NAK
        }break;
        case UIS_TOKEN_OUT | 1:{
          if( R8_USB_INT_FG & RB_U_TOG_OK ){   //��ͬ�������ݰ�������
            R8_UEP1_CTRL ^=  RB_UEP_R_TOG;
            R8_UEP1_CTRL = (R8_UEP1_CTRL & 0xf3) | 0x08; //OUT_NAK
            /* �������� */
            for(j=0; j<(MAX_PACKET_SIZE/4); j++)
              ((UINT32 *)Ep1OUTDataBuf)[j] = ((UINT32 *)Ep1Buffer)[j];
          }
          else usb_irq_flag[usb_irq_w_idx] = 0;
        }break;
        case UIS_TOKEN_IN | 1:{  //endpoint 1# �����˵��ϴ����
          R8_UEP1_CTRL ^=  RB_UEP_T_TOG;
          R8_UEP1_CTRL = (R8_UEP1_CTRL & 0xfc) | IN_NAK; //IN_NAK
        }break;
        case UIS_TOKEN_OUT | 0:{    // endpoint 0
          if( R8_USB_INT_FG & RB_U_TOG_OK )   //��ͬ�������ݰ�������
            R8_UEP0_CTRL = (R8_UEP0_CTRL & 0xf3) | 0x08; //OUT_NAK
          else usb_irq_flag[usb_irq_w_idx] = 0;
        }break;
        case UIS_TOKEN_IN | 0:{  //endpoint 0
          R8_UEP0_CTRL = (R8_UEP0_CTRL & 0xfc) | IN_NAK; //IN_NAK
        }break;
      }

      /* �е���һ��д��ַ */
      if(usb_irq_flag[usb_irq_w_idx]){
        usb_irq_w_idx++;
        if(usb_irq_w_idx >= USB_IRQ_FLAG_NUM) usb_irq_w_idx = 0;
      }

      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }

    /* setup������ */
    if(R8_USB_INT_ST & RB_UIS_SETUP_ACT)
    {
      /* ��֮ǰ�Ĵ���ӹ� -- UIS_TOKEN_SETUP */
      /* ֱ��д�� */
      usb_irq_flag[usb_irq_w_idx] = 1;
      usb_irq_pid[usb_irq_w_idx]  = UIS_TOKEN_SETUP | 0;  //����֮ǰ�ı�־
      usb_irq_len[usb_irq_w_idx]  = 8;
      /* �е���һ��д��ַ */
      usb_irq_w_idx++;
      if(usb_irq_w_idx >= USB_IRQ_FLAG_NUM) usb_irq_w_idx = 0;

      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }
  }
  else if ( R8_USB_INT_FG & RB_UIF_BUS_RST )  // USB���߸�λ
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

    R8_USB_INT_FG = RB_UIF_BUS_RST;             // ���жϱ�־
  }
  else if (  R8_USB_INT_FG & RB_UIF_SUSPEND )  // USB���߹���/�������
  {
    if ( R8_USB_MIS_ST & RB_UMS_SUSPEND )    //����
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
    else                                     //����
    {
      Ep1DataINFlag = 1;
    }

    cdc_uart_sta_trans_step = 0;
    ven_ep1_trans_step = 0;

    R8_USB_INT_FG = RB_UIF_SUSPEND;
  }
}

void USB_IRQProcessHandler( void )   /* USB�жϷ������ */
{
    static  PUINT8  pDescr;
    static  uint8_t irq_idx = 0;
    uint8_t len;
    uint8_t   buf[8];
    uint8_t   data_dir = 0;   //���ݷ���
    uint8_t   ep_idx, ep_pid;
    uint8_t   i;
    uint8_t   ep_sta;
    uint32_t  bps;

    {
        i = usb_irq_r_idx;
      usb_irq_r_idx++;
      if(usb_irq_r_idx >= USB_IRQ_FLAG_NUM) usb_irq_r_idx = 0;
      switch ( usb_irq_pid[i] & 0x3f )   // �����������ƺͶ˵��
      {
        case UIS_TOKEN_IN | 4:  //endpoint 4# �����˵��ϴ����
        {
          break;
        }
        case UIS_TOKEN_IN | 3:  //endpoint 3# �����˵��ϴ����
        {
          break;
        }
        case UIS_TOKEN_OUT | 2:    // endpoint 2# �����˵��´����
        {
            Ep2DataOUTLen = usb_irq_len[i];
            gUartRxCount += Ep2DataOUTLen;
            Ep2DataOUTFlag = 1;
            break;
        }
        case UIS_TOKEN_IN | 2:  //endpoint 2# �����˵��ϴ����
        {
          break;
        }
        case UIS_TOKEN_OUT | 1:    // endpoint 1# �����˵��´����
        {
          PRINT("usb_out1\n");
          len = usb_irq_len[i];
          //Ep1OUTDataBuf
          for(int i = 0;i<len;i++)
          PRINT("%02x  ",Ep1OUTDataBuf[i]);
          PRINT("\n");

          //CH341�������·�
          Ep1DataOUTFlag = 1;
          Ep1DataOUTLen = len;
          VENSer0SendFlag = 1;
          PFIC_DisableIRQ(USB_IRQn);
          R8_UEP1_CTRL = R8_UEP1_CTRL & 0xf3; //OUT_ACK
          PFIC_EnableIRQ(USB_IRQn);
          break;
        }
        case UIS_TOKEN_IN | 1:   // endpoint 1# �ж϶˵��ϴ����
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

            len = 0;                                                 // Ĭ��Ϊ�ɹ������ϴ�0����
            SetupReqCode = UsbSetupBuf->bRequest;

            /* ���ݷ��� */
            data_dir = USB_REQ_TYP_OUT;
            if(UsbSetupBuf->bRequestType & USB_REQ_TYP_IN) data_dir = USB_REQ_TYP_IN;

            /* �������� */
            if( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_VENDOR )
            {
              switch(SetupReqCode)
              {
                case DEF_VEN_DEBUG_WRITE:  //д���� 0X9A
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

                  /* ���������ò����ʵļĴ��� */
                  if((write_reg_add1 == 0x12)&&(write_reg_add2 == 0x13))
                  {
                    /* �����ʴ�����ü���ֵ */
                    if((UsbSetupBuf->wIndexL==0x87)&&(UsbSetupBuf->wIndexH==0xf3))
                    {
                      bps = 921600;  //13 * 921600 = 11980800
                    }
                    else if((UsbSetupBuf->wIndexL==0x87)&&(UsbSetupBuf->wIndexH==0xd9))
                    {
                      bps = 307200;  //39 * 307200 = 11980800
                    }

                    //ϵͳ��Ƶ�� 36923077
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
                    //ϵͳ��Ƶ��  32000000
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
                        CalClock = 93750; //64 ��Ƶ
                        CalDiv = 0 - UsbSetupBuf->wIndexH;
                        bps = CalClock / CalDiv;
                      }
                      else if((UsbSetupBuf->wIndexL & 0x7f) == 0)
                      {
                        CalClock = 11719;  //Լ512
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
                case DEF_VEN_DEBUG_READ:   //��Ҫ�ش����� 0X95  /* ������Ĵ��� */
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
                //A1����Ҳ��Ҫ��ʼ�����ڵ�
                case DEF_VEN_UART_INIT:  //��ʼ������ 0XA1
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

                    //ֱ�����üĴ���
                   // UART0ParaSet(data_bit_val, stop_bit_val,parity_val);

                    uart_set_m = 0;
                    uart_reg1_val = UsbSetupBuf->wIndexL;
                    uart_reg2_val = UsbSetupBuf->wIndexH;

                    if(uart_reg1_val & (1<<6))  //�жϵ���λ
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
                      /* �����ʴ�����ü���ֵ */
                      if((uart_reg1_val == 0x87)&&(uart_reg2_val == 0xf3))
                      {
                        bps = 921600;  //13 * 921600 = 11980800
                      }
                      else if((uart_reg1_val == 0x87)&&(uart_reg2_val == 0xd9))
                      {
                        bps = 307200;  //39 * 307200 = 11980800
                      }

                      //ϵͳ��Ƶ�� 36923077
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
                      //ϵͳ��Ƶ��  32000000
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
                          CalClock = 93750; //64 ��Ƶ
                          CalDiv = 0 - uart_reg2_val;
                          bps = CalClock / CalDiv;
                        }
                        else if((uart_reg1_val & 0x7f) == 0)
                        {
                          CalClock = 11719;  //Լ512
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
                case DEF_VEN_UART_M_OUT:  //����MODEM�ź���� 0XA4
                {
                  len = 0;
                  Uart0Para.ioStaus = Ep0Buffer[2];
                  UART_Status = 1;
                  break;
                }
                case DEF_VEN_BUF_CLEAR: //0XB2  /* ���δ��ɵ����� */
                {
                  len = 0;

                  //�������³�ʼ��
                  VENSer0ParaChange = 1; // ���³�ʼ������������е�����
                  break;
                }
                case DEF_VEN_I2C_CMD_X:  //0X54  ����I2C�ӿڵ�����,����ִ��
                {
                  len = 0;
                  break;
                }
                case DEF_VEN_DELAY_MS:  //0X5E  ������Ϊ��λ��ʱָ��ʱ��
                {
                  len = 0;
                  break;
                }
                case DEF_VEN_GET_VER:   //0X5E   ��ȡоƬ�汾 //��Ҫ�ش�����-->�汾��
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
            // ��׼����
            else if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD)
            {
              switch( SetupReqCode )  // ������
              {
                case USB_GET_DESCRIPTOR:  //��ȡ������
                {
                  switch( UsbSetupBuf->wValueH )
                  {
                    case 1: // �豸������
                    {
#if ( USB_WORK_MODE== USB_VENDOR_MODE)
                      {
                        memcpy(ep0_send_buf,
                               &TAB_USB_VEN_DEV_DES[0],
                               sizeof( TAB_USB_VEN_DEV_DES ));
                        pDescr = ep0_send_buf;
                        len = sizeof( TAB_USB_VEN_DEV_DES );
                      }
#else                    //CDC��
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
                    case 2:  // ����������
                    {
#if ( USB_WORK_MODE== USB_VENDOR_MODE)
                      {
                        memcpy(ep0_send_buf,
                               &TAB_USB_VEN_CFG_DES[0],
                               sizeof( TAB_USB_VEN_CFG_DES ));
                        pDescr = ep0_send_buf;
                        len = sizeof( TAB_USB_VEN_CFG_DES );
                      }
#else                    //CDC��
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
                    case 3:  // �ַ���������
                    {
                      PRINT("str %d\r\n",UsbSetupBuf->wValueL);
#if ( USB_WORK_MODE== USB_VENDOR_MODE)
                      {
                        PRINT("str %d\r\n",UsbSetupBuf->wValueL);
                        switch(UsbSetupBuf->wValueL)
                        {
                          case 0:  //����������
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

                            /* ȡ���� */
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
                            len = 0xFF;    // ��֧�ֵ�����������
                            break;
                        }
                      }
#else   //CDCģʽ
                      {
                        PRINT("str %d\r\n",UsbSetupBuf->wValueL);
                        switch(UsbSetupBuf->wValueL)
                        {
                          case 0:  //����������
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

                            /* ȡ���� */
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
                            len = 0xFF;    // ��֧�ֵ�����������
                            break;
                        }
                      }
#endif
                      break;
                    }
                    case 6:  //�豸�޶�������
                    {
                      pDescr = (PUINT8)( &My_QueDescr[0] );
                      len = sizeof( My_QueDescr );
                      break;
                    }
                    default:
                      len = 0xFF;                                  // ��֧�ֵ�����������
                      break;
                  }
                  if ( SetupLen > len ) SetupLen = len;            // �����ܳ���
                  len = (SetupLen >= THIS_ENDP0_SIZE) ? THIS_ENDP0_SIZE : SetupLen;  // ���δ��䳤��
                  memcpy( Ep0Buffer, pDescr, len );                 /* �����ϴ����� */
                  SetupLen -= len;
                  pDescr += len;

                  break;
                }
                case USB_SET_ADDRESS:  //���õ�ַ
                {
                  PRINT("SET_ADDRESS:%d\r\n",UsbSetupBuf->wValueL);
                  devinf.gUsbFlag |= DEF_BIT_USB_ADDRESS;
                  devinf.UsbAddress = UsbSetupBuf->wValueL;    // �ݴ�USB�豸��ַ

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
                  /* ����豸 */
                  if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                  {
                    R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
                    R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
                    R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
                    R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;

                    //״̬������λ
                    Ep1DataINFlag = 1;
                    Ep1DataOUTFlag = 0;

                    cdc_uart_sta_trans_step = 0;
                    ven_ep1_trans_step = 0;
                  }
                  else if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )  // �˵�
                  {
                    switch( UsbSetupBuf->wIndexL )   //�ж϶˵�
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
                  else len = 0xFF;                                  // ���Ƕ˵㲻֧��

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
                  len = 0xFF;                                       // ����ʧ��
                  break;
              }
            }
            /* ������ */
            else if( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS )
            {
              /* �����´� */
              if(data_dir == USB_REQ_TYP_OUT)
              {
                switch( SetupReqCode )  // ������
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
                    /* ��·״̬ */
                    PRINT("ctl %02x %02x\r\n",Ep0Buffer[2],Ep0Buffer[3]);
                    carrier_sta = Ep0Buffer[2] & (1<<1);   //RTS״̬
                    present_sta = Ep0Buffer[2] & (1<<0);   //DTR״̬
                    len = 0;
                    break;
                  }
                  default:
                  {
                    PRINT("CDC ReqCode%x\r\n",SetupReqCode);
                    len = 0xFF;                                       // ����ʧ��
                    break;
                  }
                }
              }
              /* �豸�ϴ� */
              else
              {
                switch( SetupReqCode )  // ������
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
                    //SetupLen �ж��ܳ���
                    len = 2;
                    CDCSetSerIdx = 0;
                    Ep0Buffer[0] = 0;
                    Ep0Buffer[1] = 0;
                    break;
                  }
                  default:
                  {
                    PRINT("CDC ReqCode%x\r\n",SetupReqCode);
                    len = 0xFF;                                       // ����ʧ��
                    break;
                  }
                }
              }
            }

            else len = 0xFF;   /* ʧ�� */
          }
          else
          {
            len = 0xFF; // SETUP�����ȴ���
          }
          if ( len == 0xFF )  // ����ʧ��
          {
            SetupReqCode = 0xFF;
            PFIC_DisableIRQ(USB_IRQn);
            R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;  // STALL
            PFIC_EnableIRQ(USB_IRQn);
          }
          else if ( len <= THIS_ENDP0_SIZE )  // �ϴ����ݻ���״̬�׶η���0���Ȱ�
          {
            if( SetupReqCode ==  USB_SET_ADDRESS)  //���õ�ַ 0x05
            {
//              PRINT("add in:%d\r\n",len);
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //Ĭ�����ݰ���DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  USB_SET_CONFIGURATION)  //��������ֵ 0x09
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //Ĭ�����ݰ���DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  USB_GET_DESCRIPTOR)  //��ȡ������  0x06
            {
              R8_UEP0_T_LEN = len;
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;  //Ĭ�����ݰ���DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  DEF_VEN_UART_INIT )  //0XA1 ��ʼ������
            {
              R8_UEP0_T_LEN = len;
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //Ĭ�����ݰ���DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  DEF_VEN_DEBUG_WRITE )  //0X9A
            {
              R8_UEP0_T_LEN = len;
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //Ĭ�����ݰ���DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  DEF_VEN_UART_M_OUT )  //0XA4
            {
              R8_UEP0_T_LEN = len;
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //Ĭ�����ݰ���DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  DEF_SET_CONTROL_LINE_STATE )  //0x22
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //Ĭ�����ݰ���DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  USB_CLEAR_FEATURE )  //0x01
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //Ĭ�����ݰ���DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else
            {
              if(data_dir == USB_REQ_TYP_IN)   //��ǰ��Ҫ�ϴ�
              {
                PFIC_DisableIRQ(USB_IRQn);
                R8_UEP0_T_LEN = len;
                R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  //Ĭ�����ݰ���DATA1
                PFIC_EnableIRQ(USB_IRQn);
              }
              else                            //��ǰ��Ҫ�´�
              {
                PFIC_DisableIRQ(USB_IRQn);
                R8_UEP0_T_LEN = len;
                R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;  //Ĭ�����ݰ���DATA1
                PFIC_EnableIRQ(USB_IRQn);
              }
            }
          }
          else  // �´����ݻ�����
          {
            //��Ȼ��δ��״̬�׶Σ�������ǰԤ���ϴ�0�������ݰ��Է�������ǰ����״̬�׶�
            R8_UEP0_T_LEN = 0;
            PFIC_DisableIRQ(USB_IRQn);
            R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;  // Ĭ�����ݰ���DATA1
            PFIC_EnableIRQ(USB_IRQn);
          }
          break;
        }
        case UIS_TOKEN_IN | 0:      // endpoint 0# IN  UIS_TOKEN_IN
        {
          switch( SetupReqCode )
          {
            /* �򵥵Ĵ���SETUP������ */
            case USB_GET_DESCRIPTOR:  //0x06  ��ȡ������
            {
              len = (SetupLen >= THIS_ENDP0_SIZE) ? THIS_ENDP0_SIZE : SetupLen;  // ���δ��䳤��
              memcpy( Ep0Buffer, pDescr, len );                    /* �����ϴ����� */
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
            //���̶�ȡ
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
              R8_UEP0_T_LEN = 0;                                      // ״̬�׶�����жϻ�����ǿ���ϴ�0�������ݰ��������ƴ���
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
                /* ���ô��� */
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
* Description    : �˵�״̬���ú���
* Input          : ep_num���˵��
                   type���˵㴫������
                   sta���л��Ķ˵�״̬
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
* Description    : USB������ʼ��������ͱ�־
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
* Description    : ��ʼ��CDC�豸
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitCDCDevice(void)
{
  /* ��ʼ������ */
  USBParaInit();

  R8_USB_CTRL = 0x00;                                                 // ���趨ģʽ

//  1.�˵���䣺
//  �˵�0
//  �˵�1��IN��OUT  �����ݽӿڣ�
//  �˵�2��IN��OUT  �����ݽӿڣ�
//  �˵�3��IN       ���ӿ�23��ϣ��ж��ϴ���
//  �˵�4��IN       ���ӿ�01��ϣ��ж��ϴ���

  R8_UEP4_1_MOD = RB_UEP4_TX_EN|RB_UEP1_TX_EN|RB_UEP1_RX_EN;

  /* �� 64 �ֽڽ��ջ�����(OUT)���� 64 �ֽڷ��ͻ�������IN�� */
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_TX_EN;

  R16_UEP0_DMA = (UINT32)&Ep0Buffer[0];
  R16_UEP1_DMA = (UINT32)&Ep1Buffer[0];
  R16_UEP2_DMA = (UINT32)&Ep2Buffer[0];
  R16_UEP3_DMA = (UINT32)&Ep3Buffer[0];
  //R16_UEP4_DMA = (UINT16)(UINT32)&Ep2Buffer[0];

  /* �˵�0״̬��OUT--ACK IN--NAK */
  R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;

  /* �˵�1״̬��OUT--ACK IN--NAK �Զ���ת */
  R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  /* �˵�2״̬��OUT--ACK IN--NAK �Զ���ת */
  R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  /* �˵�3״̬��IN--NAK �Զ���ת */
  R8_UEP3_CTRL = UEP_T_RES_NAK;

  /* �˵�4״̬��IN--NAK �ֶ���ת */
  R8_UEP4_CTRL = UEP_T_RES_NAK;

  /* �豸��ַ */
  R8_USB_DEV_AD = 0x00;

  //��ֹDP/DM��������s
  R8_UDEV_CTRL = RB_UD_PD_DIS;

  //����USB�豸��DMA�����ж��ڼ��жϱ�־δ���ǰ�Զ�����NAK
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;

  //���жϱ�־
  R8_USB_INT_FG = 0xFF;

  //����ͳһ��ѯ��
  //�����ж�          ����            �������         ���߸�λ
  R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_TRANSFER | RB_UIE_BUS_RST;
//  R8_USB_INT_EN = RB_UIE_TRANSFER ;
  PFIC_EnableIRQ(USB_IRQn);

  //ʹ��USB�˿�
  R8_UDEV_CTRL |= RB_UD_PORT_EN;

  devinf.UsbConfig = 0;
  devinf.UsbAddress = 0;
}

/*******************************************************************************
* Function Name  : InitVendorDevice
* Description    : ��ʼ������USB�豸
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitVendorDevice(void)
{
  /* ��ʼ������ */
  USBParaInit();

  R8_USB_CTRL = 0x00;                                                 // ���趨ģʽ

//  1.�˵���䣺
//  �˵�0
//  �˵�1��IN��OUT  �����ýӿڣ�
//  �˵�2��IN��OUT  �����ݽӿڣ�

  /* �� 64 �ֽڽ��ջ�����(OUT)���� 64 �ֽڷ��ͻ�������IN�� */
  R8_UEP4_1_MOD = RB_UEP1_TX_EN | RB_UEP1_RX_EN;

  /* �� 64 �ֽڽ��ջ�����(OUT)���� 64 �ֽڷ��ͻ�������IN�� */
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN;

  R16_UEP0_DMA = (UINT32)&Ep0Buffer[0];
  R16_UEP1_DMA = (UINT32)&Ep1Buffer[0];
  R16_UEP2_DMA = (UINT32)&Ep2Buffer[0];

  /* �˵�0״̬��OUT--ACK IN--NAK */
  R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;

  /* �˵�1״̬��OUT--ACK IN--NAK �Զ���ת */
  R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  /* �˵�2״̬��OUT--ACK IN--NAK �Զ���ת */
  R8_UEP2_CTRL =  UEP_R_RES_ACK | UEP_T_RES_NAK;

  /* �豸��ַ */
  R8_USB_DEV_AD = 0x00;

  //��ֹDP/DM��������s
  R8_UDEV_CTRL = RB_UD_PD_DIS;

  //����USB�豸��DMA�����ж��ڼ��жϱ�־δ���ǰ�Զ�����NAK
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;

  //���жϱ�־
  R8_USB_INT_FG = 0xFF;

  //�����ж�          ����            �������         ���߸�λ
  R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_TRANSFER | RB_UIE_BUS_RST;
  PFIC_EnableIRQ(USB_IRQn);

  //ʹ��USB�˿�
  R8_UDEV_CTRL |= RB_UD_PORT_EN;

  devinf.UsbConfig = 0;
  devinf.UsbAddress = 0;
}

/*******************************************************************************
* Function Name  : InitUSBDevPara
* Description    : USB��صı�����ʼ��
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

  UART0_RTS_Val = 0; //��� ��ʾDTE����DCE��������
  UART0_DTR_Val = 0; //��� �����ն˾���

  for(i=0; i<USB_IRQ_FLAG_NUM; i++)
  {
    usb_irq_flag[i] = 0;
  }
}

/*******************************************************************************
* Function Name  : InitUSBDevice
* Description    : ��ʼ��USB
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
        /* CDCģʽ���� */
#else
        {
            if( Ep1DataINFlag )
            {
                len  = MAX_PACKET_SIZE;
                if( RF_RxQuery( &Ep1Buffer[0], &len ) )
                {
                    /* ֱ�ӷ������� */
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
 * @brief   ��ѯUSB�Ƿ���յ�����
 *
 * @return  0��������   0x80 ״̬��������  0xFF ���¼�
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
    // �رշ�����Խӿ�
    R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;
    InitUSBDevPara();
    InitUSBDevice();
    PFIC_EnableIRQ( USB_IRQn );
}


