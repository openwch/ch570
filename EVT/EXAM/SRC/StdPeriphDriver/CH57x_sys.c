/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH57x_SYS.c
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

/*********************************************************************
 * @fn      SetSysClock
 *
 * @brief   配置系统运行时钟
 *
 * @param   sc      - 系统时钟源选择 refer to SYS_CLKTypeDef
 *
 * @return  none
 */
__HIGH_CODE
void SetSysClock(SYS_CLKTypeDef sc)
{
    uint16_t clk_sys_cfg;
    uint8_t i;
    uint8_t x32M_c;

    if(sc == RB_CLK_SYS_MOD)  // LSI
    {
        sys_safe_access_enable();
        R8_SLP_POWER_CTRL |= 0x40;
        R8_CLK_SYS_CFG |= RB_CLK_SYS_MOD;
        sys_safe_access_disable();
    }
    else
    {
        if(!(R8_HFCK_PWR_CTRL & RB_CLK_XT32M_PON))
        {
            x32M_c = R8_XT32M_TUNE;
            sys_safe_access_enable();
            R8_XT32M_TUNE |= 0x03;
            R8_HFCK_PWR_CTRL |= RB_CLK_XT32M_PON;
            sys_safe_access_disable();
            clk_sys_cfg = R8_CLK_SYS_CFG;
            sys_safe_access_enable();
            R8_CLK_SYS_CFG |= 0xC0;
            sys_safe_access_disable();
            for(i=0; i<9; i++)
            {
                __nop();
            }
            sys_safe_access_enable();
            R8_CLK_SYS_CFG = clk_sys_cfg;
            R8_XT32M_TUNE = x32M_c;
            sys_safe_access_disable();
        }

        if((sc & RB_CLK_SYS_MOD) == 0x40) // PLL div
        {
            sys_safe_access_enable();
            R8_HFCK_PWR_CTRL |= RB_CLK_PLL_PON;
            R8_FLASH_CFG = 0X01;
            R8_FLASH_SCK |= 1<<4; //50M
            sys_safe_access_disable();
        }
        else    // 32M div
        {
            if((sc&0x1F) )
            {
                sys_safe_access_enable();
                R8_FLASH_CFG = 0X02;
                sys_safe_access_disable();
            }
            else
            {
                sys_safe_access_enable();
                R8_FLASH_CFG = 0X07;
                sys_safe_access_disable();
            }
        }

        sys_safe_access_enable();
        R8_SLP_POWER_CTRL |= 0x40;
        R8_CLK_SYS_CFG = sc;
        sys_safe_access_disable();
    }
}

/*********************************************************************
 * @fn      GetSysClock
 *
 * @brief   获取当前系统时钟
 *
 * @param   none
 *
 * @return  Hz
 */
uint32_t GetSysClock(void)
{
    if((R8_CLK_SYS_CFG & RB_CLK_SYS_MOD) == RB_CLK_SYS_MOD)
    { // 32K做主频
        return (Freq_LSI);
    }
    else if((R8_CLK_SYS_CFG & RB_CLK_SYS_MOD) == 0x40)
    {
        return (600000000 / ((R8_CLK_SYS_CFG & 0x1f)?(R8_CLK_SYS_CFG & 0x1f):32));
    }
    else
    { // 32M进行分频
        return (32000000 / ((R8_CLK_SYS_CFG & 0x1f)?(R8_CLK_SYS_CFG & 0x1f):32));
    }
}

/*********************************************************************
 * @fn      SYS_GetInfoSta
 *
 * @brief   获取当前系统信息状态
 *
 * @param   i       - refer to SYS_InfoStaTypeDef
 *
 * @return  是否开启
 */
uint8_t SYS_GetInfoSta(SYS_InfoStaTypeDef i)
{
    if(i == STA_SAFEACC_ACT)
    {
        return (R8_SAFE_ACCESS_SIG & RB_SAFE_ACC_ACT);
    }
    else
    {
        return (R8_GLOB_CFG_INFO & (1 << i));
    }
}

/*********************************************************************
 * @fn      SYS_ResetExecute
 *
 * @brief   执行系统软件复位
 *
 * @param   none
 *
 * @return  none
 */
__HIGH_CODE
void SYS_ResetExecute(void)
{
    FLASH_ROM_SW_RESET();
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      SYS_DisableAllIrq
 *
 * @brief   关闭所有中断，并保留当前中断值
 *
 * @param   pirqv   - 当前保留中断值
 *
 * @return  none
 */
void SYS_DisableAllIrq(uint32_t *pirqv)
{
    *pirqv = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    PFIC->IRER[0] = 0xffffffff;
    PFIC->IRER[1] = 0xffffffff;
    asm volatile("fence.i");
}

/*********************************************************************
 * @fn      SYS_RecoverIrq
 *
 * @brief   恢复之前关闭的中断值
 *
 * @param   irq_status  - 当前保留中断值
 *
 * @return  none
 */
void SYS_RecoverIrq(uint32_t irq_status)
{
    PFIC->IENR[0] = (irq_status << 8);
    PFIC->IENR[1] = (irq_status >> 24);
}

/*********************************************************************
 * @fn      SYS_GetSysTickCnt
 *
 * @brief   获取当前系统(SYSTICK)计数值
 *
 * @param   none
 *
 * @return  当前计数值
 */
uint32_t SYS_GetSysTickCnt(void)
{
    return SysTick->CNTL;
}

/*********************************************************************
 * @fn      WWDG_ITCfg
 *
 * @brief   看门狗定时器溢出中断使能
 *
 * @param   s       - 溢出是否中断
 *
 * @return  none
 */
void WWDG_ITCfg(FunctionalState s)
{
    uint8_t ctrl = R8_RST_WDOG_CTRL;

    if(s == DISABLE)
    {
        ctrl &= ~RB_WDOG_INT_EN;
    }
    else
    {
        ctrl |= RB_WDOG_INT_EN;
    }

    sys_safe_access_enable();
    R8_RST_WDOG_CTRL = ctrl;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      WWDG_ResetCfg
 *
 * @brief   看门狗定时器复位功能
 *
 * @param   s       - 溢出是否复位
 *
 * @return  none
 */
void WWDG_ResetCfg(FunctionalState s)
{
    uint8_t ctrl = R8_RST_WDOG_CTRL;

    if(s == DISABLE)
    {
        ctrl &= ~RB_WDOG_RST_EN;
    }
    else
    {
        ctrl |= RB_WDOG_RST_EN;
    }

    sys_safe_access_enable();
    R8_RST_WDOG_CTRL = ctrl;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      WWDG_ClearFlag
 *
 * @brief   清除看门狗中断标志，重新加载计数值也可清除
 *
 * @param   none
 *
 * @return  none
 */
void WWDG_ClearFlag(void)
{
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_WDOG_INT_FLAG;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   硬件错误中断，进入后执行复位，复位类型为上电复位
 *
 * @param   none
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
__attribute__((weak))
void HardFault_Handler(void)
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
 * @fn      NMI_Handler
 *
 * @brief   不可屏蔽中断，电压监控生效时进入
 *
 * @param   none
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
__attribute__((weak))
void NMI_Handler(void)
{
    while(1);
}

/* 轮询一次大约需要 15~30 个 SysTick 时钟，周期比这更短就没法用SysTick计时 */ 
#define SYS_SYSTICK_MIN_PERIOD 1000

/*********************************************************************
 * @fn      SysTickWrapTicks
 *
 * @brief   计数器从起点数回起点所需的 SysTick 时钟数。
 *          自动重装载（STRE = 1）时，计数到 CMP（向上）或 0（向下）就重新 
 *          开始，所以一个周期是 CMP + 1 个时钟。自由运行（STRE = 0， 
 *          或者 CMP = 0xFFFFFFFF）时数到 2^32 才回起点，这里用 0 表示； 
 *          此时 CMP + 1 也会溢出成 0。
 *
 * @return  一个周期的时钟数（0 表示 2^32）
 */
__HIGH_CODE
static uint32_t SysTickWrapTicks(void)
{
    if((SysTick->CTLR & SysTick_CTLR_STRE) == 0)
    {
        return 0; /* 自由运行：2^32 */
    }
    return SysTick->CMP + 1; /* 此时也是 2^32 */
}

/*********************************************************************
 * @fn      SysTickCount
 *
 * @brief   读取计数器，并转换成一个只增不减的值，这样等待时就不用关心计数 
 *          方向：MODE = 0 向上计数，直接用 CNTL；MODE = 1 向下计数，用 
 *          （周期 - 1）- CNTL 把方向反过来。
 *
 * @return  转换后的计数值
 */
__HIGH_CODE
static uint32_t SysTickCount(void)
{
    if(SysTick->CTLR & SysTick_CTLR_MODE) /* bit4 = 1：向下计数 */
    {
        return (SysTickWrapTicks() - 1) - SysTick->CNTL;
    }
    return SysTick->CNTL; /* bit4 = 0：向上计数 */
}

/*********************************************************************
 * @fn      SysTickDelay
 *
 * @brief   等待指定数量的 SysTick 时钟。整个过程只读取 SysTick（CTLR/CMP/ 
 *          CNTL），从不写入 CTLR/CMP/SR，所以应用自己用的 SysTick 中断、 
 *          它的周期和标志都不会被影响。 
 *
 * @param   ticks   - 需要等待的 SysTick 时钟数
 *
 * @return  1       - 已经用 SysTick 完成延时
 *          0       - SysTick 不能用于计时（没有启动、周期太短没法轮询、或 
 *                   者等待过程中被关掉），调用者改用原来的空循环
*/
__HIGH_CODE
static uint8_t SysTickDelay(uint64_t ticks)
{
    if((ticks == 0) || ((SysTick->CTLR & SysTick_CTLR_STE) == 0))
    {
        return 0; /* 计数器没有在跑 */
    }

    uint32_t period = SysTickWrapTicks();
    if((period != 0) && (period < SYS_SYSTICK_MIN_PERIOD))
    {
        return 0; /* 周期太短，轮询来不及 */
    }

    uint32_t prev = SysTickCount();
    uint64_t tcnt = 0;
    while(tcnt < ticks)
    {
        if((SysTick->CTLR & SysTick_CTLR_STE) == 0)
        {
            return 0; /* 等待过程中被关掉了 */
        }

        uint32_t now = SysTickCount();
        if(now == prev)
        {
            continue; /* 读数没变，继续等 */
        }

        /* 转换后的值是递增的，差值就是经过的时钟数；如果反而变小了，说明中间
         * 计数器回过一次起点，要补上回到起点之前剩下的部分 */
        uint32_t step = (now > prev) ? (now - prev) : (period - prev + now);
        tcnt += step;
        prev = now;
    }
    return 1;
}

/*********************************************************************
 * @fn      SysTimeToTicks
 *
 * @brief   把 t（单位由 unitUs 指定）换算成延时需要等待的 SysTick 时钟数。
 *          频率取自 GetSysClock()；STCLK = 0 表示时基为 HCLK/8。结果向上取整，
 *          保证延时不会比要求的短。
 *
 * @param   t       - 时间数值
 * @param   unitUs  - t 的一个单位等于多少微秒：t 是 us 时填 1，是 ms 时填
 *                   1000
 *
 * @return  SysTick 时钟数
 */
__HIGH_CODE
static uint64_t SysTimeToTicks(uint16_t t, uint32_t unitUs)
{
    uint32_t sys_clock = GetSysClock();
    uint64_t ticks;

    if((SysTick->CTLR & SysTick_CTLR_STCLK) == 0)
    {
        sys_clock >>= 3; /* STCLK = 0：时基为 HCLK/8 */
    }

    /* 向上取整，保证延时不会比要求的短 */
    ticks = ((((uint64_t)sys_clock * t) * unitUs) + (1000000 - 1)) / 1000000;

    return ticks ? ticks : 1; /* 至少延时 1 个时钟 */
}

/*********************************************************************
 * @fn      loopDelayus
 *
 * @brief   uS 延时
 *
 * @param   t       - 时间参数
 *
 * @return  none
 */
__HIGH_CODE
static void loopDelayus(uint16_t t)
{
    uint32_t i;
#if(FREQ_SYS == 100000000)
    i = t * 25;
#elif(FREQ_SYS == 75000000)
    i = (t * 25)>>1;
#elif(FREQ_SYS == 60000000)
    i = t * 10;
#elif(FREQ_SYS == 50000000)
    i = (t * 25)>>1;
#elif(FREQ_SYS == 40000000)
    i = t * 10;
#elif(FREQ_SYS == 30000000)
    i = t * 5;
#elif(FREQ_SYS == 25000000)
    i = (t * 25)>>2;
#elif(FREQ_SYS == 24000000)
    i = t * 6;
#elif(FREQ_SYS == 20000000)
    i = t * 5;
#elif(FREQ_SYS == 16000000)
    i = t << 2;
#elif(FREQ_SYS == 8000000)
    i = t << 1;
#elif(FREQ_SYS == 6400000)
    i = t * 8 / 5;
#elif(FREQ_SYS == 4000000)
    i = t;
#elif(FREQ_SYS == 2000000)
    i = t >> 1;
#elif(FREQ_SYS == 1000000)
    i = t >> 2;
#else
    i = t;
#endif

#if((FREQ_SYS == 75000000)||\
    (FREQ_SYS == 60000000)||\
    (FREQ_SYS == 30000000))
    while(--i)
    {
        __nop();
    };
#else
    do
    {
        __nop();
    }while(--i);
#endif
}

/*********************************************************************
 * @fn      mDelayuS
 *
 * @brief   uS 延时
 *
 * @param   t       - 时间参数
 *
 * @return  none
 */
__HIGH_CODE
void mDelayuS(uint16_t t)
{
    if(t == 0)
    {
        return;
    }

    if(SysTickDelay(SysTimeToTicks(t, 1)) == 0)
    {
        loopDelayus(t); /* SysTick unavailable */
    }
}

/*********************************************************************
 * @fn      mDelaymS
 *
 * @brief   mS 延时
 *
 * @param   t       - 时间参数
 *
 * @return  none
 */
__HIGH_CODE
void mDelaymS(uint16_t t)
{
    if(t == 0)
    {
        return;
    }

    if(SysTickDelay(SysTimeToTicks(t, 1000)) == 0)
    {
        do
        {
            loopDelayus(1000); /* SysTick unavailable */
        }
        while(--t);
    }
}

#ifdef DEBUG
int _write(int fd, char *buf, int size)
{
    int i;
    for(i = 0; i < size; i++)
    {
        while(R8_UART_TFC == UART_FIFO_SIZE);                  /* 等待数据发送 */
            R8_UART_THR = *buf++; /* 发送数据 */
    }
    return size;
}

#endif

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
__attribute__((used))
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || ((uint32_t)curbrk + incr > (__get_SP() - 64)))
    return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}

/*********************************************************************
 * @fn      __wrap_memcpy
 *
 * @brief   wrap memcpy
 *
 * @return  dst
 */
__HIGH_CODE
void *__wrap_memcpy(void *dst, void *src, size_t size)
{
    __MCPY(dst, src, (void *)((uint32_t)src+size));
    return dst;
}
