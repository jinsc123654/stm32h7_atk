/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-07-24     Tanek        the first version
 * 2018-11-12     Ernest Chen  modify copyright
 */
 
#include <stdint.h>
#include <rthw.h>
#include "sys.h"
#include "led.h"  
#include "usart.h"
#include "sdram.h"
#include "ltdc.h" 
#include "lcd.h" 

#ifdef __LTDC_H
#define SDRAM_FREE_ADDR ( 0XC0000000 + 1280 * 800 * 2 )
//占用2.048M
#else

#define SDRAM_FREE_ADDR 0XC0000000 

#endif

#define _SCB_BASE       (0xE000E010UL)
#define _SYSTICK_CTRL   (*(rt_uint32_t *)(_SCB_BASE + 0x0))
#define _SYSTICK_LOAD   (*(rt_uint32_t *)(_SCB_BASE + 0x4))
#define _SYSTICK_VAL    (*(rt_uint32_t *)(_SCB_BASE + 0x8))
#define _SYSTICK_CALIB  (*(rt_uint32_t *)(_SCB_BASE + 0xC))
#define _SYSTICK_PRI    (*(rt_uint8_t  *)(0xE000ED23UL))

// Updates the variable SystemCoreClock and must be called 
// whenever the core clock is changed during program execution.
extern void SystemCoreClockUpdate(void);

// Holds the system core clock, which is the system clock 
// frequency supplied to the SysTick timer and the processor 
// core clock.

extern struct rt_memheap sram_heap;
extern struct rt_memheap sdram_heap;
#define RT_AXI_SRAM_BEGIN rt_heap2
#define RT_AXI_SRAM_SIZE 1000
static uint32_t rt_heap2[RT_AXI_SRAM_SIZE]; 








extern uint32_t SystemCoreClock;


static uint32_t _SysTick_Config(rt_uint32_t ticks)
{
    if ((ticks - 1) > 0xFFFFFF)
    {
        return 1;
    }
    
    _SYSTICK_LOAD = ticks - 1; 
    _SYSTICK_PRI = 0xFF;
    _SYSTICK_VAL  = 0;
    _SYSTICK_CTRL = 0x07;  
    
    return 0;
}

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE ( 29500 * 1024 / 4 )
//C01F4000
static uint32_t rt_heap[RT_HEAP_SIZE] __attribute__((at(SDRAM_FREE_ADDR))); 
//static uint32_t rt_heap[RT_HEAP_SIZE];     // heap default size: 4K(1024 * 4)
RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

/**
 * This function will initial your board.
 */
void rt_hw_board_init()
{
    /* System Clock Update */
    Stm32_Clock_Init(160,5,2,4);//设置时钟,400Mhz

    /* System Tick Configuration */
    _SysTick_Config( SystemCoreClock / RT_TICK_PER_SECOND);
    SDRAM_Init();
    
    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
    rt_memheap_init(&sram_heap,"SRAM",&rt_heap2[0],1000);
}

void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}
