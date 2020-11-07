#include "sys.h"
#include "led.h"  
#include "usart.h" 
#include "delay.h"
#include "sdram.h"
#include "ltdc.h" 
#include "lcd.h" 

static void dymem_thread_init(void);
static void dymem_thread_entry(void);

u32 time = 0;
int main(void)
{ 
    
    u8 led0sta=1;                //LED0前状态
    dymem_thread_init();
    
    LED_Init();                      //初始化与LED连接的硬件接口   

    LCD_Init();                        //初始化LCD
    LCD_ShowString(10,40,240,32,32,"STM32H7 RTTHREAD");     
    
    while(1)
    {
        LED0(led0sta^=1);//闪烁LED,提示系统正在运行.
        //rt_hw_us_delay(500000);
        delay_ms(1000);
        time++;
    }
}
static void dymem_thread_init(void)
{
    /* 创建线程 */
    rt_thread_t dymem_task_tid = rt_thread_create("dymem",/* 线程名称 */
                            dymem_thread_entry, RT_NULL,
                            1024, 3, 10); //
    if(dymem_task_tid != RT_NULL)
    {
        /* 启动线程 */
        rt_thread_startup(dymem_task_tid);
        rt_kprintf("LED4 thread is already started\n");
    }
    else
    {
        rt_kprintf("LED4 thread is not started\n");
    }
}

int hello_world(void)
{
    rt_kprintf("%s\n", "hello world!"); delay_ms(1000); rt_kprintf("%s\n", "hello world!"); delay_ms(1000);
    rt_enter_critical();

    rt_exit_critical();

    return 0;
}
MSH_CMD_EXPORT(hello_world, printf hello world);
static void dymem_thread_entry(void)
{   
    while(1)
    {

        delay_ms(1000);
    }
}














