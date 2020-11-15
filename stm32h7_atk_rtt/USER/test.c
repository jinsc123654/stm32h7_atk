#include "sys.h"
#include "led.h"  
#include "usart.h" 
#include "delay.h"
#include "ltdc.h" 
#include "lcd.h" 



static void xianchen1_thread_init(void);
static void xianchen1_thread_entry(void);
static void xianchen2_thread_init(void);
static void xianchen2_thread_entry(void);

u32 time = 0;
int main(void)
{ 
     
    u8 led0sta=1;                //LED0前状态

    
    LED_Init();                      //初始化与LED连接的硬件接口   
    LCD_Init();                        //初始化LCD    
    xianchen1_thread_init();
    xianchen2_thread_init();
    while(1)
    {
        LED0(led0sta^=1);//闪烁LED,提示系统正在运行.
        //rt_hw_us_delay(500000);
        delay_ms(1000);
        time++;
    }
}
static void xianchen1_thread_init(void)
{
    /* 创建线程 */
    rt_thread_t dymem_task_tid = rt_thread_create("xianchen1",/* 线程名称 */
                            xianchen1_thread_entry, RT_NULL,
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


static void xianchen1_thread_entry(void)
{   
    u32 i = 0;
    while(1)
    {
        
        LCD_ShowString(10,40,240,32,32,"xianchen1");
        LCD_ShowxNum( 210, 40, i++, 8, 32, 0 );
        delay_ms(1000);
    }
}

static void xianchen2_thread_init(void)
{
    /* 创建线程 */
    rt_thread_t dymem_task_tid = rt_thread_create("xianchen2",/* 线程名称 */
                            xianchen2_thread_entry, RT_NULL,
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


static void xianchen2_thread_entry(void)
{   
    u32 i = 0;
    while(1)
    {
        
        LCD_ShowString(10,80,240,32,32,"xianchen2");
        LCD_ShowxNum( 210, 80, i+=2, 8, 32, 0 );
        delay_ms(1000);
    }
}













