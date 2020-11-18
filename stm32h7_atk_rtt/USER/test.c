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
     
    u8 led0sta=1;                //LED0ǰ״̬

    
    LED_Init();                      //��ʼ����LED���ӵ�Ӳ���ӿ�   
    LCD_Init();                        //��ʼ��LCD    
    xianchen1_thread_init();
    xianchen2_thread_init();
    while(1)
    {
        LED0(led0sta^=1);//��˸LED,��ʾϵͳ��������.
        //rt_hw_us_delay(500000);
        delay_ms(1000);
        time++;
    }
}
static void xianchen1_thread_init(void)
{
    /* �����߳� */
    rt_thread_t dymem_task_tid = rt_thread_create("xianchen1",/* �߳����� */
                            xianchen1_thread_entry, RT_NULL,
                            1024, 3, 10); //
    if(dymem_task_tid != RT_NULL)
    {
        /* �����߳� */
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
    /* �����߳� */
    rt_thread_t dymem_task_tid = rt_thread_create("xianchen2",/* �߳����� */
                            xianchen2_thread_entry, RT_NULL,
                            1024, 3, 10); //
    if(dymem_task_tid != RT_NULL)
    {
        /* �����߳� */
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













