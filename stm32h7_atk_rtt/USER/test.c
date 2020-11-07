#include "sys.h"
#include "led.h"  
#include "usart.h" 
#include "delay.h"

static void led_thread_init(void);
static void led_thread_entry(void);


u32 time = 0;
int main(void)
{ 
    
    u8 led0sta=1;                //LED0ǰ״̬
    led_thread_init();
    
    LED_Init();                      //��ʼ����LED���ӵ�Ӳ���ӿ�   
    while(1)
    {
        LED0(led0sta^=1);//��˸LED,��ʾϵͳ��������.
        //rt_hw_us_delay(500000);
        delay_ms(1000);
        time++;
    }
}
static void led_thread_init(void)
{
    /* �����߳� */
    rt_thread_t led_task_tid = rt_thread_create("led4",/* �߳����� */
                            led_thread_entry, RT_NULL,
                            1024, 3, 10); //
    if(led_task_tid != RT_NULL)
    {
        /* �����߳� */
        rt_thread_startup(led_task_tid);
        rt_kprintf("LED4 thread is already started\n");
    }
    else
    {
        rt_kprintf("LED4 thread is not started\n");
    }
}

int hello_world(void)
{
    rt_kprintf("%s\n", "hello world!");

    return 0;
}
MSH_CMD_EXPORT(hello_world, printf hello world);
static void led_thread_entry(void)
{   
    rt_kprintf( "%d\r", time );
    while(1)
    {
        
        delay_ms( 500 );
    }
}

















