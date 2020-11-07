#include "delay.h"
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////      
//如果使用ucos,则包括下面的头文件即可.
#if RT_THREAD_OS == 1
#include <rtthread.h>

void delay_init(u16 SYSCLK )
{
    SYSCLK = SYSCLK;
    return ;
}

void delay_us(rt_uint32_t us)
{
    rt_uint32_t start, now, delta, reload, us_tick;
    if( us > 1000 )
    {
       us = 999;
    }
    start = SysTick->VAL;
    reload = SysTick->LOAD;
    us_tick = SystemCoreClock / 1000000UL;//比如系统时钟频率是80M 那么1us 需要80个周期
    do {
        now = SysTick->VAL;
        delta = start > now ? start - now : reload + start - now;//是倒数 还是正数
    } while(delta < us_tick * us);
}


#else

static u16  fac_us=0;                            //us延时倍乘数               
//延时nus
//nus为要延时的us数.    
//注意:nus的值,不要大于41943us(最大值即2^24/fac_us@fac_us=400)
void delay_us(u32 nus)
{        
    u32 temp;             
    SysTick->LOAD=nus*fac_us;                 //时间加载               
    SysTick->VAL=0x00;                        //清空计数器
    SysTick->CTRL|=1<<0 ;                      //开始倒数  
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));    //等待时间到达   
    SysTick->CTRL&=~(1<<0) ;                   //关闭SYSTICK
    SysTick->VAL =0X00;                       //清空计数器 
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=2^24 *1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对400M条件下,nms<=41.94ms 
void delay_xms(u16 nms)
{                     
    u32 temp=0;           
    SysTick->LOAD=(u32)nms*fac_us*1000;        //时间加载(SysTick->LOAD为24bit)
    SysTick->VAL =0x00;                       //清空计数器
    SysTick->CTRL|=1<<0 ;                      //开始倒数  
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));    //等待时间到达   
    SysTick->CTRL&=~(1<<0) ;                   //关闭SYSTICK
    SysTick->VAL =0X00;                       //清空计数器              
} 
//延时nms 
//nms:0~65535
void delay_ms(u16 nms)
{          
    u16 repeat=nms/30;                        //这里用30,是考虑到可能有超频应用,
                                            //比如500Mhz的时候,delay_xms最大只能延时33.5ms左右了
    u16 remain=nms%30;
    while(repeat)
    {
        delay_xms(30);
        repeat--;
    }
    if(remain)delay_xms(remain);
} 

#endif





































