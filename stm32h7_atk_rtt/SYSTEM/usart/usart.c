#include "sys.h"
#include "usart.h"      

//TDA发送数据寄存器
//RDR接收数据寄存器
//uart_init(100,115200);        频率（APB2ENR）和波特率

//如果使用ucos,则包括下面的头文件即可.
#if RT_THREAD_OS == 1
#include <string.h>

/******************************************中断shell********************************************************/
/* 第一部分：ringbuffer 实现部分 */
#define rt_ringbuffer_space_len(rb) ((rb)->buffer_size - rt_ringbuffer_data_len(rb))

struct rt_ringbuffer
{
    rt_uint8_t *buffer_ptr;

    rt_uint16_t read_mirror : 1;
    rt_uint16_t read_index : 15;
    rt_uint16_t write_mirror : 1;
    rt_uint16_t write_index : 15;

    rt_int16_t buffer_size;
};

enum rt_ringbuffer_state
{
    RT_RINGBUFFER_EMPTY,
    RT_RINGBUFFER_FULL,
    /* half full is neither full nor empty */
    RT_RINGBUFFER_HALFFULL,
};

rt_inline enum rt_ringbuffer_state rt_ringbuffer_status(struct rt_ringbuffer *rb)
{
    if (rb->read_index == rb->write_index)
    {
        if (rb->read_mirror == rb->write_mirror)
            return RT_RINGBUFFER_EMPTY;
        else
            return RT_RINGBUFFER_FULL;
    }
    return RT_RINGBUFFER_HALFFULL;
}

/** 
 * get the size of data in rb 
 */
rt_size_t rt_ringbuffer_data_len(struct rt_ringbuffer *rb)
{
    switch (rt_ringbuffer_status(rb))
    {
    case RT_RINGBUFFER_EMPTY:
        return 0;
    case RT_RINGBUFFER_FULL:
        return rb->buffer_size;
    case RT_RINGBUFFER_HALFFULL:
    default:
        if (rb->write_index > rb->read_index)
            return rb->write_index - rb->read_index;
        else
            return rb->buffer_size - (rb->read_index - rb->write_index);
    };
}

void rt_ringbuffer_init(struct rt_ringbuffer *rb,
                        rt_uint8_t           *pool,
                        rt_int16_t            size)
{
    RT_ASSERT(rb != RT_NULL);
    RT_ASSERT(size > 0);

    /* initialize read and write index */
    rb->read_mirror = rb->read_index = 0;
    rb->write_mirror = rb->write_index = 0;

    /* set buffer pool and size */
    rb->buffer_ptr = pool;
    rb->buffer_size = RT_ALIGN_DOWN(size, RT_ALIGN_SIZE);
}

/**
 * put a character into ring buffer
 */
rt_size_t rt_ringbuffer_putchar(struct rt_ringbuffer *rb, const rt_uint8_t ch)
{
    RT_ASSERT(rb != RT_NULL);

    /* whether has enough space */
    if (!rt_ringbuffer_space_len(rb))
        return 0;

    rb->buffer_ptr[rb->write_index] = ch;

    /* flip mirror */
    if (rb->write_index == rb->buffer_size-1)
    {
        rb->write_mirror = ~rb->write_mirror;
        rb->write_index = 0;
    }
    else
    {
        rb->write_index++;
    }

    return 1;
}
/**
 * get a character from a ringbuffer
 */
rt_size_t rt_ringbuffer_getchar(struct rt_ringbuffer *rb, rt_uint8_t *ch)
{
    RT_ASSERT(rb != RT_NULL);

    /* ringbuffer is empty */
    if (!rt_ringbuffer_data_len(rb))
        return 0;

    /* put character */
    *ch = rb->buffer_ptr[rb->read_index];

    if (rb->read_index == rb->buffer_size-1)
    {
        rb->read_mirror = ~rb->read_mirror;
        rb->read_index = 0;
    }
    else
    {
        rb->read_index++;
    }

    return 1;
}


/* 第二部分：finsh 移植对接部分 */
#define UART_RX_BUF_LEN 200
rt_uint8_t uart_rx_buf[UART_RX_BUF_LEN] = {0};
struct rt_ringbuffer  uart_rxcb;         /* 定义一个 ringbuffer cb */
static struct rt_semaphore shell_rx_sem; /* 定义一个静态信号量 */

/* 初始化串口，中断方式 */
static int uart_init(void)
{
    /* 初始化串口接收 ringbuffer  */
    rt_ringbuffer_init(&uart_rxcb, uart_rx_buf, UART_RX_BUF_LEN);

    /* 初始化串口接收数据的信号量 */
    rt_sem_init(&(shell_rx_sem), "shell_rx", 0, 0);

    /* 初始化串口参数，如波特率、停止位等等 */
    uart_init_sys(100,115200);        //串口初始化为115200

    return 0;
}
INIT_BOARD_EXPORT(uart_init);

/* 移植控制台，实现控制台输出, 对接 rt_hw_console_output */
void rt_hw_console_output(const char *str)
{
    /* 进入临界区 */
    rt_enter_critical();
    
    while(*str!='\0')
    {
        if( *str == '\n')
        {
            USART1->TDR = '\r';
            while((USART1->ISR&0X40)==0);//等待发送结束
        }
        USART1->TDR = *str++;
        while((USART1->ISR&0X40)==0);//等待发送结束
    }
    
    /* 退出临界区 */
    rt_exit_critical();
}

/* 移植 FinSH，实现命令行交互, 需要添加 FinSH 源码，然后再对接 rt_hw_console_getchar */
/* 中断方式 */
char rt_hw_console_getchar(void)
{
    char ch = 0;

    /* 从 ringbuffer 中拿出数据 */
    while (rt_ringbuffer_getchar(&uart_rxcb, (rt_uint8_t *)&ch) != 1)
    {
        rt_sem_take(&shell_rx_sem, RT_WAITING_FOREVER);
    } 
    return ch;   
}
void USART1_IRQHandler(void)
{
    int ch = -1;

    rt_interrupt_enter();          //在中断中一定要调用这对函数，进入中断
    
    if ( USART1->ISR&(1<<5) )
    {
        ch = USART1->RDR;
        /* 读取到数据，将数据存入 ringbuffer */
        rt_ringbuffer_putchar(&uart_rxcb, ch);
        rt_sem_release(&shell_rx_sem);
    }
    USART1->ICR|=1<<3;    //清除溢出错误,否则可能会卡死在串口中断服务函数里面
    rt_interrupt_leave();    //在中断中一定要调用这对函数，离开中断
} 
#else




//加入以下代码,支持printf函数,而不需要选择use MicroLIB      
#if 1
#pragma import(__use_no_semihosting)  
//解决HAL库使用时,某些情况可能报错的bug
int _ttywrch(int ch)    
{
    ch=ch;
    return ch;
}
//标准库需要的支持函数                 
struct __FILE 
{ 
    int handle; 
    /* Whatever you require here. If the only file you are using is */ 
    /* standard output using printf() for debugging, no file handling */ 
    /* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
    x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
    while((USART1->ISR&0X40)==0);//循环发送,直到发送完毕   
    USART1->TDR = (u8) ch;      
    return ch;
}
#endif 
//end
//////////////////////////////////////////////////////////////////

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误       
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，    接收完成标志
//bit14，    接收到0x0d
//bit13~0，    接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记      
  
void USART1_IRQHandler(void)
{
    u8 res;    
#if SYSTEM_SUPPORT_OS         //如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
    OSIntEnter();    
#endif
    if(USART1->ISR&(1<<5))//接收到数据
    {     
        res=USART1->RDR; 
        if((USART_RX_STA&0x8000)==0)//接收未完成
        {
            if(USART_RX_STA&0x4000)//接收到了0x0d
            {
                if(res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
                else USART_RX_STA|=0x8000;    //接收完成了 
            }else //还没收到0X0D
            {    
                if(res==0x0d)USART_RX_STA|=0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA&0X3FFF]=res;
                    USART_RX_STA++;
                    if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收      
                }         
            }
        }                                                    
    } 
    USART1->ICR|=1<<3;    //清除溢出错误,否则可能会卡死在串口中断服务函数里面
#if SYSTEM_SUPPORT_OS     //如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
    OSIntExit();                                               
#endif
} 
#endif  
#endif          //RT Thread
//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//注意:USART1的时钟,是可以通过RCC_D2CCIP2R选择的
//但是我们一般用默认的选择即可(默认选择rcc_pclk2作为串口1时钟)
//pclk2即APB2的时钟频率,一般为100Mhz
//bound:波特率 
/////////////////////////////////////小数计算例如0.5 0.5*16将该值放入BRR低四位
//BRR                串口波特率设置            当OVER8 = 0时，0:3小数            4:15整数        当OVER8 = 1时，0:2小数            4:15整数，位3需要保持清零
//CR1                串口控制寄存器1        15：OVER8
//CR2                串口控制寄存器2
//
//
//
//BRR计算方式   串口时钟（*1M） / ( 波特率 * 16 );然后得到小数部分乘16放入BRR,然后在计算整数部分放入BRR
//时钟
//----------------------------------------------------------------------------------------------
//APB2ENR        位4         U1    |        位5            U6    |                            |                            |                            |                            |
//APB1ENR        位17        U2    |        位18        U3    |    位19        U4    |    位20        U5    |    位30        U7    |    位31        U8    |
//----------------------------------------------------------------------------------------------
//AF
//AF7:SPI2/3/6/USART1~3/6/UART7/SDIO1;            AF8:USART4/5/8/SPDIF/SAI2/4;

//发送配置过程
//设定CR1的    M位定义字长                        位28  M1    位12    M0        配合组成2位        00，8个数据位    01，9个数据位    10，7个数据位
//设定CR1的    STOP位定义停止位位数        
//设定CR1的    OVER8位过采样模式            位15    1，8倍        0,16倍，通常16倍（只有当UE为0时才可以写入）
//设定BRR        设定波特率
//设定CR1的    UE位使能                                位0        UE
//DMA配置
//设定CR1的    TE位使能发送位                    位3        TE
//向TDR写入数据
//写完一个数据后等待ISR寄存器的TC位置1，完成发送再发送下一个        位6        TC

//接收配置过程
//设定CR1的    M位定义字长                        位28  M1    位12    M0        配合组成2位        00，8个数据位    01，9个数据位    10，7个数据位
//设定CR1的    STOP位定义停止位位数        
//设定CR1的    OVER8位过采样模式            位15    1，8倍        0,16倍，通常16倍（只有当UE为0时才可以写入）
//设定BRR        设定波特率
//设定CR1的    UE位使能                                位0        UE
//DMA配置
//设定CR1的    RE位使能发送位                    位2        RE
//设定CR1的    RXNEIE/RXFNEIE接收缓冲区非空中断使能      位5        RXNEIE/RXFNEIE
//当接收到数据
//ISR寄存器的RXNE位会置1此时就表示数据已经传输到RDR寄存器中了。
//若开启中断则的RXNEIE位会被置1，触发中断
//若开启其他错误检测，则相应的位会置1
//读取RDR寄存器会使的RXNE位置0
//

void uart_init_sys(u32 pclk2,u32 bound)
{       
    u32    temp;       
    temp=(pclk2*1000000+bound/2)/bound;    //得到USARTDIV@OVER8=0,采用四舍五入计算
    RCC->AHB4ENR|=1<<0;       //使能PORTA口时钟  
    RCC->APB2ENR|=1<<4;      //使能串口1时钟 
    GPIO_Set(GPIOA,PIN9|PIN10,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_MID,GPIO_PUPD_PU);//PA9,PA10,复用功能,上拉输出
     GPIO_AF_Set(GPIOA,9,7);    //PA9,AF7
    GPIO_AF_Set(GPIOA,10,7);//PA10,AF7         
    //波特率设置
     USART1->BRR=temp;         //波特率设置@OVER8=0     
    USART1->CR1=0;             //清零CR1寄存器
    USART1->CR1|=0<<28;         //设置M1=0
    USART1->CR1|=0<<12;         //设置M0=0&M1=0,选择8位字长 
    USART1->CR1|=0<<15;     //设置OVER8=0,16倍过采样 
    USART1->CR1|=1<<3;      //串口发送使能 
#if EN_USART1_RX              //如果使能了接收
    //使能接收中断 
    USART1->CR1|=1<<2;      //串口接收使能
    USART1->CR1|=1<<5;        //接收缓冲区非空中断使能            
    MY_NVIC_Init(3,3,USART1_IRQn,2);//组2，最低优先级 
#endif
    USART1->CR1|=1<<0;      //串口使能
}














