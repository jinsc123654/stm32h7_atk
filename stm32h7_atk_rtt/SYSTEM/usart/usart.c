#include "sys.h"
#include "usart.h"      

//TDA�������ݼĴ���
//RDR�������ݼĴ���
//uart_init(100,115200);        Ƶ�ʣ�APB2ENR���Ͳ�����

//���ʹ��ucos,����������ͷ�ļ�����.
#if RT_THREAD_OS == 1
#include <string.h>

/******************************************�ж�shell********************************************************/
/* ��һ���֣�ringbuffer ʵ�ֲ��� */
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


/* �ڶ����֣�finsh ��ֲ�ԽӲ��� */
#define UART_RX_BUF_LEN 200
rt_uint8_t uart_rx_buf[UART_RX_BUF_LEN] = {0};
struct rt_ringbuffer  uart_rxcb;         /* ����һ�� ringbuffer cb */
static struct rt_semaphore shell_rx_sem; /* ����һ����̬�ź��� */

/* ��ʼ�����ڣ��жϷ�ʽ */
static int uart_init(void)
{
    /* ��ʼ�����ڽ��� ringbuffer  */
    rt_ringbuffer_init(&uart_rxcb, uart_rx_buf, UART_RX_BUF_LEN);

    /* ��ʼ�����ڽ������ݵ��ź��� */
    rt_sem_init(&(shell_rx_sem), "shell_rx", 0, 0);

    /* ��ʼ�����ڲ������粨���ʡ�ֹͣλ�ȵ� */
    uart_init_sys(100,115200);        //���ڳ�ʼ��Ϊ115200

    return 0;
}
INIT_BOARD_EXPORT(uart_init);

/* ��ֲ����̨��ʵ�ֿ���̨���, �Խ� rt_hw_console_output */
void rt_hw_console_output(const char *str)
{
    /* �����ٽ��� */
    rt_enter_critical();
    
    while(*str!='\0')
    {
        if( *str == '\n')
        {
            USART1->TDR = '\r';
            while((USART1->ISR&0X40)==0);//�ȴ����ͽ���
        }
        USART1->TDR = *str++;
        while((USART1->ISR&0X40)==0);//�ȴ����ͽ���
    }
    
    /* �˳��ٽ��� */
    rt_exit_critical();
}

/* ��ֲ FinSH��ʵ�������н���, ��Ҫ��� FinSH Դ�룬Ȼ���ٶԽ� rt_hw_console_getchar */
/* �жϷ�ʽ */
char rt_hw_console_getchar(void)
{
    char ch = 0;

    /* �� ringbuffer ���ó����� */
    while (rt_ringbuffer_getchar(&uart_rxcb, (rt_uint8_t *)&ch) != 1)
    {
        rt_sem_take(&shell_rx_sem, RT_WAITING_FOREVER);
    } 
    return ch;   
}
void USART1_IRQHandler(void)
{
    int ch = -1;

    rt_interrupt_enter();          //���ж���һ��Ҫ������Ժ����������ж�
    
    if ( USART1->ISR&(1<<5) )
    {
        ch = USART1->RDR;
        /* ��ȡ�����ݣ������ݴ��� ringbuffer */
        rt_ringbuffer_putchar(&uart_rxcb, ch);
        rt_sem_release(&shell_rx_sem);
    }
    USART1->ICR|=1<<3;    //����������,������ܻῨ���ڴ����жϷ���������
    rt_interrupt_leave();    //���ж���һ��Ҫ������Ժ������뿪�ж�
} 
#else




//�������´���,֧��printf����,������Ҫѡ��use MicroLIB      
#if 1
#pragma import(__use_no_semihosting)  
//���HAL��ʹ��ʱ,ĳЩ������ܱ����bug
int _ttywrch(int ch)    
{
    ch=ch;
    return ch;
}
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
    int handle; 
    /* Whatever you require here. If the only file you are using is */ 
    /* standard output using printf() for debugging, no file handling */ 
    /* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
    x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
    while((USART1->ISR&0X40)==0);//ѭ������,ֱ���������   
    USART1->TDR = (u8) ch;      
    return ch;
}
#endif 
//end
//////////////////////////////////////////////////////////////////

#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���       
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��    ������ɱ�־
//bit14��    ���յ�0x0d
//bit13~0��    ���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���      
  
void USART1_IRQHandler(void)
{
    u8 res;    
#if SYSTEM_SUPPORT_OS         //���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
    OSIntEnter();    
#endif
    if(USART1->ISR&(1<<5))//���յ�����
    {     
        res=USART1->RDR; 
        if((USART_RX_STA&0x8000)==0)//����δ���
        {
            if(USART_RX_STA&0x4000)//���յ���0x0d
            {
                if(res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
                else USART_RX_STA|=0x8000;    //��������� 
            }else //��û�յ�0X0D
            {    
                if(res==0x0d)USART_RX_STA|=0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA&0X3FFF]=res;
                    USART_RX_STA++;
                    if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����      
                }         
            }
        }                                                    
    } 
    USART1->ICR|=1<<3;    //����������,������ܻῨ���ڴ����жϷ���������
#if SYSTEM_SUPPORT_OS     //���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
    OSIntExit();                                               
#endif
} 
#endif  
#endif          //RT Thread
//��ʼ��IO ����1
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//ע��:USART1��ʱ��,�ǿ���ͨ��RCC_D2CCIP2Rѡ���
//��������һ����Ĭ�ϵ�ѡ�񼴿�(Ĭ��ѡ��rcc_pclk2��Ϊ����1ʱ��)
//pclk2��APB2��ʱ��Ƶ��,һ��Ϊ100Mhz
//bound:������ 
/////////////////////////////////////С����������0.5 0.5*16����ֵ����BRR����λ
//BRR                ���ڲ���������            ��OVER8 = 0ʱ��0:3С��            4:15����        ��OVER8 = 1ʱ��0:2С��            4:15������λ3��Ҫ��������
//CR1                ���ڿ��ƼĴ���1        15��OVER8
//CR2                ���ڿ��ƼĴ���2
//
//
//
//BRR���㷽ʽ   ����ʱ�ӣ�*1M�� / ( ������ * 16 );Ȼ��õ�С�����ֳ�16����BRR,Ȼ���ڼ����������ַ���BRR
//ʱ��
//----------------------------------------------------------------------------------------------
//APB2ENR        λ4         U1    |        λ5            U6    |                            |                            |                            |                            |
//APB1ENR        λ17        U2    |        λ18        U3    |    λ19        U4    |    λ20        U5    |    λ30        U7    |    λ31        U8    |
//----------------------------------------------------------------------------------------------
//AF
//AF7:SPI2/3/6/USART1~3/6/UART7/SDIO1;            AF8:USART4/5/8/SPDIF/SAI2/4;

//�������ù���
//�趨CR1��    Mλ�����ֳ�                        λ28  M1    λ12    M0        ������2λ        00��8������λ    01��9������λ    10��7������λ
//�趨CR1��    STOPλ����ֹͣλλ��        
//�趨CR1��    OVER8λ������ģʽ            λ15    1��8��        0,16����ͨ��16����ֻ�е�UEΪ0ʱ�ſ���д�룩
//�趨BRR        �趨������
//�趨CR1��    UEλʹ��                                λ0        UE
//DMA����
//�趨CR1��    TEλʹ�ܷ���λ                    λ3        TE
//��TDRд������
//д��һ�����ݺ�ȴ�ISR�Ĵ�����TCλ��1����ɷ����ٷ�����һ��        λ6        TC

//�������ù���
//�趨CR1��    Mλ�����ֳ�                        λ28  M1    λ12    M0        ������2λ        00��8������λ    01��9������λ    10��7������λ
//�趨CR1��    STOPλ����ֹͣλλ��        
//�趨CR1��    OVER8λ������ģʽ            λ15    1��8��        0,16����ͨ��16����ֻ�е�UEΪ0ʱ�ſ���д�룩
//�趨BRR        �趨������
//�趨CR1��    UEλʹ��                                λ0        UE
//DMA����
//�趨CR1��    REλʹ�ܷ���λ                    λ2        RE
//�趨CR1��    RXNEIE/RXFNEIE���ջ������ǿ��ж�ʹ��      λ5        RXNEIE/RXFNEIE
//�����յ�����
//ISR�Ĵ�����RXNEλ����1��ʱ�ͱ�ʾ�����Ѿ����䵽RDR�Ĵ������ˡ�
//�������ж����RXNEIEλ�ᱻ��1�������ж�
//���������������⣬����Ӧ��λ����1
//��ȡRDR�Ĵ�����ʹ��RXNEλ��0
//

void uart_init_sys(u32 pclk2,u32 bound)
{       
    u32    temp;       
    temp=(pclk2*1000000+bound/2)/bound;    //�õ�USARTDIV@OVER8=0,���������������
    RCC->AHB4ENR|=1<<0;       //ʹ��PORTA��ʱ��  
    RCC->APB2ENR|=1<<4;      //ʹ�ܴ���1ʱ�� 
    GPIO_Set(GPIOA,PIN9|PIN10,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_MID,GPIO_PUPD_PU);//PA9,PA10,���ù���,�������
     GPIO_AF_Set(GPIOA,9,7);    //PA9,AF7
    GPIO_AF_Set(GPIOA,10,7);//PA10,AF7         
    //����������
     USART1->BRR=temp;         //����������@OVER8=0     
    USART1->CR1=0;             //����CR1�Ĵ���
    USART1->CR1|=0<<28;         //����M1=0
    USART1->CR1|=0<<12;         //����M0=0&M1=0,ѡ��8λ�ֳ� 
    USART1->CR1|=0<<15;     //����OVER8=0,16�������� 
    USART1->CR1|=1<<3;      //���ڷ���ʹ�� 
#if EN_USART1_RX              //���ʹ���˽���
    //ʹ�ܽ����ж� 
    USART1->CR1|=1<<2;      //���ڽ���ʹ��
    USART1->CR1|=1<<5;        //���ջ������ǿ��ж�ʹ��            
    MY_NVIC_Init(3,3,USART1_IRQn,2);//��2��������ȼ� 
#endif
    USART1->CR1|=1<<0;      //����ʹ��
}














