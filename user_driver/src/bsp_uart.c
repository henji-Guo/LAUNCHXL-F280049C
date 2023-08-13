/*********************** Motor Control Protocol **************************
'S'(1 byte) + motorID(1 byte) + CMD(1 byte) + PLAYLOAD(4byte) + 'P'(1字节) 

motorID : 0~255

CMD : 
        0x00    MOTOR_START
        0x01    MOTOR_STOP
        0x03    SET_DATA
        0x04    GET_DATA

GET_DATA :
            0x00    Iabc Iαβ Id Iq
            0x01    Uabc Uab Ud Uq
            0x02    
            
************************ 通讯协议命令 **************************/
#include "device.h"
#include "driverlib.h"
#include "f28004x_device.h"
#include "monitor.h"
#include "bsp_uart.h"

struct bsp_uart {
    uint16_t* tx_buf;
    int length;
    int fifo_depth;
    int flag;
};

struct bsp_uart bsp_uart;

uint16_t SCIA_RX_BUF[10];

__interrupt static void SCIA_RX_ISR(void);
__interrupt static void SCIA_TX_ISR(void);

static void sci_gpio_mux(void)
{
    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(28, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(29, GPIO_QUAL_ASYNC);
}

static void sci_a_init(void)
{
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
    SCI_clearOverflowStatus(SCIA_BASE);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
    SCI_resetChannels(SCIA_BASE);
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 6500000, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
    SCI_disableLoopback(SCIA_BASE);
    SCI_performSoftwareReset(SCIA_BASE);
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX8, SCI_FIFO_RX8);
    SCI_enableFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
    
    SCI_enableInterrupt(SCIA_BASE,SCI_INT_RXFF);
    Interrupt_register(INT_SCIA_RX,&SCIA_RX_ISR);
    Interrupt_register(INT_SCIA_TX,&SCIA_TX_ISR);
    Interrupt_enable(INT_SCIA_RX);
    Interrupt_enable(INT_SCIA_TX);
}

int uart_isIDLE(struct bsp_uart *huart)
{
    if(huart->length <= 0)
        return 0;
    return -1;
}

void uart_int_transmit(struct bsp_uart *huart,uint16_t *sendMsg, uint16_t len)
{
    int fillLength;

    huart->length = len;
    huart->tx_buf = sendMsg;
    huart->flag = 1;

    /* get the fifo available space */
    fillLength = huart->fifo_depth - SCI_getTxFIFOStatus(SCIA_BASE);

    /* fill in the data to fifo  */
    SCI_writeCharArray(SCIA_BASE,huart->tx_buf,fillLength);

    /* minus the length that has been sent */
    huart->length -= fillLength;
    huart->tx_buf += fillLength;

    /* enable TX FIFO Interrupt */
    SCI_enableInterrupt(SCIA_BASE,SCI_INT_TXFF);

}

void uart_init(void)
{
    sci_gpio_mux();
    sci_a_init();
    bsp_uart.fifo_depth = 16;
    bsp_uart.length = 0;
    bsp_uart.flag = 0;
}

__interrupt
static void SCIA_RX_ISR(void)
{
    uint32_t flag;
    flag = SCI_getInterruptStatus(SCIA_BASE);
    if(flag & SCI_INT_RXFF){
        SCI_readCharArray(SCIA_BASE,SCIA_RX_BUF,8);
        monitor_cmd(SCIA_RX_BUF);
    }
    SCI_clearInterruptStatus(SCIA_BASE,flag);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

__interrupt
static void SCIA_TX_ISR(void)
{
    int flag;
    int fillLen,sendLen;

    flag = SCI_getInterruptStatus(SCIA_BASE);
    if(flag & SCI_INT_TXFF){
        /* if length < 0 , mean that the data has been sent */
        if (bsp_uart.length <= 0){
            SCI_disableInterrupt(SCIA_BASE,SCI_INT_TXFF);
        } else {
            fillLen = bsp_uart.fifo_depth - SCI_getTxFIFOStatus(SCIA_BASE);

            sendLen = (bsp_uart.length < fillLen) ? bsp_uart.length : fillLen; 

            SCI_writeCharArray(SCIA_BASE,bsp_uart.tx_buf,sendLen);
            bsp_uart.length -= sendLen;
            bsp_uart.tx_buf += sendLen;
        }
    }
    SCI_clearInterruptStatus(SCIA_BASE,flag);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
