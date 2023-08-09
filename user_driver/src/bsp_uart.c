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

uint16_t SCA_RX_BUF[10];

__interrupt static void SCA_RX_ISR(void);

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
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, SCI_FIFO_RX8);
    SCI_enableFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
    
    SCI_enableInterrupt(SCIA_BASE,SCI_INT_RXFF);
    Interrupt_register(INT_SCIA_RX,&SCA_RX_ISR);
    Interrupt_enable(INT_SCIA_RX);

}

void uart_init(void)
{
    sci_gpio_mux();
    sci_a_init();
}

__interrupt
static void SCA_RX_ISR(void)
{
    uint32_t flag;
    flag = SCI_getInterruptStatus(SCIA_BASE);
    if(flag & SCI_INT_RXFF){
        SCI_readCharArray(SCIA_BASE,SCA_RX_BUF,8);
        SCI_writeCharArray(SCIA_BASE,SCA_RX_BUF,8);

    }
    SCI_clearInterruptStatus(SCIA_BASE,flag);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
