/*
 * i2c_example_test.c
 *
 *  Created on: 2023年6月26日
 *      Author: GHJ
 */
#include "driverlib.h"
#include "f28004x_device.h"
#include "string.h"

#define I2C_SLAVE_ADDRESS   0x76
#define I2C_TX_DATA_LENGTH  0x01
#define I2C_RX_DATA_LENGTH  0x06
uint16_t i2c_tx_buffer[I2C_TX_DATA_LENGTH];
uint16_t i2c_rx_buffer[I2C_RX_DATA_LENGTH];

/* BMP280 Sensor Address */
#define BMP280_id       0xD0
#define BMP280_reset    0xE0
#define BMP280_status   0xF3
#define BMP280_config   0xF4
#define BMP280_ctrl     0xD0
#define BMP280_press_msb    0xF7
#define BMP280_press_lsb    0xF8
#define BMP280_press_xlsb    0xF9
#define BMP280_temp_msb     0xFA
#define BMP280_temp_lsb     0xFB
#define BMP280_temp_xlsb     0xFC

static inline void i2c_gpio_mux_init(void)
{
    // SDA  GPIO35
    // SCL  GPIO37
    EALLOW;
    // Reset GPIO MUX
    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0b00;
    GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 0b00;
    // I2CA GPIO MUX 3 => GMUX 00 MUX 11
    GpioCtrlRegs.GPBGMUX1.bit.GPIO35 = 0b00;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO37 = 0b00;
    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0b11;
    GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 0b11;
    // GPIO ASYNC
    GpioCtrlRegs.GPBQSEL1.bit.GPIO35 = 0b11;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO37 = 0b11;
    // GPIO Pullup
    GpioCtrlRegs.GPBPUD.bit.GPIO35 = 0b0;
    GpioCtrlRegs.GPBPUD.bit.GPIO37 = 0b0;
    // GPIO is control by the master not CLA
    GpioCtrlRegs.GPBCSEL1.bit.GPIO35 = 0b00;
    GpioCtrlRegs.GPBCSEL1.bit.GPIO37 = 0b00;
    EDIS;
}

static inline void i2c_init(void)
{
    /* I2C Master TX/RX Flowchat */
    /* I2C Basic configuration */

    // disable I2C ISR=0
    I2caRegs.I2CMDR.bit.IRS = 0;

    // Clock reference "TRM 24.1.5 Clock Generation"
    // Configure I2C Module clock using I2CPSC register I2C Module clock should be 7-12Mhz
    I2caRegs.I2CPSC.bit.IPSC = 4;  // I2C_Module_CLK(Fmod) = 100MHz SYSCLK / (ISP=9 + 1) = 10Mhz

    // Configure I2C Baud rate using I2CCLKL and I2CCLKH
    // I2C Baud rate = (I2CCLKL + d)+(I2CCLKH + d)/Fmod
    I2caRegs.I2CCLKL = 5;
    I2caRegs.I2CCLKH = 5;

    // Configure I2C Own address using I2COAR (no need in master tx/rx mode)
    I2caRegs.I2COAR.bit.OAR = 0x00;

    // Configure I2C Slave address to talk to using I2CSAR
    I2caRegs.I2CSAR.bit.SAR = I2C_SLAVE_ADDRESS;

    // FIFO mode (16-deep x 8-bit FIFO)
    // enable and configure TX/RX FIFO,configure TX/RX FIFO level,if need can enable FIFO interrupts
    I2caRegs.I2CFFTX.bit.I2CFFEN = 0x1;
    I2caRegs.I2CFFTX.bit.TXFFRST = 0x1;
    I2caRegs.I2CFFTX.bit.TXFFIL = 0x0;
//    I2caRegs.I2CFFTX.bit.TXFFIENA = 0x1;
    I2caRegs.I2CFFRX.bit.RXFFRST = 0x1;
    I2caRegs.I2CFFRX.bit.RXFFIL = 0x6;
//    I2caRegs.I2CFFRX.bit.RXFFIENA = 0x1;

    // enable I2C IRS=1
    I2caRegs.I2CMDR.bit.IRS = 1;

    // Application code decides to initiate i2c transaction
    // check I2C bus free I2CSTR.BB == 0
    while(I2caRegs.I2CSTR.bit.BB);

    // Master Transmitter MST=1 or Master Receive MST=0
    // Master Transmitter MST=1
    I2caRegs.I2CMDR.bit.MST = 1;

    // check Repeat mode RM=1 or No-repeat mode RM=0
    // note that repeat mode don't care I2CNT
    I2caRegs.I2CMDR.bit.RM = 0;

    // No-repeat mode
    // set I2CCNT number of bytes to be transmitted
    I2caRegs.I2CCNT = I2C_TX_DATA_LENGTH;

    // set I2CMDR
    I2caRegs.I2CMDR.all = 0x6620;

    // put data in fifo
    for (int i = 0; i < I2C_TX_DATA_LENGTH; i++) {
        // check TX FIFO is full
        while(I2caRegs.I2CFFTX.bit.TXFFST == 16);
        I2caRegs.I2CDXR.bit.DATA = i2c_tx_buffer[i];
    }

    // check i2c tx status
    while(I2caRegs.I2CSTR.bit.XSMT != 1 || I2caRegs.I2CSTR.bit.BYTESENT != 1);
    I2caRegs.I2CSTR.bit.BYTESENT = 1;


    // check I2CSTR ARDY bit
    while(I2caRegs.I2CSTR.bit.ARDY != 1);
    I2caRegs.I2CSTR.bit.ARDY = 1;

    // STOP Signal (if necessary)
//     I2caRegs.I2CMDR.bit.STP = 1;

//     while(I2caRegs.I2CSTR.bit.BB);


    // enter recieve mode
    // Master Receive MST=0
    I2caRegs.I2CMDR.bit.MST = 0;

    // check Repeat mode RM=1 or No-repeat mode RM=0
    // note that repeat mode don't care I2CNT
    I2caRegs.I2CMDR.bit.RM = 0;

    // No-repeat mode
    // set I2CCNT number of bytes to be recieved
    I2caRegs.I2CCNT = I2C_RX_DATA_LENGTH;

    // set I2CMDR
    I2caRegs.I2CMDR.all = 0x6420; // TRM misdescription

    while(I2caRegs.I2CFFRX.bit.RXFFINT != 1);

    // put data in fifo
    for (int i = 0; i < I2C_RX_DATA_LENGTH; i++) {
        // check TX FIFO is full
        if(I2caRegs.I2CFFRX.bit.RXFFST > 0){
            i2c_rx_buffer[i] = I2caRegs.I2CDRR.all;
        }
    }

    // check NACK
    if(I2caRegs.I2CSTR.bit.NACKSNT != 1)
        I2caRegs.I2CMDR.bit.NACKMOD = 1;
    while(I2caRegs.I2CSTR.bit.NACKSNT != 1);
    I2caRegs.I2CSTR.bit.NACKSNT = 1;

    // send stop
    I2caRegs.I2CMDR.bit.STP = 1;

}

void i2c_test()
{
    i2c_gpio_mux_init();

    // tx_buffer & rx_buffer
    memset(i2c_tx_buffer,0,I2C_TX_DATA_LENGTH);
    memset(i2c_rx_buffer,0,I2C_RX_DATA_LENGTH);

    i2c_tx_buffer[0] =  BMP280_press_msb;

    i2c_init();

}

