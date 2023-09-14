//###########################################################################
//
// FILE:   sfra_gui_scicomms_driverlib.c
//
// TITLE:  Comms kernel as an interface to SFRA GUI
//
// AUTHOR: Manish Bhardwaj (C2000 Systems Solutions, Houston , TX)
//
//#############################################################################
// $TI Release: C2000 Software Frequency Response Analyzer Library v1.40.00.00 $
// $Release Date: Fri Jul 21 14:23:53 CDT 2023 $
// $Copyright:
// Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################

#include <stdint.h>
#include "driverlib.h"
#include "device.h"
#include "sfra_gui_scicomms_driverlib.h"

//
// Function prototypes for Command RECEIVE State machine
// ------------------------------------------------------------
//
void SFRA_GUI_getCmdByte(void);
void SFRA_GUI_echoCmdByte(void);
void SFRA_GUI_getSizeByte(void);
void SFRA_GUI_echoSizeByte(void);
void SFRA_GUI_getDataByte(void);
void SFRA_GUI_echoDataByte(void);
void SFRA_GUI_packWord(void);
void SFRA_GUI_packArray(void);
void SFRA_GUI_cmdInterpreter(void);

//
// Function prototypes for Command Interpreter and dispatcher
//
void SFRA_GUI_lifePulseTsk(void); // 0
void SFRA_GUI_setText(void);      // 1
void SFRA_GUI_setButton(void);    // 2
void SFRA_GUI_setSlider(void);    // 3
void SFRA_GUI_getVariable(void);  // 4
void SFRA_GUI_getArray(void);     // 5
void SFRA_GUI_getData(void);      // 6
void SFRA_GUI_setData32(void);    // 7
void SFRA_GUI_spareTsk08(void);   // 8

void SFRA_GUI_sendData(void);

//
// Variable declarations
// State pointer for Command Packet Receive
//
void (*SFRA_GUI_rcvTaskPointer)(void);

//
// Array of pointers to Function (that are tasks)
//
void (*SFRA_GUI_cmdDispatcher[SFRA_GUI_CMD_NUMBER])(void);

volatile int16_t *SFRA_GUI_varSetTxtList[16];
volatile int16_t *SFRA_GUI_varSetBtnList[16];
volatile int16_t *SFRA_GUI_varSetSldrList[16];
volatile int16_t *SFRA_GUI_varGetList[16];
volatile int32_t *SFRA_GUI_arrayGetList[16];
volatile int16_t *SFRA_GUI_dataGetList[16];
volatile uint32_t *SFRA_GUI_dataSetList[16];

volatile int16_t SFRA_GUI_commsOKflg;
volatile int16_t SFRA_GUI_serialCommsTimer;

volatile uint32_t SFRA_GUI_sci_base_addr;

uint16_t SFRA_GUI_lowByteFlag;
uint16_t SFRA_GUI_sendTaskPtr;
uint16_t SFRA_GUI_rxChar;
uint16_t SFRA_GUI_rxWord;
uint16_t SFRA_GUI_cmdPacket[SFRA_GUI_PKT_SIZE];
uint16_t SFRA_GUI_taskDoneFlag;
uint16_t SFRA_GUI_numWords;
uint16_t SFRA_GUI_wordsLeftToGet;

uint16_t SFRA_GUI_dataOut16;
int32_t SFRA_GUI_dataOut32;

int16_t *SFRA_GUI_memDataPtr16;
int32_t *SFRA_GUI_memDataPtr32;

//
// for debug
//
int16_t  SFRA_GUI_rcvTskPtrShdw;

int16_t SFRA_GUI_delayer;

int16_t SFRA_GUI_memGetPtr;
uint32_t SFRA_GUI_memGetAddress;
int16_t SFRA_GUI_memGetAmount;

int16_t SFRA_GUI_memSetPtr;
uint32_t SFRA_GUI_memSetValue;

uint32_t SFRA_GUI_temp;

uint16_t SFRA_GUI_led_flag;
uint16_t SFRA_GUI_led_gpio;

uint16_t SFRA_GUI_sweep_start;

void SFRA_GUI_config(volatile uint32_t sci_base,
                     uint32_t vbus_clk,
                     uint32_t baudrate,
                     uint16_t scirx_gpio_pin,
                     uint32_t scirx_gpio_pin_config,
                     uint16_t scitx_gpio_pin,
                     uint32_t scitx_gpio_pin_config,
                     uint16_t led_indicator_flag,
                     uint16_t led_gpio_pin,
                     uint32_t led_gpio_pin_config,
                     SFRA_F32 *sfra,
                     uint16_t plot_option)
{
    int16_t j = 0;

    //
    // setup Gpio for SCI comms for SFRA
    //

    GPIO_setPinConfig(scirx_gpio_pin_config);
    GPIO_setPinConfig(scitx_gpio_pin_config);
    GPIO_setQualificationMode(scirx_gpio_pin, GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(scitx_gpio_pin, GPIO_QUAL_ASYNC);

    //
    // Note: Assumes Clocks to SCI are turned on in setupDevice()->Device_init()
    // Note: Assumes GPIO pins for SCIA are configured to Primary function
    //

    //
    // 1 stop bit,  No parity, 8 char bits,
    //
    SCI_setConfig(sci_base,
                  vbus_clk, baudrate,
                    (SCI_CONFIG_WLEN_8 |
                    SCI_CONFIG_STOP_ONE |
                    SCI_CONFIG_PAR_NONE));
    //
    // No loopback
    //
    SCI_disableLoopback(sci_base);

    SCI_enableInterrupt(sci_base, SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);

    //
    // Relinquish SCI from Reset by SW Reset and setting TXE, and RXE bits
    //
    SCI_enableModule(sci_base);
    SCI_performSoftwareReset(sci_base);

    HWREGH(sci_base + SCI_O_FFTX) = 0x8040;
    HWREGH(sci_base + SCI_O_FFRX) = 0x204f;
    HWREGH(sci_base + SCI_O_FFCT) = 0x0;

    //
    // Disable RX ERR, SLEEP, TXWAKE
    //
    SCI_clearInterruptStatus(sci_base,
                             SCI_INT_TXRDY | SCI_INT_RXRDY_BRKDT );

    //
    // Initialize the CmdPacket Rcv Handler state machine ptr
    //
    SFRA_GUI_rcvTaskPointer = &SFRA_GUI_getCmdByte;
    //
    // DEBUG
    //
    SFRA_GUI_rcvTskPtrShdw = 1;
    //
    // Init to 1st state
    //
    SFRA_GUI_sendTaskPtr = 0;
    //
    // Start with LSB during Byte-to-Word packing
    //
    SFRA_GUI_lowByteFlag = 1;

    SFRA_GUI_dataOut16 = 0;
    SFRA_GUI_dataOut32 = 0;

    //
    // for debug
    //
    SFRA_GUI_rcvTskPtrShdw = 0;

    SFRA_GUI_delayer = 0;

    SFRA_GUI_memGetPtr = 0;
    SFRA_GUI_memGetAddress = 0x00000000;
    SFRA_GUI_memGetAmount = 0;

    SFRA_GUI_memSetPtr = 0;
    SFRA_GUI_memSetValue = 0x00000000;

    SFRA_GUI_sweep_start = 0;
    SFRA_GUI_serialCommsTimer = 0;
    SFRA_GUI_commsOKflg = 0;

    SFRA_GUI_sci_base_addr = sci_base;

    //
    // clear Command Packet
    //
    for (j = 0; j < SFRA_GUI_PKT_SIZE; j++)
    {
        SFRA_GUI_cmdPacket[j] = 0x0;
    }

    j = 0;

    //
    // init all dispatch Tasks
    //
    SFRA_GUI_cmdDispatcher[0] = SFRA_GUI_lifePulseTsk;
    SFRA_GUI_cmdDispatcher[1] = SFRA_GUI_setText;
    SFRA_GUI_cmdDispatcher[2] = SFRA_GUI_setButton;
    SFRA_GUI_cmdDispatcher[3] = SFRA_GUI_setSlider;
    SFRA_GUI_cmdDispatcher[4] = SFRA_GUI_getVariable;
    SFRA_GUI_cmdDispatcher[5] = SFRA_GUI_getArray;
    SFRA_GUI_cmdDispatcher[6] = SFRA_GUI_getData;
    SFRA_GUI_cmdDispatcher[7] = SFRA_GUI_setData32;
    SFRA_GUI_cmdDispatcher[8] = SFRA_GUI_spareTsk08;



    SFRA_GUI_varSetBtnList[0] = (int16_t *)&(SFRA_GUI_sweep_start);

    SFRA_GUI_varGetList[0] = (int16_t *)&(sfra->vecLength);
    SFRA_GUI_varGetList[1] = (int16_t *)&(sfra->status);
    SFRA_GUI_varGetList[2] = (int16_t *)&(sfra->freqIndex);

    //
    //"Setable" variables
    // assign GUI "setable" by Text parameter address
    //
    SFRA_GUI_dataSetList[0] = (uint32_t *)&(sfra->freqStart);
    SFRA_GUI_dataSetList[1] = (uint32_t *)&(sfra->amplitude);
    SFRA_GUI_dataSetList[2] = (uint32_t *)&(sfra->freqStep);

    //
    // assign a GUI "getable" parameter array address
    //
    SFRA_GUI_arrayGetList[0] = (int32_t *)sfra->freqVect;


    if(plot_option == SFRA_GUI_PLOT_GH_CL)
    {
        SFRA_GUI_arrayGetList[1] = (int32_t *)sfra->gh_magVect;
        SFRA_GUI_arrayGetList[2] = (int32_t *)sfra->gh_phaseVect;

        SFRA_GUI_arrayGetList[3] = (int32_t *)sfra->cl_magVect;
        SFRA_GUI_arrayGetList[4] = (int32_t *)sfra->cl_phaseVect;
    }
    //
    // default is to plot gh and h
    //
    else
    {
        SFRA_GUI_arrayGetList[1] = (int32_t *)sfra->gh_magVect;
        SFRA_GUI_arrayGetList[2] = (int32_t *)sfra->gh_phaseVect;

        SFRA_GUI_arrayGetList[3] = (int32_t *)sfra->h_magVect;
        SFRA_GUI_arrayGetList[4] = (int32_t *)sfra->h_phaseVect;
    }



    SFRA_GUI_arrayGetList[5] = (int32_t *)&(sfra->freqStart);
    SFRA_GUI_arrayGetList[6] = (int32_t *)&(sfra->amplitude);
    SFRA_GUI_arrayGetList[7] = (int32_t *)&(sfra->freqStep);


    if(led_indicator_flag == 1)
    {
        GPIO_setDirectionMode(led_gpio_pin, GPIO_DIR_MODE_OUT);
        GPIO_setQualificationMode(led_gpio_pin, GPIO_QUAL_SYNC);
        GPIO_setPinConfig(led_gpio_pin_config);
        SFRA_GUI_led_flag = 1;
        SFRA_GUI_led_gpio = led_gpio_pin;
    }
    else
    {
        SFRA_GUI_led_flag = 0;
    }

}

//
// Host Command RECEIVE and DISPATCH State Machine
//

//
// State Machine Entry Point
//
void SFRA_GUI_runSerialHostComms(SFRA_F32 *sfra)
{
    if(SFRA_GUI_sweep_start == 1)
    {
        SFRA_GUI_sweep_start = 0;
        sfra->start = 1;
    }
    //
    // Call routine pointed to by state pointer
    //
    (*SFRA_GUI_rcvTaskPointer)();

    SFRA_GUI_serialCommsTimer++;
}


//
// Task 1
//
void SFRA_GUI_getCmdByte(void)
{
    //
    // check if a char has been received
    //
    if((SCI_getRxStatus(SFRA_GUI_sci_base_addr) & SCI_RXSTATUS_READY ) != 0)
    {
        SFRA_GUI_rxChar = SCI_readCharBlockingNonFIFO(SFRA_GUI_sci_base_addr);
        //
        // point to next state
        //
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_echoCmdByte;
        SFRA_GUI_serialCommsTimer = 0;
        //
        // DEBUG
        //RcvTskPtrShdw = 2;
        //
        SFRA_GUI_echoCmdByte();
    }
    //
    //~2.5 s timeout, SFRA GUI function is called at 100Hz (recommended)
    // hence 2500/100 = 2.5sec
    //
    else if((SCI_getRxStatus(SFRA_GUI_sci_base_addr)&SCI_RXSTATUS_BREAK) != 0
        || SFRA_GUI_serialCommsTimer > 2500)
    {

        SCI_enableModule(SFRA_GUI_sci_base_addr);

        //
        // If break detected or serialport times out, reset SCI
        //--- Needed by some serialports when code is run with an emulator
        //
        SCI_performSoftwareReset(SFRA_GUI_sci_base_addr);

        SCI_clearInterruptStatus(SFRA_GUI_sci_base_addr,
                                 SCI_INT_TXRDY | SCI_INT_RXRDY_BRKDT);

        asm(" RPT#8 || NOP");

        //
        // Init to 1st state
        //
        SFRA_GUI_sendTaskPtr = 0;
        SFRA_GUI_serialCommsTimer = 0;

        //
        // go back and wait for new CMD
        //
        SFRA_GUI_commsOKflg = 0;
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_getCmdByte;
    }
    else
    {

    }
}

//
// Task 2
//
void SFRA_GUI_echoCmdByte(void)
{
    //
    // is TXBUF empty ?, that is TXRDY = 1
    //
    if(SCI_isTransmitterBusy(SFRA_GUI_sci_base_addr) == 0)
    {
        SCI_writeCharBlockingNonFIFO(SFRA_GUI_sci_base_addr, SFRA_GUI_rxChar);
        SFRA_GUI_cmdPacket[0] = SFRA_GUI_rxChar;
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_getSizeByte;
        //
        // DEBUG
        // RcvTskPtrShdw = 3;
        // Un-comment for simple echo test
        // RcvTaskPointer = &GetCmdByte;
        // Reset Time-out timer
        //
        SFRA_GUI_serialCommsTimer = 0;
    }

}

//
// Task 3
//
void SFRA_GUI_getSizeByte(void)
{
    //
    // check if a char has been received
    //
    if((SCI_getRxStatus(SFRA_GUI_sci_base_addr) & SCI_RXSTATUS_READY ) != 0)
    {
        SFRA_GUI_rxChar = SCI_readCharBlockingNonFIFO(SFRA_GUI_sci_base_addr);

        //
        // point to next state
        //
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_echoSizeByte;
        //
        // DEBUG
        //RcvTskPtrShdw = 4;
        //
        SFRA_GUI_echoSizeByte();
    }

    //
    // 1000*1mS = 1.0 sec timeout, SFRA GUI function is called at 1ms
    //
    else if(SFRA_GUI_serialCommsTimer > 1000)
    {
        SFRA_GUI_commsOKflg = 0;
        //
        // Abort, go back wait for new CMD
        //
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_getCmdByte;
        SFRA_GUI_serialCommsTimer = 0;
    }
}

//
// Task 4
//
void SFRA_GUI_echoSizeByte(void)
{
    //
    // is TXBUF empty ?, that is TXRDY = 1
    //
    if(SCI_isTransmitterBusy(SFRA_GUI_sci_base_addr) == 0)
    {
        SCI_writeCharBlockingNonFIFO(SFRA_GUI_sci_base_addr, SFRA_GUI_rxChar);
        SFRA_GUI_cmdPacket[1] = SFRA_GUI_rxChar;
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_getDataByte;
        //
        // DEBUG
        //RcvTskPtrShdw = 5;
        // Un-comment for Test
        //RcvTaskPointer = &GetCmdByte;
        // Reset Time-out timer
        //
        SFRA_GUI_serialCommsTimer = 0;
    }
}

//
// Task 5
//
void SFRA_GUI_getDataByte(void)
{
    //
    // check if a char has been received
    //
    if((SCI_getRxStatus(SFRA_GUI_sci_base_addr) & SCI_RXSTATUS_READY ) != 0)
    {
        SFRA_GUI_rxChar = SCI_readCharBlockingNonFIFO(SFRA_GUI_sci_base_addr);
        //
        // point to next state
        //
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_echoDataByte;
        //
        // DEBUG
        //RcvTskPtrShdw = 6;
        //
        SFRA_GUI_echoDataByte();
    }

    //
    // 1000*1mS = 1 sec timeout, SFRA GUI function is called at 1ms/100Hz
    //
    else if(SFRA_GUI_serialCommsTimer > 1000)
    {
        SFRA_GUI_commsOKflg = 0;
        //
        // Abort, go back wait for new CMD
        //
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_getCmdByte;
        SFRA_GUI_serialCommsTimer = 0;
    }
}

//
// Task 6
//
void SFRA_GUI_echoDataByte(void)
{
    //
    // is TXBUF empty ?, that is TXRDY = 1
    //
    if(SCI_isTransmitterBusy(SFRA_GUI_sci_base_addr) == 0)
    {
        SCI_writeCharBlockingNonFIFO(SFRA_GUI_sci_base_addr, SFRA_GUI_rxChar);
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_packWord;
        //
        // DEBUG
        //RcvTskPtrShdw = 7;
        //
    }
}

//
// expects LSB first then MSB // Task 7
//
void SFRA_GUI_packWord(void)
{
    if(SFRA_GUI_lowByteFlag == 1)
    {
        SFRA_GUI_rxWord = SFRA_GUI_rxChar;
        SFRA_GUI_lowByteFlag = 0;
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_getDataByte;
        //
        // DEBUG
        // RcvTskPtrShdw = 5;
        //
        SFRA_GUI_getDataByte();
    }
    else
    {
        SFRA_GUI_rxWord = SFRA_GUI_rxWord | (SFRA_GUI_rxChar << 8);
        SFRA_GUI_lowByteFlag = 1;
        //
        // store data in packet
        //
        SFRA_GUI_cmdPacket[2] = SFRA_GUI_rxWord;
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_cmdInterpreter;
        //
        // DEBUG
        // RcvTskPtrShdw = 8;
        // indicate new task underway
        //
        SFRA_GUI_taskDoneFlag = 0;
    }
}

//
// Task 8
//
void SFRA_GUI_cmdInterpreter(void)
{
    if(SFRA_GUI_taskDoneFlag == 0)
    {
        //
        // dispatch Task
        //
        (*SFRA_GUI_cmdDispatcher[SFRA_GUI_cmdPacket[0]])();
    }

    //
    // Incase Task never finishes
    // 2500*1mS = 2.5 sec timeout
    //
    if(SFRA_GUI_serialCommsTimer > 2500)
    {
        SFRA_GUI_commsOKflg = 0;
        //
        // Abort, go back wait for new CMD
        //
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_getCmdByte;
        SFRA_GUI_serialCommsTimer = 0;
    }
    if(SFRA_GUI_taskDoneFlag == 1)
    {
        SFRA_GUI_rcvTaskPointer = &SFRA_GUI_getCmdByte;
        //
        // DEBUG
        //RcvTskPtrShdw = 1;
        //
    }
}

//
// Slave Tasks commanded by Host
//

//
// CmdPacket[0] = 0
//
void SFRA_GUI_lifePulseTsk(void)
{
    if(SFRA_GUI_led_flag == 1)
    {
        //
        // LED2-ON
        //
        if(SFRA_GUI_cmdPacket[2] == 0x0000 && SFRA_GUI_cmdPacket[1] == 0x00)
        {
            GPIO_togglePin(SFRA_GUI_led_gpio);
        }
        //
        // LED2-OFF
        //
        if(SFRA_GUI_cmdPacket[2] == 0x0001 && SFRA_GUI_cmdPacket[1] == 0x00)
        {
            GPIO_togglePin(SFRA_GUI_led_gpio);
        }
        //
        // LED2-Toggle
        //
        if(SFRA_GUI_cmdPacket[2] == 0x0002 && SFRA_GUI_cmdPacket[1] == 0x00)
        {
            GPIO_togglePin(SFRA_GUI_led_gpio);
        }
    }

    SFRA_GUI_commsOKflg = 1;
    SFRA_GUI_serialCommsTimer = 0;
    SFRA_GUI_taskDoneFlag = 1;
}

//
// CmdPacket[0] = 1
//
void SFRA_GUI_setText(void)
{
    *SFRA_GUI_varSetTxtList[SFRA_GUI_cmdPacket[1]] = SFRA_GUI_cmdPacket[2];

    //
    // indicate Task execution is complete
    //
    SFRA_GUI_taskDoneFlag = 1;
}

//
// CmdPacket[0] = 2
//
void SFRA_GUI_setButton(void)
{
    *SFRA_GUI_varSetBtnList[SFRA_GUI_cmdPacket[1]] = SFRA_GUI_cmdPacket[2];

    //
    // indicate Task execution is complete
    //
    SFRA_GUI_taskDoneFlag = 1;
}

//
// CmdPacket[0] = 3
//
void SFRA_GUI_setSlider(void)
{
    *SFRA_GUI_varSetSldrList[SFRA_GUI_cmdPacket[1]] = SFRA_GUI_cmdPacket[2];
    //
    // indicate Task execution is complete
    //
    SFRA_GUI_taskDoneFlag = 1;
}

//
// CmdPacket[0] = 4
//
void SFRA_GUI_getVariable(void)
{
    SFRA_GUI_sendData();
}

//
//Send a Uint16 array one element at a time
// CmdPacket[0] = 5
//
void SFRA_GUI_getArray(void)
{
    SFRA_GUI_sendData();
}

//
// CmdPacket[0] = 6
//
void SFRA_GUI_getData(void)
{
    switch(SFRA_GUI_memGetPtr)
    {
        case 0:
            SFRA_GUI_memGetAddress = SFRA_GUI_cmdPacket[2];
            SFRA_GUI_memGetPtr = 1;

            SFRA_GUI_wordsLeftToGet = 1;
            SFRA_GUI_sendTaskPtr = 1;
            SFRA_GUI_taskDoneFlag = 1;
            break;

        case 1:
            SFRA_GUI_temp = SFRA_GUI_cmdPacket[2];
            SFRA_GUI_memGetAddress = SFRA_GUI_memGetAddress +
                                     (SFRA_GUI_temp << 16);
            SFRA_GUI_memDataPtr16 = (int16_t *)SFRA_GUI_memGetAddress;
            SFRA_GUI_dataOut16 = *SFRA_GUI_memDataPtr16;
            SFRA_GUI_sendData();

            if(SFRA_GUI_taskDoneFlag == 1)
            {
                SFRA_GUI_memGetPtr = 0;
            }
            break;
        }

    //
    // indicate Task execution is complete
    // TaskDoneFlag = 1;
    //
}

//
// CmdPacket[0] = 7 [Edited to get 32-bit set text and set label working]
//
void SFRA_GUI_setData32(void)
{
    switch(SFRA_GUI_memSetPtr)
    {
        case 0:
            SFRA_GUI_memSetValue = SFRA_GUI_cmdPacket[2];
            SFRA_GUI_memSetPtr = 1;

            SFRA_GUI_taskDoneFlag = 1;
            break;

        case 1:
            SFRA_GUI_temp = SFRA_GUI_cmdPacket[2];
            SFRA_GUI_memSetValue = SFRA_GUI_memSetValue + (SFRA_GUI_temp << 16);

            *SFRA_GUI_dataSetList[SFRA_GUI_cmdPacket[1]] = SFRA_GUI_memSetValue;

            SFRA_GUI_memSetPtr = 0;
            SFRA_GUI_taskDoneFlag = 1;
            break;
    }

}

//
// CmdPacket[0] = 8
//
void SFRA_GUI_spareTsk08(void)
{
    //
    // indicate Task execution is complete
    //
    SFRA_GUI_taskDoneFlag = 1;
}

//
//
//
void SFRA_GUI_sendData(void)
{
    if(SFRA_GUI_cmdPacket[0] == 0x04 || SFRA_GUI_cmdPacket[0] == 0x06)
    {
        switch(SFRA_GUI_sendTaskPtr)
        {
        case 0:  //initialization

            SFRA_GUI_memDataPtr16 =
                    (int16_t *) SFRA_GUI_varGetList[SFRA_GUI_cmdPacket[1]];
            SFRA_GUI_dataOut16 = *SFRA_GUI_memDataPtr16;
            SFRA_GUI_wordsLeftToGet = SFRA_GUI_cmdPacket[2];
            //
            //Note that case 0 rolls into case 1 (no break)
            //

        case 1:  //send LSB
            if(SFRA_GUI_wordsLeftToGet > 0)
            {
                if(SCI_isTransmitterBusy(SFRA_GUI_sci_base_addr) == 0)
                {
                SCI_writeCharBlockingNonFIFO(SFRA_GUI_sci_base_addr,
                                             SFRA_GUI_dataOut16 & 0x000000FF);
                SFRA_GUI_sendTaskPtr = 2;
                }
            }
            else
            {
                SFRA_GUI_sendTaskPtr = 0;
                SFRA_GUI_taskDoneFlag = 1;
                break;
            }

        case 2: //send MSB
            if(SCI_isTransmitterBusy(SFRA_GUI_sci_base_addr) == 0)
            {
                SCI_writeCharBlockingNonFIFO(SFRA_GUI_sci_base_addr,
                                       SFRA_GUI_dataOut16 >> 8 & 0x000000FF);

                SFRA_GUI_memDataPtr16 = SFRA_GUI_memDataPtr16 + 1;
                SFRA_GUI_dataOut16 = *SFRA_GUI_memDataPtr16;
                SFRA_GUI_wordsLeftToGet = SFRA_GUI_wordsLeftToGet - 1;
                SFRA_GUI_sendTaskPtr = 1;
            }
            break;
        }
    }
    else
    {
        switch(SFRA_GUI_sendTaskPtr)
        {
        case 0:  //initialization
        SFRA_GUI_memDataPtr32 =
                (int32_t *) SFRA_GUI_arrayGetList[SFRA_GUI_cmdPacket[1]];
        SFRA_GUI_dataOut32 = *SFRA_GUI_memDataPtr32;
        SFRA_GUI_wordsLeftToGet = SFRA_GUI_cmdPacket[2];
        //
        //Note that case 0 rolls into case 1 (no break)
        //
        case 1:  //send LSB
            if(SFRA_GUI_wordsLeftToGet > 0)
            {
                if(SCI_isTransmitterBusy(SFRA_GUI_sci_base_addr) == 0)
                {
                    SCI_writeCharBlockingNonFIFO(SFRA_GUI_sci_base_addr,
                                        SFRA_GUI_dataOut32 & 0x000000FF);
                    SFRA_GUI_sendTaskPtr = 2;
                }
            }
            else
            {
                SFRA_GUI_sendTaskPtr = 0;
                SFRA_GUI_taskDoneFlag = 1;
                break;
            }

        case 2:
            if(SCI_isTransmitterBusy(SFRA_GUI_sci_base_addr) == 0)
            {
                SCI_writeCharBlockingNonFIFO(SFRA_GUI_sci_base_addr,
                                         SFRA_GUI_dataOut32 >> 8 & 0x000000FF);
                SFRA_GUI_sendTaskPtr = 3;
            }

        case 3:
            if(SCI_isTransmitterBusy(SFRA_GUI_sci_base_addr) == 0)
            {
                SCI_writeCharBlockingNonFIFO(SFRA_GUI_sci_base_addr,
                                         SFRA_GUI_dataOut32 >> 16 & 0x000000FF);
                SFRA_GUI_sendTaskPtr = 4;
            }

        case 4:
            //
            // send MSB
            //
            if(SCI_isTransmitterBusy(SFRA_GUI_sci_base_addr) == 0)
            {
                SCI_writeCharBlockingNonFIFO(SFRA_GUI_sci_base_addr,
                                         SFRA_GUI_dataOut32 >> 24 & 0x000000FF);

                SFRA_GUI_memDataPtr32 = SFRA_GUI_memDataPtr32 + 1;
                SFRA_GUI_dataOut32 = *SFRA_GUI_memDataPtr32;
                SFRA_GUI_wordsLeftToGet = SFRA_GUI_wordsLeftToGet - 1;
                SFRA_GUI_sendTaskPtr = 1;
            }
            break;
            default:
            break;
        }
    }

}

