//###########################################################################
//
// FILE:   sfra_gui_scicomms_driverlib.h
//
// TITLE:  Comms kernel as an interface to SFRA GUI header file
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
#ifndef SFRA_GUI_H
#define SFRA_GUI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "driverlib.h"
#include "device.h"
#include "sfra_f32.h"

#define SFRA_GUI_PKT_SIZE 6
#define SFRA_GUI_CMD_NUMBER 16
#define SFRA_GUI_MAX_CMD_NUM 8


#define SFRA_GUI_PLOT_GH_H  1
#define SFRA_GUI_PLOT_GH_CL 2

//
//! \brief Configures the SFRA_GUI module
//! \param sci_base  Base address of the SCI module used by the SFRA GUI
//! \param vbus_clk  Frequency of the VBUS, used by the SCI module
//! \param baudrate  baudrate used by the SFRA GUI
//! \param scirx_gpio_pin  GPIO pin used for SCI_RX
//! \param scirx_gpio_pin_config  GPIO pin config used for SCI_RX
//! \param scitx_gpio_pin  GPIO pin used for SCI_TX
//! \param scitx_gpio_pin_config  GPIO pin config used for SCI_TX
//! \param led_indicator_flag  Flag to indicate if LED toggle for SFRA_GUI is
//!                            enabled, 1 -> Enable , anything else Disable
//! \param led_gpio_pin  GPIO pin used for LED, if led_flag_indicator is 1
//!                      otherwise pass 0
//! \param led_gpio_pin_config  GPIO pin config value for LED,
//!                      if led_flag_indicator is 1 otherwise pass 0
//! \param *sfra Pointer to sfra object
//! \param plot option used to select what SFRA GUI will plot,
//!                      1 -  GH & H
//!                      2 -  CL & H
//! \return None
//!
void SFRA_GUI_config( volatile uint32_t sci_base,
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
                      uint16_t plot_option);

//
//! \brief Runs the serial host comms GUI ,
//!        needs to be called at ~100ms for proper function
//! \param *sfra Pointer to sfra object
//! \return None
//!
void SFRA_GUI_runSerialHostComms(SFRA_F32 *sfra);



#ifdef __cplusplus
}
#endif // extern "C"

#endif // end of SFRA_F32_H definition


