//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
// a comment
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Blinky (blinky)</h1>
//!
//! A very simple example that blinks the on-board LED using direct register
//! access.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
  while(1);
}
#endif

#ifdef REV2
#define UART_FRONTPANEL      UART0_BASE
#define UARTx_BASE           UART0_BASE
#define LED_PERIPH           SYSCTL_PERIPH_GPIOP
#define LED_BASE             GPIO_PORTP_BASE
#define LED_PIN_0              GPIO_PIN_3
#else
#ifdef PART_TM4C1290NCPDT 
#warning "PART_TM4C1290NCPDT"
#define LED_PERIPH_0 SYSCTL_PERIPH_GPIOJ
#define LED_PERIPH_1 SYSCTL_PERIPH_GPIOP
#define LED_BASE_0 GPIO_PORTJ_BASE
#define LED_BASE_1 GPIO_PORTP_BASE
#define LED_PIN_0 GPIO_PIN_0
#define LED_PIN_1 GPIO_PIN_1
#define LED_PIN_2 GPIO_PIN_0

#define UART_FRONTPANEL      UART4_BASE
#define UARTx_BASE           UART4_BASE
#elif defined(PART_TM4C1294NCPDT ) || defined(PART_TM4C129ENCPDT)
#warning "PART_TM4C1294NCPDT or TM4C129ENCPDT"
#define LED_PERIPH_0 SYSCTL_PERIPH_GPION
#define LED_PERIPH_1 SYSCTL_PERIPH_GPIOF
#define LED_BASE_0 GPIO_PORTN_BASE
#define LED_BASE_1 GPIO_PORTF_AHB_BASE
#define LED_PIN_0 GPIO_PIN_0
#define LED_PIN_1 GPIO_PIN_1
#define LED_PIN_2 GPIO_PIN_0

#define UART_FRONTPANEL      UART0_BASE
#define UARTx_BASE           UART0_BASE
#else
#error PART needs to be specified.
#endif
#endif
//*****************************************************************************
//
// Blink the on-board LEDs
//
//*****************************************************************************
int
main(void)
{
  volatile uint32_t ui32Loop;

  //
  // Enable the GPIO ports that are used for the on-board LED.
  // Two are J; one is P
  //
  MAP_SysCtlPeripheralEnable(LED_PERIPH_0);

  //
  // Check if the peripheral access is enabled.
  //
  while(!MAP_SysCtlPeripheralReady(LED_PERIPH_0)) {
  }

  // Enable the GPIO port that is used for the on-board LED.
  //
  MAP_SysCtlPeripheralEnable(LED_PERIPH_1);

  //
  // Check if the peripheral access is enabled.
  //
  while(!MAP_SysCtlPeripheralReady(LED_PERIPH_1)) {
  }


  //
  // Enable the GPIO pin for the LEDs (PN0).
  // Pins PJ0 (blue), PJ1 (green) and PP0 (red)
  // Set the direction as output, and
  // enable the GPIO pin for digital function.
  //
  // Configure the GPIO Pin Mux for PJ0
  // for GPIO_PJ0
  //
  MAP_GPIOPinTypeGPIOOutput(LED_BASE_0, LED_PIN_0);
    
  //
  // Configure the GPIO Pin Mux for PJ1
  // for GPIO_PJ1
  //
  MAP_GPIOPinTypeGPIOOutput(LED_BASE_0, LED_PIN_1);
  //
  // Configure the GPIO Pin Mux for PP0
  // for GPIO_PP0
  //
  MAP_GPIOPinTypeGPIOOutput(LED_BASE_1, LED_PIN_2);


  // original code: to be deleted
  //GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

  //
  // Loop forever.
  //
  while(1) {
    //
    // Turn on the LEDs -- RED
    
    // 
    MAP_GPIOPinWrite(LED_BASE_0, LED_PIN_0, LED_PIN_0);

    //
    // Delay for a bit.
    //
    for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++) {
    }

    //
    // Turn off the LED.
    //
    MAP_GPIOPinWrite(LED_BASE_0, LED_PIN_0, 0x0);


    //
    // Delay for a bit.
    //
    for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++) {
    }
    // Turn on the LEDs -- GREEN
    
    // 
    MAP_GPIOPinWrite(LED_BASE_0, LED_PIN_1, LED_PIN_1);

    //
    // Delay for a bit.
    //
    for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++) {
    }

    //
    // Turn off the LED.
    //
    MAP_GPIOPinWrite(LED_BASE_0, LED_PIN_1, 0x0);


    //
    // Delay for a bit.
    //
    for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++) {
    }
    // Turn on the LEDs -- BLUE
    
    // 
    MAP_GPIOPinWrite(LED_BASE_1, LED_PIN_2, LED_PIN_2);

    //
    // Delay for a bit.
    //
    for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++) {
    }

    //
    // Turn off the LED.
    //
    MAP_GPIOPinWrite(LED_BASE_1, LED_PIN_2, 0x0);


    //
    // Delay for a bit.
    //
    for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++) {
    }
    
  }
}
