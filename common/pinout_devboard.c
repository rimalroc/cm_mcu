//*****************************************************************************
//
// Configure the device pins for different signals
//
// This was modified taken as a base pinout_rev1.c

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "pinout.h"

//*****************************************************************************
//
//! \addtogroup pinout_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! Configures the device pins for the customer specific usage.
//!
//! \return None.
//
//*****************************************************************************

void
PinoutSet(void)
{
#if defined(PART_TM4C1294NCPDT)
#warning "using alternative pins definition"



    // Enable Peripheral Clocks 
    //
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);


    //
    // Configure the GPIO Pin Mux for PE3
	// for AIN0
    //
	MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PK1
	// for AIN17
    //
	MAP_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PK3
	// for AIN19
    //
	MAP_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PK0
	// for AIN16
    //
	MAP_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PK2
	// for AIN18
    //
	MAP_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_2);

	//
	// Unlock the Port Pin and Set the Commit Bit
	//
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE+GPIO_O_CR)   |= GPIO_PIN_7;
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = 0x0;

    //
    // Configure the GPIO Pin Mux for PM7
	// for GPIO_PM7
    //
//	MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_7);
//	MAP_GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

// gpios

#define X(NAME, PPIN, PPORT, LOCALPIN, INPUT) \
    MAP_GPIOPinTypeGPIOInput(GPIO_PORT##PPORT##_BASE, GPIO_PIN_##LOCALPIN);

#ifdef DEVBOARD
#include "common/gpio_pins_devboard.def"
#elif defined(DEMO)
#warning "pins for Demo havne't been defined"
#elif defined(PROTO)
#warning "pins for Demo havne't been defined"
#else
#error "need to define either DEVBOARD, DEMO or PROTO"
#endif

    //
    // Configure the GPIO Pin Mux for PB2
	// for I2C0SCL
    //
	MAP_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PB3
	// for I2C0SDA
    //
	MAP_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	MAP_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PG0
	// for I2C1SCL
    //
	MAP_GPIOPinConfigure(GPIO_PG0_I2C1SCL);
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PG1
	// for I2C1SDA
    //
	MAP_GPIOPinConfigure(GPIO_PG1_I2C1SDA);
	MAP_GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PL1
	// for I2C2SCL
    //
	MAP_GPIOPinConfigure(GPIO_PL1_I2C2SCL);
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTL_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PN4
	// for I2C2SDA
    //
	MAP_GPIOPinConfigure(GPIO_PN4_I2C2SDA);
	MAP_GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PK4
	// for I2C3SCL
    //
	MAP_GPIOPinConfigure(GPIO_PK4_I2C3SCL);
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTK_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PK5
	// for I2C3SDA
    //
	MAP_GPIOPinConfigure(GPIO_PK5_I2C3SDA);
	MAP_GPIOPinTypeI2C(GPIO_PORTK_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PK6
	// for I2C4SCL
    //
	MAP_GPIOPinConfigure(GPIO_PK6_I2C4SCL);
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTK_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PK7
	// for I2C4SDA
    //
	MAP_GPIOPinConfigure(GPIO_PK7_I2C4SDA);
	MAP_GPIOPinTypeI2C(GPIO_PORTK_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PA6
	// for I2C6SCL
    //
	MAP_GPIOPinConfigure(GPIO_PA6_I2C6SCL);
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PA7
	// for I2C6SDA
    //
	MAP_GPIOPinConfigure(GPIO_PA7_I2C6SDA);
	MAP_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PA0
	// for U0RX
    //
	MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PA1
	// for U0TX
    //
	MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PB0
	// for U1RX
    //
	MAP_GPIOPinConfigure(GPIO_PB0_U1RX);
	MAP_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PB1
	// for U1TX
    //
	MAP_GPIOPinConfigure(GPIO_PB1_U1TX);
	MAP_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PA2
	// for U4RX
    //
	MAP_GPIOPinConfigure(GPIO_PA2_U4RX);
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PA3
	// for U4TX
    //
	MAP_GPIOPinConfigure(GPIO_PA3_U4TX);
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_3);
#endif

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

