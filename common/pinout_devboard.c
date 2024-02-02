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
#include "pinsel.h"

#include "common/utils.h"

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
#if defined(PART_TM4C1294NCPDT) || defined(PART_TM4C129ENCPDT)
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



// gpios
#ifdef DEVBOARD
// configure the GPIO pins
#define X(NAME, PPIN, PPORT, LOCALPIN, INPUT, TYPE) \
	if (INPUT){ \
    	MAP_GPIOPinTypeGPIOInput(GPIO_PORT##PPORT##_BASE, GPIO_PIN_##LOCALPIN); \
		MAP_GPIOPadConfigSet(GPIO_PORT##PPORT##_BASE, GPIO_PIN_##LOCALPIN, GPIO_STRENGTH_2MA, TYPE);\
	} \
	else	\
		MAP_GPIOPinTypeGPIOOutput(GPIO_PORT##PPORT##_BASE, GPIO_PIN_##LOCALPIN);

#include "common/gpio_pins_devboard.def"

// The following definition is deprecated
/*
const uint32_t GPIO_PORT_BASE_I2C[] = 
                { GPIO_PORTB_BASE, GPIO_PORTG_BASE
                , GPIO_PORTG_BASE, GPIO_PORTG_BASE
                , GPIO_PORTG_BASE, GPIO_PORTB_BASE
                , GPIO_PORTA_BASE, GPIO_PORTD_BASE
                , GPIO_PORTD_BASE };

const uint32_t GPIO_Pxx_I2CxSCL[] = 
                { GPIO_PB2_I2C0SCL, GPIO_PG0_I2C1SCL
                , GPIO_PL1_I2C2SCL, GPIO_PK4_I2C3SCL
                , GPIO_PK6_I2C4SCL, GPIO_PB0_I2C5SCL
                , GPIO_PA6_I2C6SCL, GPIO_PORTD_BASE
                , GPIO_PORTD_BASE };
const uint32_t GPIO_Pxx_I2CxSDA[] = 
                { GPIO_PB3_I2C0SDA, GPIO_PG1_I2C1SDA
                , GPIO_PN4_I2C2SDA, GPIO_PK5_I2C3SDA
                , GPIO_PK7_I2C4SDA, GPIO_PORTB_BASE
                , GPIO_PA7_I2C6SDA, GPIO_PORTD_BASE
                , GPIO_PORTD_BASE };
*/

///////////////////////////
// configure the I2C pins

// First define the pins, is better to define in a .def file and include it as done for common/i2c_pins_demo.def
// but defining this way is faster, more intuitive for the begining and also
// VScode shows you inmediately if the expansion works good
// #define DEVPINS // now defined in pinsel.h

#define X(NAME, PPORT, SCL_PIN, SDA_PIN, NI2C, RESET, isMASTER) \
    MAP_GPIOPinConfigure(GPIO_P##PPORT##SDA_PIN##_I2C##NI2C##SDA); \
    MAP_GPIOPinTypeI2C(GPIO_PORT##PPORT##_BASE, GPIO_PIN_##SDA_PIN); \
    MAP_GPIOPinConfigure(GPIO_P##PPORT##SCL_PIN##_I2C##NI2C##SCL); \
    MAP_GPIOPinTypeI2CSCL(GPIO_PORT##PPORT##_BASE, GPIO_PIN_##SCL_PIN); 

//#include "common/i2c_pins_demo.def"
I2C_DEVBOARD_PINS

#elif defined(DEMO)
#warning "pins for Demo havne't been defined"
#elif defined(PROTO)
#warning "pins for Demo havne't been defined"
#else
#error "need to define either DEVBOARD, DEMO or PROTO"
#endif


/*
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
*/
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

	// UART for monitoring to zynq
	//
    // Configure the GPIO Pin Mux for PC6
	// for U5RX
    //
	MAP_GPIOPinConfigure(GPIO_PC6_U5RX);
	MAP_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PC7
	// for U5TX
    //
	MAP_GPIOPinConfigure(GPIO_PC7_U5TX);
	MAP_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_7);
#endif

}

// Default values.
#define GPIO_DEFAULT_CM_READY       0x0
#define GPIO_DEFAULT_LED_CM_STATUS  0x0     // 0..3: LED_CM_STATUS_CLOCK, LED_CM_STATUS_KU15P, LED_CM_STATUS_ZU11EG, LED_CM_STATUS_TEMP_ALERT
#define GPIO_DEFAULT_LED_CM_USER    0x00    // 0..7: LED_USER_BLUE_0, LED_USER_BLUE_1 LED_USER_ORANGE_0, LED_USER_ORANGE_1, LED_USER_GREEN_0, LED_USER_GREEN_1, LED_USER_RED_0, LED_USER_RED_1
#define GPIO_DEFAULT_MUX_SEL        0x1     // 0..2: B2B_MUX1_SEL, B2B_MUX2_SEL, LTTC_MUX1_SEL
// Hint: The power down (PD) pin of the multiplexers is active high.
#define GPIO_DEFAULT_MUX_PD         0x0     // 0..2: B2B_MUX1_PD, B2B_MUX2_PD, LTTC_MUX1_PD
#define GPIO_DEFAULT_CLOCK_SEL      0x00    // 0..4: AD_CLK2_KUP_SEL, AD_CLK3_KUP_SEL, AD_CLK4_KUP_SEL, AD_CLK5_ZUP_SEL, CLK_LHC_FPGA_SEL
#define GPIO_DEFAULT_POWER_CTRL     0x00    // 0..7: KUP_CORE_RUN, KUP_P3V3_IO_RUN, KUP_DDR4_TERM_EN, ZUP_CORE_RUN, ZUP_PS_DDR4_TERM_EN, ZUP_PL_DDR4_TERM_EN, FIREFY_P1V8_RUN, FIREFY_P3V3_RUN
#define GPIO_DEFAULT_KUP_CTRL_STAT  0x3     // 0..2: KUP_PROG_B_3V3, KUP_INIT_B_3V3, KUP_DONE_3V3
#define GPIO_DEFAULT_ZUP_CTRL_STAT  0xB     // 0..5: ZUP_PS_PROG_B, ZUP_PS_INIT_B, ZUP_PS_DONE, ZUP_PS_nPOR, ZUP_PS_ERR_STATUS, ZUP_PS_ERR_OUT
#define GPIO_DEFAULT_RESET          0x3     // 0..1: I2C_MUX_nRST, MCU_PEx_nRST
#define GPIO_DEFAULT_SPARE_KUP_ZUP  0x00    // 0..7: MCU_2_KUP_SE0, MCU_2_KUP_SE1, MCU_2_KUP_SE2, MCU_2_KUP_SE3, MCU_2_ZUP_SE0, MCU_2_ZUP_SE1, MCU_2_ZUP_SE2, MCU_2_ZUP_SE3
#define GPIO_DEFAULT_RESERVED       0x00    // 0..2: PWR_CLK, PWR_KU15P, PWR_ZU11EG

void GpioInit_All(void)
{
	// The following is the TODO of initialization from Prodesign
/*
    GpioSet_CmReady(GPIO_DEFAULT_CM_READY);
    GpioSet_LedCmStatus(GPIO_DEFAULT_LED_CM_STATUS);
    GpioSet_LedMcuUser(GPIO_DEFAULT_LED_CM_USER);
    GpioSet_MuxSel(GPIO_DEFAULT_MUX_SEL);
    GpioSet_MuxPD(GPIO_DEFAULT_MUX_PD);
    GpioSet_ClockSel(GPIO_DEFAULT_CLOCK_SEL);
    GpioSet_PowerCtrl(GPIO_DEFAULT_POWER_CTRL);
    GpioSet_KupCtrlStat(GPIO_DEFAULT_KUP_CTRL_STAT);
    GpioSet_ZupCtrlStat(GPIO_DEFAULT_ZUP_CTRL_STAT);
    GpioSet_Reset(GPIO_DEFAULT_RESET);
    GpioSet_SpareKupZup(GPIO_DEFAULT_SPARE_KUP_ZUP);
    GpioSet_Reserved(GPIO_DEFAULT_RESERVED);
*/
//CmReady
	write_gpio_pin(BLADE_POWER_OK, (bool) (GPIO_DEFAULT_CM_READY & 0x1));
//LedCmStatus 0..3: LED_CM_STATUS_CLOCK, LED_CM_STATUS_KU15P, LED_CM_STATUS_ZU11EG, LED_CM_STATUS_TEMP_ALERT
	write_gpio_pin(CLK_DOMAIN_PG, 	(bool) (GPIO_DEFAULT_LED_CM_STATUS & 0x1));
	write_gpio_pin(KUP_DOMAIN_PG, 	(bool) (GPIO_DEFAULT_LED_CM_STATUS & 0x2));
	write_gpio_pin(ZUP_DOMAIN_PG, 	(bool) (GPIO_DEFAULT_LED_CM_STATUS & 0x4));
	write_gpio_pin(TEMP_ERROR, 		(bool) (GPIO_DEFAULT_LED_CM_STATUS & 0x8));
//LedMcuUser  0..7: LED_USER_BLUE_0, LED_USER_BLUE_1 LED_USER_ORANGE_0, LED_USER_ORANGE_1, LED_USER_GREEN_0, LED_USER_GREEN_1, LED_USER_RED_0, LED_USER_RED_1
	write_gpio_pin(MCU_USER_LED0, 	(bool) (GPIO_DEFAULT_LED_CM_USER & 0x1));
	write_gpio_pin(MCU_USER_LED1, 	(bool) (GPIO_DEFAULT_LED_CM_USER & 0x2));
	write_gpio_pin(MCU_USER_LED2, 	(bool) (GPIO_DEFAULT_LED_CM_USER & 0x4));
	write_gpio_pin(MCU_USER_LED3, 	(bool) (GPIO_DEFAULT_LED_CM_USER & 0x8));
	write_gpio_pin(MCU_USER_LED4, 	(bool) (GPIO_DEFAULT_LED_CM_USER & 0x10));
	write_gpio_pin(MCU_USER_LED5, 	(bool) (GPIO_DEFAULT_LED_CM_USER & 0x20));
	write_gpio_pin(MCU_USER_LED6, 	(bool) (GPIO_DEFAULT_LED_CM_USER & 0x40));
	write_gpio_pin(MCU_USER_LED7, 	(bool) (GPIO_DEFAULT_LED_CM_USER & 0x80));
//MuxSel 0..2: B2B_MUX1_SEL, B2B_MUX2_SEL, LTTC_MUX1_SEL
	write_gpio_pin(B2B_MUX1_SEL, 	(bool) (GPIO_DEFAULT_MUX_SEL & 0x1));
	write_gpio_pin(B2B_MUX2_SEL, 	(bool) (GPIO_DEFAULT_MUX_SEL & 0x2));
	write_gpio_pin(LTTC_MUX1_SEL, 	(bool) (GPIO_DEFAULT_MUX_SEL & 0x4));
//MuxPD 0..2: B2B_MUX1_PD, B2B_MUX2_PD, LTTC_MUX1_PD
	write_gpio_pin(B2B_MUX1_PD, 	(bool) (GPIO_DEFAULT_MUX_PD & 0x1));
	write_gpio_pin(B2B_MUX2_PD, 	(bool) (GPIO_DEFAULT_MUX_PD & 0x2));
	write_gpio_pin(LTTC_MUX1_PD, 	(bool) (GPIO_DEFAULT_MUX_PD & 0x4));
//ClockSel 0..4: AD_CLK2_KUP_SEL, AD_CLK3_KUP_SEL, AD_CLK4_KUP_SEL, AD_CLK5_ZUP_SEL, CLK_LHC_FPGA_SEL
	write_gpio_pin(AD_CLK2_KUP_SEL, (bool) (GPIO_DEFAULT_CLOCK_SEL & 0x1));
	write_gpio_pin(AD_CLK3_KUP_SEL, (bool) (GPIO_DEFAULT_CLOCK_SEL & 0x2));
	write_gpio_pin(AD_CLK4_KUP_SEL, (bool) (GPIO_DEFAULT_CLOCK_SEL & 0x4));
	write_gpio_pin(AD_CLK5_ZUP_SEL, (bool) (GPIO_DEFAULT_CLOCK_SEL & 0x8));
	write_gpio_pin(CLK_LHC_FPGA_SEL, (bool) (GPIO_DEFAULT_CLOCK_SEL & 0x10));
//PowerCtrl 0..7: KUP_CORE_RUN, KUP_P3V3_IO_RUN, KUP_DDR4_TERM_EN, ZUP_CORE_RUN, ZUP_PS_DDR4_TERM_EN, ZUP_PL_DDR4_TERM_EN, FIREFY_P1V8_RUN, FIREFY_P3V3_RUN
	write_gpio_pin(KUP_CORE_RUN, 		(bool) (GPIO_DEFAULT_POWER_CTRL & 0x1));
	write_gpio_pin(KUP_P3V3_IO_RUN, 	(bool) (GPIO_DEFAULT_POWER_CTRL & 0x2));
	write_gpio_pin(KUP_DDR4_TERM_EN, 	(bool) (GPIO_DEFAULT_POWER_CTRL & 0x4));
	write_gpio_pin(ZUP_CORE_RUN, 		(bool) (GPIO_DEFAULT_POWER_CTRL & 0x8));
	write_gpio_pin(ZUP_PS_DDR4_TERM_EN, (bool) (GPIO_DEFAULT_POWER_CTRL & 0x10));
	write_gpio_pin(ZUP_PL_DDR4_TERM_EN, (bool) (GPIO_DEFAULT_POWER_CTRL & 0x20));
	//write_gpio_pin(FIREFY_P1V8_RUN, 	(bool) (GPIO_DEFAULT_POWER_CTRL & 0x40));
	write_gpio_pin(FIREFY_P3V3_RUN, 	(bool) (GPIO_DEFAULT_POWER_CTRL & 0x80));
//KupCtrlStat 0..2: KUP_PROG_B_3V3, KUP_INIT_B_3V3, KUP_DONE_3V3
	write_gpio_pin(KUP_PROG_B_3V3, 	(bool) (GPIO_DEFAULT_KUP_CTRL_STAT & 0x1));
	write_gpio_pin(KUP_INIT_B_3V3, 	(bool) (GPIO_DEFAULT_KUP_CTRL_STAT & 0x2));
	//write_gpio_pin(KUP_DONE_3V3, (bool) (GPIO_DEFAULT_KUP_CTRL_STAT & 0x4));
//ZupCtrlStat 0..5: ZUP_PS_PROG_B, ZUP_PS_INIT_B, ZUP_PS_DONE, ZUP_PS_nPOR, ZUP_PS_ERR_STATUS, ZUP_PS_ERR_OUT
	write_gpio_pin(ZUP_PS_PROG_B, 	(bool) (GPIO_DEFAULT_ZUP_CTRL_STAT & 0x1));
	write_gpio_pin(ZUP_PS_INIT_B, 	(bool) (GPIO_DEFAULT_ZUP_CTRL_STAT & 0x2));
	//	write_gpio_pin(ZUP_PS_DONE, (bool) (GPIO_DEFAULT_ZUP_CTRL_STAT & 0x4));
	write_gpio_pin(ZUP_PS_nPOR, 	(bool) (GPIO_DEFAULT_ZUP_CTRL_STAT & 0x8));
	//	write_gpio_pin(ZUP_PS_ERR_STATUS, (bool) (GPIO_DEFAULT_ZUP_CTRL_STAT & 0x10));
	//	write_gpio_pin(ZUP_PS_ERR_OUT, (bool) (GPIO_DEFAULT_ZUP_CTRL_STAT & 0x20));
// Reset 0..1: I2C_MUX_nRST, MCU_PEx_nRST
	write_gpio_pin(I2C_MUX_nRST, 	(bool) (GPIO_DEFAULT_RESET & 0x1));
	write_gpio_pin(MCU_PEx_nRST, 	(bool) (GPIO_DEFAULT_RESET & 0x2));
//SpareKupZup 0..7: MCU_2_KUP_SE0, MCU_2_KUP_SE1, MCU_2_KUP_SE2, MCU_2_KUP_SE3, MCU_2_ZUP_SE0, MCU_2_ZUP_SE1, MCU_2_ZUP_SE2, MCU_2_ZUP_SE3
	write_gpio_pin(MCU_2_KUP_SE0, 	(bool) (GPIO_DEFAULT_SPARE_KUP_ZUP & 0x1));
	write_gpio_pin(MCU_2_KUP_SE1, 	(bool) (GPIO_DEFAULT_SPARE_KUP_ZUP & 0x2));
	write_gpio_pin(MCU_2_KUP_SE2, 	(bool) (GPIO_DEFAULT_SPARE_KUP_ZUP & 0x4));
	write_gpio_pin(MCU_2_KUP_SE3, 	(bool) (GPIO_DEFAULT_SPARE_KUP_ZUP & 0x8));
	write_gpio_pin(MCU_2_ZUP_SE0, 	(bool) (GPIO_DEFAULT_SPARE_KUP_ZUP & 0x10));
	write_gpio_pin(MCU_2_ZUP_SE1, 	(bool) (GPIO_DEFAULT_SPARE_KUP_ZUP & 0x20));
	write_gpio_pin(MCU_2_ZUP_SE2, 	(bool) (GPIO_DEFAULT_SPARE_KUP_ZUP & 0x40));
	write_gpio_pin(MCU_2_ZUP_SE3, 	(bool) (GPIO_DEFAULT_SPARE_KUP_ZUP & 0x80));
//Reserved 0..2: PWR_CLK, PWR_KU15P, PWR_ZU11EG  (these names are not correct)
	write_gpio_pin(CLK_PM_CTRL0, 	(bool) (GPIO_DEFAULT_RESERVED & 0x1));
	write_gpio_pin(KUP_PM_CTRL0, 	(bool) (GPIO_DEFAULT_RESERVED & 0x2));
	write_gpio_pin(ZUP_PM_CTRL0, 	(bool) (GPIO_DEFAULT_RESERVED & 0x4));

}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

