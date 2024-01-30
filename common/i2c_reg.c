// higher level utilities for I2C
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "common/i2c_reg.h"
#include "common/pinsel.h"
#include "common/utils.h"

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#endif // USE_FREERTOS

// This array helps us simplify the use of different I2C devices in the board.
const uint32_t I2C_BASE[] = {I2C0_BASE, I2C1_BASE, I2C2_BASE, I2C3_BASE, I2C4_BASE,
                             I2C5_BASE, I2C6_BASE, I2C7_BASE, I2C8_BASE, I2C9_BASE};
const uint32_t SYSCTL_PERIPH_I2C[] = 
                { SYSCTL_PERIPH_I2C0, SYSCTL_PERIPH_I2C1
                , SYSCTL_PERIPH_I2C2, SYSCTL_PERIPH_I2C3
                , SYSCTL_PERIPH_I2C4, SYSCTL_PERIPH_I2C5
                , SYSCTL_PERIPH_I2C6, SYSCTL_PERIPH_I2C7
                , SYSCTL_PERIPH_I2C8, SYSCTL_PERIPH_I2C9 };

#if defined(REV1) || defined(REV2)
// initialize I2C module 0
// Slightly modified version of TI's example code
void initI2C0(const uint32_t sysclockfreq)
{
  // enable I2C module 0
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  //
  // Wait for the I2C0 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C0);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0))
    ;

  // Enable the module as a slave
  uint8_t slave_addr = 0x50;
  MAP_I2CSlaveInit(I2C0_BASE, slave_addr);
  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C0 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  // MAP_I2CMasterInitExpClk(I2C0_BASE, sysclockfreq, false);
  // while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));

  // clear I2C FIFOs
  // HWREG(I2C0_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C0_BASE);
  MAP_I2CTxFIFOFlush(I2C0_BASE);

  // no resets here
}

// initialize I2C module 1
// Slightly modified version of TI's example code
void initI2C1(const uint32_t sysclockfreq)
{
  // enable I2C module 1
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
  //
  // Wait for the I2C1 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C1);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1))
    ;

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C1 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C1_BASE, sysclockfreq, false);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1))
    ;

  // clear I2C FIFOs
  // HWREG(I2C1_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C1_BASE);
  MAP_I2CTxFIFOFlush(I2C1_BASE);

  // toggle relevant reset
  write_gpio_pin(_PWR_I2C_RESET, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(_PWR_I2C_RESET, 0x1); // active low
}
// i2c 2 is for the clock chips
void initI2C2(const uint32_t sysclockfreq)
{
  // enable I2C module 2
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
  //
  // Wait for the I2C1 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C2);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2))
    ;

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C1 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C2_BASE, sysclockfreq, false);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2))
    ;

  // clear I2C FIFOs
  // HWREG(I2C1_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C2_BASE);
  MAP_I2CTxFIFOFlush(I2C2_BASE);

  // toggle relevant reset
  write_gpio_pin(_CLOCKS_I2C_RESET, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(_CLOCKS_I2C_RESET, 0x1); // active low
}

// I2C controller 3 is for V_OPTICS on the CM Rev 1
void initI2C3(const uint32_t sysclockfreq)
{
  // enable I2C module 3
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
  //
  // Wait for the I2C3 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C3)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C3);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C3))
    ;

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C3 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C3_BASE, sysclockfreq, false);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C3))
    ;

  // clear I2C FIFOs
  // HWREG(I2C3_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C3_BASE);
  MAP_I2CTxFIFOFlush(I2C3_BASE);

  // toggle relevant reset
  write_gpio_pin(_F2_OPTICS_I2C_RESET, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(_F2_OPTICS_I2C_RESET, 0x1); // active low
}

// I2C controller 4 is for F1_OPTICS on the CM
void initI2C4(const uint32_t sysclockfreq)
{
  // enable I2C module 3
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C4);
  //
  // Wait for the I2C4 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C4)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C4);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C4);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C4);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C4))
    ;

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C4 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C4_BASE, sysclockfreq, false);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C4))
    ;

  // clear I2C FIFOs
  // HWREG(I2C4_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C4_BASE);
  MAP_I2CTxFIFOFlush(I2C4_BASE);

  // toggle relevant reset
  write_gpio_pin(_F1_OPTICS_I2C_RESET, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(_F1_OPTICS_I2C_RESET, 0x1); // active low
}

#ifdef REV2
// I2C controller 5 is for FPGAs on the CM
void initI2C5(const uint32_t sysclockfreq)
{
  // enable I2C module 5
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C5);
  //
  // Wait for the I2C5 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C5)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C5);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C5);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C5);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C5))
    ;

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C5 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C5_BASE, sysclockfreq, false);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C5))
    ;

  // clear I2C FIFOs
  // HWREG(I2C5_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C5_BASE);
  MAP_I2CTxFIFOFlush(I2C5_BASE);

  // toggle relevant reset
  write_gpio_pin(_FPGA_I2C_RESET, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(_FPGA_I2C_RESET, 0x1); // active low
}
#elif defined(REV1)
// I2C controller 6 is for FPGAs on the CM
void initI2C6(const uint32_t sysclockfreq)
{
  // enable I2C module 3
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C6);
  //
  // Wait for the I2C6 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C6)) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C6);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C6);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C6);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C6))
    ;

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C6 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C6_BASE, sysclockfreq, false);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C6))
    ;

  // clear I2C FIFOs
  // HWREG(I2C6_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C6_BASE);
  MAP_I2CTxFIFOFlush(I2C6_BASE);

  // toggle relevant reset
  write_gpio_pin(_FPGA_I2C_RESET, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(_FPGA_I2C_RESET, 0x1); // active low
}
#endif

#endif //defined(REV1) || defined(REV2)
// L0MDT specific




#if defined(DEVBOARD) || defined (DEMO) || defined (PROTO)
void initI2C(int I2C_number, const uint32_t sysclockfreq)
{
  // enable I2C module 3
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C[I2C_number]);
  //
  // Wait for the I2C6 module to be ready.
  //
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C[I2C_number])){
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  //
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C[I2C_number]);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C[I2C_number]);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C[I2C_number]);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C[I2C_number])){}
    

  // Enable and initialize the I2C master module.  Use the system clock for
  // the I2C6 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  MAP_I2CMasterInitExpClk(I2C_BASE[I2C_number], sysclockfreq, false);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C[I2C_number])){}
    

  // clear I2C FIFOs
  // HWREG(I2C6_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C_BASE[I2C_number]);
  MAP_I2CTxFIFOFlush(I2C_BASE[I2C_number]);


}

void initI2C_as_slave(int I2C_number, const uint32_t sysclockfreq)
{
  // enable I2C module 0
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C[I2C_number]);
  //
  // Wait for the I2C0 module to be ready.
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C[I2C_number])) {
  }

  // Stop the Clock, Reset and Enable I2C Module
  // in Master Function
  MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C[I2C_number]);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C[I2C_number]);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C[I2C_number]);

  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C[I2C_number]))
    ;

  // Enable the module as a slave
  uint8_t slave_addr = 0x50;
  MAP_I2CSlaveInit(I2C_BASE[I2C_number], slave_addr);

  // clear I2C FIFOs
  // HWREG(I2C0_BASE + I2C_0_FIFOCTL) = 80008000;
  MAP_I2CRxFIFOFlush(I2C_BASE[I2C_number]);
  MAP_I2CTxFIFOFlush(I2C_BASE[I2C_number]);

  // no resets here
}
#endif


#if defined(DEVBOARD) || defined (DEMO)
void initI2C_all(const uint32_t sysclockfreq)
{
  initI2C(0,sysclockfreq);
  initI2C(1,sysclockfreq);
  initI2C(2,sysclockfreq);
  initI2C(3,sysclockfreq);
  initI2C(4,sysclockfreq);
  initI2C(5,sysclockfreq);
  initI2C(6,sysclockfreq);
  initI2C_as_slave(7,sysclockfreq);
  initI2C(8,sysclockfreq);
  write_gpio_pin(I2C_MUX_nRST, 0x0); // active low
  MAP_SysCtlDelay(sysclockfreq / 10);
  ;
  write_gpio_pin(I2C_MUX_nRST, 0x1); // active low
}
#elif defined(PROTO)
#error "I2C functions for ptototype has not yet been implemented"
#endif