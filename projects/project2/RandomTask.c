/*
 * RandomTask.c
 *
 *  Created on: May 19, 2019
 *      Author: wittich
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// memory mappings
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

// local includes
#include "common/i2c_reg.h"
#include "common/smbus.h"
#include "common/smbus_units.h"


// pilfered and adapted from http://billauer.co.il/blog/2018/01/c-pmbus-xilinx-fpga-kc705/
enum { PM_VOLTAGE, PM_NONVOLTAGE, PM_STATUS, PM_LINEAR11, PM_LINEAR16U, PM_LINEAR16S } pm_types ;

struct pm_list {
  unsigned char command;
  int size;
  char *name;
  char *units;
  int type;
};

static struct pm_list pm_common[] = {
//  { 0x19, 1, "CAPABILITY", "", PM_STATUS },
//  { 0x4F, 2, "OT_FAULT_LIMIT", "", PM_STATUS}, // 0xEBE8
//  { 0x79, 2, "STATUS_WORD", "", PM_STATUS },
//  { 0x7e, 1, "STATUS_CML", "", PM_STATUS }, // 0x0000
  { 0x8d, 2, "READ_TEMPERATURE_1", "C", PM_LINEAR11 },
  { 0x8f, 2, "READ_TEMPERATURE_3", "C", PM_LINEAR11 },
  { 0x88, 2, "READ_VIN", "V", PM_LINEAR11 },
  { 0x8B, 2, "READ_VOUT", "V", PM_LINEAR16U },
 // { 0x89, 2, "READ_IIN", "A", PM_NONVOLTAGE },
//  { 0x98, 1, "PMBUS_REV", "", PM_STATUS}, // 0x22 by default
//  { 0xd3, 2, "VIN_SCALE_MONITOR", "V/V", PM_NONVOLTAGE }
};
const int ncommon = sizeof(pm_common)/sizeof(pm_common[0]);

// prototype of mutex'd print
void Print(const char* str);

extern tSMBus g_sMaster;

tSMBusStatus eStatus = SMBUS_OK;

// playground to test various things
void RandomTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t data[2];

  vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 500 ) );

  const uint8_t addrs[] = { 0x40, 0x44, 0x43, 0x46, 0x45};

  for (;;) {
    for ( uint8_t ps = 0; ps < 5; ++ ps ) {
      char tmp[64];
      // select the appropriate output for the mux
      data[0] = 0x1U<<ps;
      snprintf(tmp, 64, "RDM: Output of mux set to 0x%02x\n", data[0]);
      Print(tmp);
      bool success = writeI2C(I2C1_BASE, 0x70U, data, 1);
      if ( ! success ) {
        Print("RDM: Write of MUX output failed\n");
      }
      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 ));
      data[0] = 0xAAU;
      success = readI2C(I2C1_BASE, 0x70U, data, 1);
      if ( ! success ) {
        Print("RDM: Read of MUX output failed\n");
      }
      else {
        snprintf(tmp, 64, "RDM: read back register on mux to be %02x\n", data[0]);
        Print(tmp);
      }
      // loop over commands
      for (int c = 0; c < ncommon; ++c ) {
//        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 50 ));
//        // read out the temperature register
//        data[0] = 0U; data[1] = 0U;
//        readI2Creg(I2C1_BASE, 0x40U, pm_common[c].command, data, pm_common[c].size);
////        uint16_t data16 = (data[1]<<8) | data[0];
////
////        float temperature = decode_linear11(data16);
////        int val1 = temperature;
////        int val2 = ((temperature-val1)*100.0f);
////        snprintf(tmp, 64, "RDM: %d %s is %d.%02d (%04x)\n", ps, pm_common[c].name,
////                 val1, val2, data16);
        data[0] = 0x0U; data[1] = 0x0U;
        tSMBusStatus r = SMBusMasterByteWordRead(&g_sMaster, addrs[ps], pm_common[c].command,
            data, pm_common[c].size);
        if ( r != SMBUS_OK ) {
          Print("SMBUS command failed\n");
        }
        while ( SMBusStatusGet(&g_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
          vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // busy wait
        }
        if (eStatus != SMBUS_OK ) {
          snprintf(tmp, 64, "RDM: SMBUS ERROR: %d\n", eStatus);
          Print(tmp);
        }
        snprintf(tmp, 64, "RDM: %d %s is 0x%02x %02x\n", ps, pm_common[c].name, data[1], data[0]);
        Print(tmp);
        if ( pm_common[c].type == PM_LINEAR11 ) {
          linear11_val_t ii; ii.raw = (data[1] << 8) | data[0];
          float val = linear11_to_float(ii);
          int tens = val;
          int fraction = (val - tens)*100.0;
          snprintf(tmp, 64, "\t\t%d.%02d (linear11)\n", tens, fraction);
          Print(tmp);
        }
        else if ( pm_common[c].type == PM_LINEAR16U ) {
          uint16_t ii = (data[1] << 8) | data[0];
          float val = linear16u_to_float(ii);
          int tens = val;
          int fraction = (val - tens)*100.0;
          snprintf(tmp, 64,  "\t\t%d.%02d (linear16u)\n", tens, fraction);
          Print(tmp);
        }
        // wait here for the x msec, where x is 2nd argument below.
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 100 ) );
      }
      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 1000 ) );
    }
  }

}
