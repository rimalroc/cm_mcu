/*
 * SensorControl.c
 *
 *  Created on: Jan 14, 2021
 *      Author: fatimayousuf
 */

#include <strings.h>
#include <sys/_types.h>
#include "parameters.h"
#include "SensorControl.h"
#include "Semaphore.h"
#include "common/smbus_helper.h"
#include "Tasks.h"

// Register definitions
// -------------------------------------------------
// 8 bit 2's complement signed int, valid from 0-80 C, LSB is 1 deg C
// Same address for 4 XCVR and 12 Tx/Rx devices

// two bytes, 12 FF to be disabled
#define ECU0_14G_TX_DISABLE_REG 0x34U
// one byte, 4 FF to be enabled/disabled (only 4 LSB are used)
#define ECU0_25G_XVCR_TX_DISABLE_REG 0x56U
// two bytes, 12 FF to be disabled
#define ECU0_14G_RX_DISABLE_REG 0x34U
// one byte, 4 FF to be enabled/disabled (only 4 LSB are used)
#define ECU0_25G_XVCR_RX_DISABLE_REG 0x35U
// one byte, 4 FF to be enabled/disabled (4 LSB are Rx, 4 LSB are Tx)
#define ECU0_25G_XVCR_CDR_REG 0x62U
// two bytes, 12 FF to be enabled/disabled. The byte layout
// is a bit weird -- 0-3 on byte 4a, 4-11 on byte 4b
#define ECU0_25G_TXRX_CDR_REG 0x4AU





// dump monitor information
/*
BaseType_t psmon_ctl(int argc, char **argv, char *m)
{
  BaseType_t i1 = strtol(argv[1], NULL, 10);

  if (i1 < 0 || i1 >= dcdc_args.n_commands) {
    snprintf(m, SCRATCH_SIZE, "%s: Invalid argument, must be between 0 and %d\r\n", argv[0],
             dcdc_args.n_commands - 1);
    return pdFALSE;
  }
  // update times, in seconds
  TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
  TickType_t last = pdTICKS_TO_MS(dcdc_args.updateTick) / 1000;
  int copied = 0;
  if (checkStale(last, now)) {
    int mins = (now - last) / 60;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s (0x%02x)\r\n",
                     dcdc_args.commands[i1].name, dcdc_args.commands[i1].command);
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "SUPPLY %s\r\n", dcdc_args.devices[ps].name);
    for (int page = 0; page < dcdc_args.n_pages; ++page) {
      float val = dcdc_args.pm_values[ps * (dcdc_args.n_commands * dcdc_args.n_pages) +
                                      page * dcdc_args.n_commands + i1];
      int tens, frac;
      float_to_ints(val, &tens, &frac);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "VALUE %02d.%02d\t", tens, frac);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
  }

  return pdFALSE;
}
*/
// send power control commands
extern struct gpio_pin_t oks[N_PS_OKS];
BaseType_t power_ctl(int argc, char **argv, char *m)
{
  int s = SCRATCH_SIZE;

  uint32_t message;
  if (strncmp(argv[1], "on", 2) == 0) {
    message = PS_ON; // turn on power supply
  }
  else if (strncmp(argv[1], "off", 3) == 0) {
    message = PS_OFF; // turn off power supply
  }
  else if (strncmp(argv[1], "clearfail", 9) == 0) {
    message = PS_ANYFAIL_ALARM_CLEAR;
  }
  else if (strncmp(argv[1], "status", 5) == 0) { // report status to UART
    int copied = 0;
    static int i = 0;
    if (i == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "State machine state: %s\r\n",
                         getPowerControlStateName(getPowerControlState()));
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "External Alarm: %d\r\n",
                         getPowerControlExternalAlarmState());
    }
    for (; i < N_PS_OKS; ++i) {
      enum ps_state j = getPSStatus(i);
      char *c;
      switch (j) {
        case PWR_UNKNOWN:
          c = "PWR_UNKNOWN";
          break;
        case PWR_ON:
          c = "PWR_ON";
          break;
        case PWR_OFF:
          c = "PWR_OFF";
          break;
        case PWR_DISABLED:
          c = "PWR_DISABLED";
          break;
        case PWR_FAILED:
          c = "PWR_FAILED";
          break;
        default:
          c = "UNKNOWN";
          break;
      }

      copied +=
          snprintf(m + copied, SCRATCH_SIZE - copied, "%16s: %s\r\n", oks[i].name, c);
      if ((SCRATCH_SIZE - copied) < 20 && (i < N_PS_OKS)) {
        ++i;
        return pdTRUE;
      }
    }
    i = 0;
    return pdFALSE;
  }
  else if (strncmp(argv[1], "igmask", 6) == 0) { // report ignore mask to UART
    int copied = 0;
    static int i = 0;
    uint16_t ignore_mask = getPowerControlIgnoreMask();
    for (; i < N_PS_OKS; ++i) {
      BaseType_t ignored = (ignore_mask & (0x1U << i)) != 0;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-16s: %ld\r\n", oks[i].name, ignored);
      if ((SCRATCH_SIZE - copied) < 20 && (i < N_PS_OKS)) {
        ++i;
        return pdTRUE;
      }
    }
    i = 0;
    return pdFALSE;
  }
  else {
    snprintf(m, s, "power_ctl: invalid argument %s received\r\n", argv[1]);
    return pdFALSE;
  }
  // Send a message to the power supply task, if needed
  xQueueSendToBack(xPwrQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}

// takes 1-2 arguments
BaseType_t alarm_ctl(int argc, char **argv, char *m)
{
  int s = SCRATCH_SIZE;
  if (argc < 2) {
    snprintf(m, s, "%s: need one or more arguments\r\n", argv[0]);
    return pdFALSE;
  }

  uint32_t message;
  if (strncmp(argv[1], "clear", 4) == 0) {
    message = ALM_CLEAR_ALL; // clear all alarms
    xQueueSendToBack(xAlmQueue, &message, pdMS_TO_TICKS(10));
    // xQueueSendToBack(voltAlarmTask.xAlmQueue, &message, pdMS_TO_TICKS(10));
    m[0] = '\0'; // no output from this command

    return pdFALSE;
  }
  else if (strncmp(argv[1], "status", 5) == 0) { // report status to UART
    int copied = 0;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: ALARM status\r\n", argv[0]);
    uint32_t stat = getTempAlarmStatus();
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Raw: 0x%08lx\r\n", stat);

    float ff_val = getAlarmTemperature(FF);
    int tens, frac;
    float_to_ints(ff_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP FFLY: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_FIREFLY_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    float fpga_val = getAlarmTemperature(FPGA);
    float_to_ints(fpga_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP FPGA: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_FPGA_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    float dcdc_val = getAlarmTemperature(DCDC);
    float_to_ints(dcdc_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP DCDC: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_DCDC_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    float tm4c_val = getAlarmTemperature(TM4C);
    float_to_ints(tm4c_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP TM4C: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_TM4C_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    uint32_t adc_volt_stat = getVoltAlarmStatus();
    float voltthres = getAlarmVoltageThres() * 100;
    float_to_ints(voltthres, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "VOLT ADC: %s (for FPGAs) \t Threshold: +/-%02d.%02d %%\r\n",
                 (adc_volt_stat) ? "ALARM" : "GOOD", tens, frac);

    configASSERT(copied < SCRATCH_SIZE);

    return pdFALSE;
  }
  else if (strcmp(argv[1], "settemp") == 0) {
    if (argc != 4) {
      snprintf(m, s, "Invalid command\r\n");
      return pdFALSE;
    }
    float newtemp = (float)strtol(argv[3], NULL, 10);
    char *device = argv[2];
    if (!strncasecmp(device, "ff", 2)) {
      setAlarmTemperature(FF, newtemp);
      snprintf(m, s, "%s: set Firefly alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    if (!strncasecmp(device, "fpga", 4)) {
      setAlarmTemperature(FPGA, newtemp);
      snprintf(m, s, "%s: set FPGA alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    if (!strncasecmp(device, "dcdc", 4)) {
      setAlarmTemperature(DCDC, newtemp);
      snprintf(m, s, "%s: set DCDC alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    if (!strncasecmp(device, "tm4c", 4)) {
      setAlarmTemperature(TM4C, newtemp);
      snprintf(m, s, "%s: set TM4C alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    else {
      snprintf(m, s, "%s is not a valid device.\r\n", argv[2]);
      return pdFALSE;
    }
  }
  else if (strcmp(argv[1], "setvoltthres") == 0) {
    if (argc != 3) {
      snprintf(m, s, "Invalid command\r\n");
      return pdFALSE;
    }
    float voltthres = (float)strtol(argv[2], NULL, 10);
    setAlarmVoltageThres(voltthres);
    snprintf(m, s, "alarm voltages are set their threshold by +/-%s %% \r\n", argv[2]);
    return pdFALSE;
  }
  else {
    snprintf(m, s, "%s: invalid argument %s received\r\n", argv[0], argv[1]);
    return pdFALSE;
  }
  return pdFALSE;
}

// send LED commands
BaseType_t led_ctl(int argc, char **argv, char *m)
{

  BaseType_t i1 = strtol(argv[1], NULL, 10);

  BaseType_t ones = i1 % 10;
  BaseType_t tens = i1 / 10; // integer truncation

  uint32_t message = HUH; // default: message not understood
  if (ones < 5 && tens > 0 && tens < 4) {
    message = i1;
  }
  // Send a message to the LED task
  xQueueSendToBack(xLedQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}

// this command takes no arguments
BaseType_t adc_ctl(int argc, char **argv, char *m)
{
  int copied = 0;

  static int whichadc = 0;
  if (whichadc == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "ADC outputs\r\n");
  }
  for (; whichadc < ADC_CHANNEL_COUNT; ++whichadc) {
    float val = getADCvalue(whichadc);
    int tens, frac;
    float_to_ints(val, &tens, &frac);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%14s: %02d.%02d\r\n",
                       getADCname(whichadc), tens, frac);
    if ((SCRATCH_SIZE - copied) < 20 && (whichadc < 20)) {
      ++whichadc;
      return pdTRUE;
    }
  }
  whichadc = 0;
  return pdFALSE;
}





// dump clock monitor information
BaseType_t clkmon_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  static int c = 0;
  BaseType_t i = strtol(argv[1], NULL, 10);

  if (i < 0 || i > 4) {
    snprintf(m, SCRATCH_SIZE, "%s: Invalid argument %s\r\n", argv[0], argv[1]);
    return pdFALSE;
  }
  // print out header once
  if (c == 0) {
    const char *clk_ids[5] = {"0A", "0B", "1A", "1B", "1C"};
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Monitoring SI clock with id R%s\r\n",
                       clk_ids[i]);
    char *header = "REG_TABLE";
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-15s REG_ADDR BIT_MASK  VALUE \r\n", header);
  }
  // update times, in seconds
  TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
  TickType_t last = pdTICKS_TO_MS(clockr0a_args.updateTick) / 1000;

  if (checkStale(last, now)) {
    unsigned mins = (now - last) / 60;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "%s: stale data, last update %u minutes ago\r\n", argv[0], mins);
  }
  // i = 0 corresponds to SI5341, others to SI5395
  if (i == 0) {

    for (; c < clockr0a_args.n_commands; ++c) {
      uint8_t val = clockr0a_args.sm_values[c];
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-15s : 0x%04x   0x%02x    0x%04x\r\n", clockr0a_args.commands[c].name, clockr0a_args.commands[c].command, clockr0a_args.commands[c].bit_mask, val);

      // copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
      if ((SCRATCH_SIZE - copied) < 50) {
        ++c;
        return pdTRUE;
      }
    }
    if (c % 2 == 1) {
      m[copied++] = '\r';
      m[copied++] = '\n';
      m[copied] = '\0';
    }
    c = 0;
  }
  // i = 0 corresponds to SI5341, others to SI5395
  else {

    for (; c < clock_args.n_commands; ++c) {
      uint8_t val = clock_args.sm_values[(i - 1) * (clock_args.n_commands * clock_args.n_pages) + c];
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-15s : 0x%04x   0x%02x    0x%04x\r\n", clock_args.commands[c].name, clock_args.commands[c].command, clock_args.commands[c].bit_mask, val);
      if ((SCRATCH_SIZE - copied) < 50) {
        ++c;
        return pdTRUE;
      }
    }
    if (c % 2 == 1) {
      m[copied++] = '\r';
      m[copied++] = '\n';
      m[copied] = '\0';
    }
    c = 0;
  }
  // get and print out the file name
  char progname_clkdesgid[CLOCK_PROGNAME_REG_NAME];     // program name from DESIGN_ID register of clock chip
  char progname_eeprom[CLOCK_EEPROM_PROGNAME_REG_NAME]; // program name from eeprom
  getClockProgram(i, progname_clkdesgid, progname_eeprom);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Program (read from clock chip): %s", progname_clkdesgid);
  if (strncmp(progname_clkdesgid, "5395ABP1", 3) == 0 || strncmp(progname_clkdesgid, "5341ABP1", 3) == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, " (not found)");
  }

  snprintf(m + copied, SCRATCH_SIZE - copied, "\r\nProgram (read from eeprom): %s\r\n", progname_eeprom);

  return pdFALSE;
}

extern struct MonitorI2CTaskArgs_t clock_args;
extern struct MonitorI2CTaskArgs_t clockr0a_args;


extern struct MonitorTaskArgs_t dcdc_args;
//extern struct dev_i2c_addr_t pm_addrs_dcdc[N_PM_ADDRS_DCDC];
//extern struct pm_command_t extra_cmds[N_EXTRA_CMDS]; // LocalTasks.c

// this command takes no arguments since there is only one command
// right now.
BaseType_t sensor_summary(int argc, char **argv, char *m)
{
  int copied = 0;
  // collect all sensor information
  // highest temperature for each
  // Firefly
  // FPGA
  // DCDC
  // TM4C
  float tm4c_temp = getADCvalue(ADC_INFO_TEMP_ENTRY);
  int tens, frac;
  float_to_ints(tm4c_temp, &tens, &frac);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "MCU %02d.%02d\r\n", tens, frac);
  // Fireflies. These are reported as ints but we are asked
  // to report a float.
  int8_t imax_temp = -99;
  for (int i = 0; i < NFIREFLIES; ++i) {
    #ifndef DEVBOARD
    int8_t v = getFFtemp(i);
    #else
    int8_t v = 0;
    #endif
    if (v > imax_temp)
      imax_temp = v;
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY %02d.0\r\n", imax_temp);
  // FPGAs.
  float max_fpga;
  /*
  if (fpga_args.n_devices == 2)
    max_fpga = MAX(fpga_args.pm_values[0], fpga_args.pm_values[1]);
  else
    max_fpga = fpga_args.pm_values[0];
    */
  float_to_ints(max_fpga, &tens, &frac);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FPGA %02d.%02d\r\n", tens, frac);

  // DCDC. The first command is READ_TEMPERATURE_1.
  // I am assuming it stays that way!!!!!!!!
  float max_temp = -99.0f;
  /*
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
    for (int page = 0; page < dcdc_args.n_pages; ++page) {
      float thistemp = dcdc_args.pm_values[ps * (dcdc_args.n_commands * dcdc_args.n_pages) +
                                           page * dcdc_args.n_commands + 0];
      if (thistemp > max_temp)
        max_temp = thistemp;
    }
  }
  */
  float_to_ints(max_temp, &tens, &frac);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "REG %02d.%02d\r\n", tens, frac);

  return pdFALSE;
}