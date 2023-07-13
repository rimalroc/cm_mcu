/*
 * BoardCommands.c
 *
 *  Created on: Jan 18, 2021
 *      Author: fatimayousuf
 */
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdlib.h>
#include "FreeRTOSConfig.h"
#include "commands/parameters.h"
#include "common/utils.h"
#include "driverlib/gpio.h"
#include "BoardCommands.h"
#include "common/pinsel.h"
#include "inc/hw_hibernate.h"
#include "driverlib/hibernate.h"
#include "Tasks.h"

// This command takes no arguments
BaseType_t restart_mcu(int argc, char **argv, char *m)
{
  snprintf(m, SCRATCH_SIZE, "Restarting MCU\r\n");
  MAP_SysCtlReset(); // This function does not return
  __builtin_unreachable();
  return pdFALSE;
}
/*
// Takes 3 arguments
BaseType_t set_board_id(int argc, char **argv, char *m)
{
  int copied = 0;

  uint64_t pass, addr, data;
  pass = strtoul(argv[1], NULL, 16);
  addr = strtoul(argv[2], NULL, 16);
  data = strtoul(argv[3], NULL, 16);
  uint64_t block = EEPROMBlockFromAddr(addr);
  if (block != 1) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Please input address in Block 1\r\n");
    return pdFALSE;
  }

  if (pass != 0x12345678) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "Wrong password. Type eeprom_info to get password.");
  } // data not printing correctly?
  else {
    uint64_t unlock = EPRMMessage((uint64_t)EPRM_UNLOCK_BLOCK, block, pass);
    xQueueSendToBack(xEPRMQueue_in, &unlock, portMAX_DELAY);

    uint64_t message = EPRMMessage((uint64_t)EPRM_WRITE_SINGLE, addr, data);
    xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

    uint64_t lock = EPRMMessage((uint64_t)EPRM_LOCK_BLOCK, block << 32, 0);
    xQueueSendToBack(xEPRMQueue_in, &lock, portMAX_DELAY);

    if (addr == ADDR_FF) {
      ff_USER_mask = data;
    }
  }

  return pdFALSE;
}

// one-time use, has one function and takes 0 arguments
BaseType_t set_board_id_password(int argc, char **argv, char *m)
{
  int copied = 0;

  uint64_t message = EPRMMessage((uint64_t)EPRM_PASS_SET, 0, PASS);
  xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Block locked\r\n");

  return pdFALSE;
}

BaseType_t board_id_info(int argc, char **argv, char *m)
{
  int copied = 0;

  uint32_t sn = read_eeprom_single(EEPROM_ID_SN_ADDR);
  uint32_t ps = read_eeprom_single(EEPROM_ID_PS_IGNORE_MASK);

  uint32_t num = (uint32_t)sn >> 16;
  uint32_t rev = ((uint32_t)sn) & 0xff;

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "ID:%08lx\r\n", sn);

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Board number: %lu\r\n", num);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Revision: %lu\r\n", rev);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Firefly USER config: %lx\r\n", ff_USER_mask);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Firefly PRESENT config: %lx\r\n", ff_PRESENT_mask);
  // copied += // this is here to remind you to update `copied` if you add more lines
  snprintf(m + copied, SCRATCH_SIZE - copied, "PS ignore mask: %lx\r\n", ps);

  return pdFALSE;
}
*/

BaseType_t gpio_ctl(int argc, char **argv, char *m)
{
  int port = -1;
  int pin = -1;
  // should either have three or four arguments
  if (!(argc == 3 || argc == 4)) {
    snprintf(m, SCRATCH_SIZE, "%s: usage: %s (set|get) <name of pin> <val>\r\n", argv[0], argv[0]);
    return pdFALSE;
  }
  /// X-Macro start
  // find the corresponding pins and ports
#define X(NAME, PPIN, PPORT, LOCALPIN, INPUT)        \
  if (strncmp(#NAME, argv[2], strlen(#NAME)) == 0) { \
    port = GPIO_PORT##PPORT##_BASE;                  \
    pin = GPIO_PIN_##LOCALPIN;                       \
  }
#ifdef DEVBOARD
#include "common/gpio_pins_devboard.def"
#elif defined(DEMO)
#warning "pins for Demo havne't been defined"
#elif defined(PROTO)
#warning "pins for Demo havne't been defined"
#else
#error "need to define either DEVBOARD, DEMO or PROTO"
#endif
  // X-Macro end
  // ensure we found a match
  if (pin == -1 || port == -1) {
    snprintf(m, SCRATCH_SIZE, "%s: couldn't find pin %s\r\n", argv[0], argv[2]);
    return pdFALSE;
  }
  if (argc == 4) {
    if (strncmp(argv[1], "set", 3) == 0) {
      // Check GPIOPinWrite docs for why this is so weird
      int pinval = atoi(argv[3]);
      if (pinval == 1)
        pinval = pin;
      else if (pinval != 0) {
        snprintf(m, SCRATCH_SIZE, "%s: value %s not understood\r\n", argv[0], argv[3]);
        return pdFALSE;
      }
      MAP_GPIOPinWrite(port, pin, pinval);
      snprintf(m, SCRATCH_SIZE, "%s: set %s to %s\r\n", argv[0], argv[2], argv[3]);
      return pdFALSE;
    }
    else {
      snprintf(m, SCRATCH_SIZE, "%s: command %s not understood\r\n", argv[0], argv[1]);
      return pdFALSE;
    }
  }
  else if (argc == 3) {
    if (strncmp(argv[1], "get", 3) == 0) {
      // see comments on the "set" command above
      uint32_t val = MAP_GPIOPinRead(port, pin);
      if (val == pin)
        val = 1;
      else
        val = 0;

      snprintf(m, SCRATCH_SIZE, "%s: pin %s reads %lu\r\n", argv[0], argv[2], val);
      return pdFALSE;
    }
    else {
      snprintf(m, SCRATCH_SIZE, "%s: command %s not understood\r\n", argv[0], argv[1]);
      return pdFALSE;
    }
  }
  m[0] = '\0';
  return pdFALSE;
}

#ifdef REV2
// interface to 3.8 V
// interface: v38 on|off 1|2
BaseType_t v38_ctl(int argc, char **argv, char *m)
{
  // argument handling
  int copied = 0;
  bool turnOn = true;
  if (strncmp(argv[1], "off", 3) == 0)
    turnOn = false;
  BaseType_t whichFF = atoi(argv[2]);
  if (whichFF < 1 || whichFF > 2) {
    snprintf(m, SCRATCH_SIZE, "%s: should be 1 or 2 for F1/F2 (got %s)\r\n", argv[0], argv[2]);
    return pdFALSE;
  }
  UBaseType_t ffmask[2] = {0, 0};
  ffmask[whichFF - 1] = 0xe;
  int ret = enable_3v8(ffmask, !turnOn);
  if (ret != 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "enable 3v8 failed with %d\r\n", ret);
    if (ret == 5)
      snprintf(m + copied, SCRATCH_SIZE - copied, "please release semaphore \r\n");
    return pdFALSE;
  }
  snprintf(m + copied, SCRATCH_SIZE - copied, "%s: 3V8 turned %s for F%ld\r\n", argv[0],
           turnOn == true ? "on" : "off", whichFF);
  return pdFALSE;
}
#endif // REV2
