/*
 * CommandLineTask.c
 *
 *  Created on: Apr 7, 2019
 *      Author: wittich, rzou
 */

// Include commands
#include <strings.h>
#include "commands/BoardCommands.h"
#include "commands/BufferCommands.h"
#include "commands/EEPROMCommands.h"
#include "commands/I2CCommands.h"
#include "commands/SensorControl.h"
#include "commands/SoftwareCommands.h"
#include "common/smbus_units.h"
#include "common/printf.h"
#include "common/log.h"
#include "Semaphore.h"

static char m[SCRATCH_SIZE];

// this command takes no arguments and never returns.
static BaseType_t bl_ctl(int argc, char **argv, char *m)
{
  Print("Jumping to boot loader.\r\n");
  ROM_SysCtlDelay(100000);
  // this code is copied from the JumpToBootLoader()
  // stack from the boot_demo1 application in the
  // ek-tm4c129exl part of tiva ware.
  //
  // We must make sure we turn off SysTick and its interrupt before entering
  // the boot loader!
  //
  ROM_SysTickIntDisable();
  ROM_SysTickDisable();

  //
  // Disable all processor interrupts.  Instead of disabling them
  // one at a time, a direct write to NVIC is done to disable all
  // peripheral interrupts.
  //
  HWREG(NVIC_DIS0) = 0xffffffff;
  HWREG(NVIC_DIS1) = 0xffffffff;
  HWREG(NVIC_DIS2) = 0xffffffff;
  HWREG(NVIC_DIS3) = 0xffffffff;

  //
  // Return control to the boot loader.  This is a call to the SVC
  // handler in the boot loader.
  //
  (*((void (*)(void))(*(uint32_t *)0x2c)))();

  // the above points to a memory location in flash.
  // shut up compiler warning. This will never get called
  return pdFALSE;
}

// this command takes one argument
static BaseType_t clock_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  int status = -1; // shut up clang compiler warning
  BaseType_t i = strtol(argv[1], NULL, 10);
  if (!((i == 1) || (i == 2))) {
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied,
                 "Invalid mode %ld for clock, only 1 (reset) and 2 (program) supported\r\n", i);
    return pdFALSE;
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s mode set to %ld. \r\n", argv[0], i);
  if (i == 1) {
    status = initialize_clock();
    if (status == 0)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "clock synthesizer successfully initialized. \r\n");
  }
  else if (i == 2) {
    status = load_clock();
    if (status == 0)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "clock synthesizer successfully programmed. \r\n");
  }
  if (status == -1)
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s operation failed (1). \r\n", argv[0]);
  else if (status == -2)
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s operation failed (2). \r\n", argv[0]);
  else if (status != 0)
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s invalid return value. \r\n", argv[0]);
  return pdFALSE;
}

#ifdef REV2
// this command takes one argument (from triplet version but will take two argument to include an input from config versions for octlet eeprom)
static BaseType_t init_load_clock_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  char *clk_ids[5] = {"r0a", "r0b", "r1a", "r1b", "r1c"};
  BaseType_t i = strtol(argv[1], NULL, 10);
  if (i < 0 || i > 4) {
    snprintf(m + copied, SCRATCH_SIZE - copied,
             "Invalid clock chip %ld , the clock id options are r0a:0, r0b:1, r1a:2, "
             "r1b:3 and r1c:4 \r\n",
             i);
    return pdFALSE;
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s is programming clock %s. \r\n", argv[0], clk_ids[i]);
  int status = -1; // shut up clang compiler warning
  enum power_system_state power_state = getPowerControlState();
  if (power_state != POWER_ON) { // if the power state is not fully on
    snprintf(m + copied, SCRATCH_SIZE - copied, " 3V3 died. skip loadclock\r\n");
    return pdFALSE; // skip this iteration
  }
  // grab the semaphore to ensure unique access to I2C controller
  if (acquireI2CSemaphore(i2c2_sem) == pdFAIL) {
    snprintf(m + copied, SCRATCH_SIZE, "%s: could not get semaphore in time\r\n", argv[0]);
    return pdFALSE;
  }
  status = init_load_clk(i); // status is 0 if all registers can be written to a clock chip. otherwise, it implies that some write registers fail in a certain list.
  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c2_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c2_sem);
  }

  if (status == 0) {
    snprintf(m + copied, SCRATCH_SIZE - copied,
             "clock synthesizer with id %s successfully programmed. \r\n", clk_ids[i]);
  }
  else {
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s operation failed \r\n", argv[0]);
  }
  return pdFALSE;
}
#endif // REV2

typedef struct __attribute__((packed)) {
  linear11_val_t v_in;
  uint16_t v_out;
  linear11_val_t i_out;
  linear11_val_t i_out_max;
  linear11_val_t duty_cycle;
  linear11_val_t temperature;
  linear11_val_t unused1;
  linear11_val_t freq;
  uint8_t v_out_status;
  uint8_t i_out_status;
  uint8_t input_status;
  uint8_t temperature_status;
  uint8_t cml_status;
  uint8_t mfr_status;
  uint8_t flash_status;
  uint8_t unused[9];
} snapshot_t;

extern struct dev_i2c_addr_t pm_addrs_dcdc[];


static BaseType_t help_command_fcn(int argc, char **, char *m);

////////////////////////////////////////////////////////////////////////

static const char *const pcWelcomeMessage =
    "CLI based on microrl.\r\nType \"help\" to view a list of registered commands.\r\n";

struct command_t {
  const char *commandstr;
  BaseType_t (*interpreter)(int argc, char **, char *m);
  const char *helpstr;
  const int num_args;
};

#define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))
static struct command_t commands[] = {
    {"adc", adc_ctl, "Displays a table showing the state of ADC inputs.\r\n", 0},
//    {"alm", alarm_ctl, "args: (clear|status|settemp|setvoltthres|#)\r\nGet or clear status of alarm task.\r\n", -1},
#ifndef DEVBOARD
    {"bootloader", bl_ctl, "Call the boot loader\r\n", 0},
#endif
//    {"clkmon", clkmon_ctl, "Displays a table showing the clock chips' statuses given the clock chip id option\r\n", 1},
//    {"clock", clock_ctl,
//     "args: (1|2)\r\nReset (1) or program the clock synthesizer to 156.25 MHz (2).\r\n", 1},
//    {"fpga_reset", fpga_reset, "Reset Kintex (k) or Virtex (V) FPGA\r\n", 1},
/*
    {"ff", ff_ctl,
     "args: (xmit|cdr on/off (0-23|all)) | regw reg# val (0-23|all) | regr reg# (0-23)\r\n"
     " Firefly controlling and monitoring commands\r\n",
     -1},
    {
        "ff_status",
        ff_status,
        "Displays a table showing the status of the fireflies.\r\n",
        0,
    },
    {
        "ff_los",
        ff_los_alarm,
        "Displays a table showing the loss of signal alarms of the fireflies.\r\n",
        0,
    },
    {
        "ff_cdr_lol",
        ff_cdr_lol_alarm,
        "Displays a table showing the CDR loss of lock alarms of the fireflies.\r\n",
        0,
    },
    {
        "ff_temp",
        ff_temp,
        "Displays a table showing the temperature of the I2C fireflies.\r\n",
        0,
    },
    {"fpga", fpga_ctl, "Displays a table showing the state of FPGAs.\r\n",
     -1},
*/
    { "gpio",
      gpio_ctl,
      "Get or set any GPIO pin.\r\n",
      -1,},
    {"help", help_command_fcn, "This help command\r\n", -1},
//    {"id", board_id_info, "Prints board ID information.\r\n", 0},
//    {"i2cr", i2c_ctl_r,
//     "args: <dev> <address> <number of bytes>\r\nRead I2C controller. Addr in hex.\r\n", 3},
//    {"i2crr", i2c_ctl_reg_r,
//     "i2crr <dev> <address> <n reg addr bytes> <reg addr> <n data bytes> \r\n Read I2C controller. Addr in hex\r\n", 5},
//    {"i2cw", i2c_ctl_w, "i2cw <dev> <address> <number of bytes> <value>\r\n Write I2C controller.\r\n",
//     4},
//    {"i2cwr", i2c_ctl_reg_w,
//     "args: <dev> <address> <number of reg bytes> <reg> <number of bytes>\r\nWrite I2C controller.\r\n", 6},
//    { "i2c_scan",
//        i2c_scan,
//        "Scan current I2C bus.\r\n",
//        1},

//    {
//        "loadclock",
//        init_load_clock_ctl,
//        "args: the clock id options to program are r0a:0, r0b:1, r1a:2, r1b:3 and r1c:4.\r\n",
//        1},

//    {"led", led_ctl, "Manipulate red LED.\r\n", 1},
//    {"mem", mem_ctl, "Size of heap.\r\n", 0},
   { "pwr",
       power_ctl,
       "args: (on|off|status|clearfail)\r\nTurn on or off all power, get status or clear "
       "failures.\r\n",
       1 },
//    {"psmon", psmon_ctl, "Displays a table showing the state of power supplies.\r\n", 1},
    {"restart_mcu", restart_mcu, "Restart the microcontroller\r\n", 0},
//    {"semaphore", sem_ctl, "args: (none)|<i2cdev 1-6> <take|release>\r\nTake or release a semaphore\r\n", -1},

    {
        "simple_sensor",
        sensor_summary,
        "Displays a table showing the state of temps.\r\n",
        0,
    },
    {
        "stack_usage",
        stack_ctl,
        "Print out system stack high water mark.\r\n",
        0,
    },
    {
        "taskinfo",
        taskInfo,
        "Info about FreeRTOS tasks\r\n",
        0,
    },
    {"taskstats",
     TaskStatsCommand,
     "Displays a table showing the state of each FreeRTOS task\r\n", 0},

    {"uptime", uptime, "Display uptime in minutes\r\n", 0},
    {"version", ver_ctl, "Display information about MCU firmware version\r\n", 0},

    {"watchdog", watchdog_ctl, "Display status of the watchdog task\r\n", 0},


};
#ifdef DEVBOARD
static void U0Print(const char *str)
{
  UARTPrint(UART0_BASE, str);
}
#elif defined(REV1)
static void U4Print(const char *str)
{
  UARTPrint(UART4_BASE, str);
}
static void U1Print(const char *str)
{
  UARTPrint(UART1_BASE, str);
}
#elif defined(REV2) // REV1
static void U0Print(const char *str)
{
  UARTPrint(UART0_BASE, str);
}
#endif

struct microrl_user_data_t {
  uint32_t uart_base;
};

static BaseType_t help_command_fcn(int argc, char **argv, char *m)
{
  int copied = 0;
  if (argc == 1) {
    static int i = 0;
    for (; i < NUM_COMMANDS; ++i) {
      int left = SCRATCH_SIZE - copied;
      // need room for command string, help string, newlines, etc, and trailing \0
      unsigned int len = strlen(commands[i].helpstr) + strlen(commands[i].commandstr) + 7;
      if (left < len) {
        return pdTRUE;
      }
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s:\r\n %s",
                         commands[i].commandstr, commands[i].helpstr);
    }
    i = 0;
    return pdFALSE;
  }
  else { // help on a specific command.
    // help for any command that matches the entered command
    for (int j = 0; j < NUM_COMMANDS; ++j) {
      if (strncmp(commands[j].commandstr, argv[1], strlen(argv[1])) == 0) {
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s:\r\n %s",
                           commands[j].commandstr, commands[j].helpstr);
        // return pdFALSE;
      }
    }
  }
  if (copied == 0) {
    snprintf(m + copied, SCRATCH_SIZE - copied,
             "%s: No command starting with %s found\r\n", argv[0], argv[1]);
  }
  return pdFALSE;
}

static int execute(void *p, int argc, char **argv)
{
  struct microrl_user_data_t *userdata = p;
  uint32_t base = userdata->uart_base;

  UARTPrint(base, "\r\n"); // the microrl does not terminate the active command

  // find the command in the list
  // argc here includes the actual command itself, so the
  // number of supplied arguments is argc-1
  for (int i = 0; i < NUM_COMMANDS; ++i) {
    if (strncmp(commands[i].commandstr, argv[0], 256) == 0) {
      if ((argc == commands[i].num_args + 1) || commands[i].num_args < 0) {
        int retval = commands[i].interpreter(argc, argv, m);
        if (m[0] != '\0')
          UARTPrint(base, m);
        while (retval == pdTRUE) {
          retval = commands[i].interpreter(argc, argv, m);
          if (m[0] != '\0')
            UARTPrint(base, m);
        }
        m[0] = '\0';
        return 0;
      }
      else {
        snprintf(m, SCRATCH_SIZE,
                 "Wrong number of arguments for command %s: %d expected, got %d\r\n", argv[0],
                 commands[i].num_args, argc - 1);
        UARTPrint(base, m);
        return 0;
      }
    }
  }
  UARTPrint(base, "Command unknown: ");
  UARTPrint(base, argv[0]);
  UARTPrint(base, "\r\n");

  return 0;
}

// The actual task
void vCommandLineTask(void *pvParameters)
{
  uint8_t cRxedChar;

  configASSERT(pvParameters != 0);

  CommandLineTaskArgs_t *args = pvParameters;
  StreamBufferHandle_t uartStreamBuffer = args->UartStreamBuffer;
  uint32_t uart_base = args->uart_base;

  UARTPrint(uart_base, pcWelcomeMessage);
  struct microrl_user_data_t rl_userdata = {
      .uart_base = uart_base,
  };
#ifdef DEVBOARD
  void (*printer)(const char *) = U0Print;
#elif defined(REV1)
  void (*printer)(const char *) = U4Print;
#elif defined(REV2)
  void (*printer)(const char *) = U0Print;
#endif

  struct microrl_config rl_config = {
      .print = printer, // default to front panel
      // set callback for execute
      .execute = execute,
      .prompt_str = "% ",
      .prompt_length = 2,
      .userdata = &rl_userdata,
  };
#ifdef DEVBOARD
//nothing
#elif defined(REV1)
  // this is a hack
  if (uart_base == UART1_BASE) {
    rl_config.print = U1Print; // switch to Zynq
  }
#endif // REV1
  microrl_t rl;
  microrl_init(&rl, &rl_config);
  microrl_set_execute_callback(&rl, execute);
  microrl_insert_char(&rl, ' '); // this seems to be necessary?

  for (;;) {
    /* This implementation reads a single character at a time.  Wait in the
       Blocked state until a character is received. */
    xStreamBufferReceive(uartStreamBuffer, &cRxedChar, 1, portMAX_DELAY);
    microrl_insert_char(&rl, cRxedChar);

    // monitor stack usage for this task
    CHECK_TASK_STACK_USAGE(args->stack_size);
  }
}
