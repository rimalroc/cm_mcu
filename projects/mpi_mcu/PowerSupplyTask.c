/*
 * PowerSupplyTask.c
 *
 *  Created on: May 13, 2019
 *      Author: wittich
 */

// includes for types
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>

// local includes
#include "Tasks.h"
#include "common/i2c_reg.h"
#include "common/pinout.h"
#include "common/pinsel.h"
#include "common/power_ctl.h"
#include "common/utils.h"
#include "common/log.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"

void Print(const char *);

// Holds the handle of the created queue for the power supply task.
QueueHandle_t xPwrQueue = NULL;

enum power_system_state currentState = POWER_INIT; // start in POWER_INIT state

enum power_system_state getPowerControlState(void)
{
  return currentState;
}

extern const struct gpio_pin_t oks[];
extern int32_t N_PS_OKS_;
static uint16_t check_ps_oks(void)
{
  uint16_t status = 0U;
  for (int i = 0; i < N_PS_OKS_; ++i) {
    int val = read_gpio_pin(oks[i].pin_number);
    if (val)
      status |= 1U << i;
  }
  return status;
}
/*
static uint16_t getPSFailMask(void)
{
  // PS ignore mask stored in on-board EEPROM
  static bool configured = false;

  static uint32_t ps_ignore_mask;
  #if defined(DEVBOARD)
    ps_ignore_mask = 0x0000U;
  #elif defined (DEMO) || defined (PROTO)
    #error "getPSFailMask not defined for DEMO or PROTO"
  #else
    #error "getPSFailMask: need to defined a baord type"
  if (!configured) {
    ps_ignore_mask = read_eeprom_single(EEPROM_ID_PS_IGNORE_MASK);
    if (ps_ignore_mask & ~(PS_OKS_F1_MASK_L4 | PS_OKS_F1_MASK_L5 | PS_OKS_F2_MASK_L4 | PS_OKS_F2_MASK_L5)) {
      log_warn(LOG_PWRCTL, "Warning: mask 0x%x included masks at below L4; ignoring\r\n", ps_ignore_mask);
      // mask out supplies at startup L1, L2 or L3. We do not allow those to fail.
      ps_ignore_mask &= (PS_OKS_F1_MASK_L4 | PS_OKS_F1_MASK_L5 | PS_OKS_F2_MASK_L4 | PS_OKS_F2_MASK_L5);
    }
    configured = true;
  }
  #endif
  return (uint16_t)(0xFFFFU & ps_ignore_mask); // 16 bit
}
*/
void printfail(uint16_t failed_mask, uint16_t supply_ok_mask, uint16_t supply_bitset)
{
  log_error(LOG_PWRCTL, "psfail: fail, supply_mask, bitset =  %x,%x,%x\r\n", failed_mask,
            supply_ok_mask, supply_bitset);
}

static const char *const power_system_state_names[] = {
    "FAIL",
    "INIT",
    "DOWN",
    "OFF",
    "CLKON",
    "KUON",
    "ZUON",
    "VUON",
    "FFON",
    "ON",
};

const char *getPowerControlStateName(enum power_system_state s)
{
  return power_system_state_names[s];
}

// alarm from outside this task
static bool external_alarm = false;
const bool getPowerControlExternalAlarmState(void)
{
  return external_alarm;
}
// which power supply OKs to ignore
static uint16_t ignore_mask;
const uint16_t getPowerControlIgnoreMask(void)
{
  return ignore_mask;
}

// monitor and control the power supplies
void PowerSupplyTask(void *parameters)
{

  // compile-time sanity check

  // powerdown request from the CLI
  bool cli_powerdown_request = false;

  // masks to enable/check appropriate supplies

  // HACK
  // setting the enables both to true for debugging blank bo

  // exceptions are stored in the internal EEPROM -- the IGNORE mask.
//  ignore_mask = getPSFailMask();
  // doing nothing with ignored mask, might be helpful in future
//  if (ignore_mask) {  }


  uint16_t supply_ok_mask = PS_OKS_GEN_MASK;
  uint16_t supply_ok_mask_clock = 0U, supply_ok_mask_ku = 0U, 
          supply_ok_mask_zu = 0U, supply_ok_mask_vu = 0U,
          supply_ok_mask_ff = 0U;



  supply_ok_mask_clock = PS_OKS_MASK_CLOCK;
  #if defined(PROTO)
    supply_ok_mask |= PS_OKS_PROTO_MASK;
    supply_ok_mask_vu = supply_ok_mask_clock | PS_OKS_MASK_VU;
    supply_ok_mask_ff = supply_ok_mask_vu | PS_OKS_MASK_FF;
  #else
    supply_ok_mask |= PS_OKS_DEMO_MASK;
    supply_ok_mask_ku = supply_ok_mask_clock | PS_OKS_MASK_KU;
    supply_ok_mask_zu = supply_ok_mask_ku | PS_OKS_MASK_ZU;
    supply_ok_mask_ff = supply_ok_mask_zu | PS_OKS_MASK_FF;
  #endif

// Do we need to initiazile some power supply?

  bool power_supply_alarm = false;
  uint16_t failed_mask = 0x0U;

  // initialize to the current tick time *after the initialization*
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // this loop never exits
  for (;;) {
    // first check for message on the queue and collect all messages.
    // non-blocking call.
    uint32_t message;
    if (xQueueReceive(xPwrQueue, &message, 0)) { // TODO: what about > 1 message
      switch (message) {
        case PS_OFF:
          cli_powerdown_request = true;
          break;
        case TEMP_ALARM:
          external_alarm = true;
          break;
        case TEMP_ALARM_CLEAR:
          external_alarm = false;
          break;
        case PS_ON:
          cli_powerdown_request = false;
          break;
        case PS_ANYFAIL_ALARM_CLEAR:
          power_supply_alarm = false;
          failed_mask = 0x0U;
          break;
        default:
          break;
      }
    }
    #ifdef DEVBOARD
    bool ignorefail = true; // HACK THIS NEEDS TO BE FIXED TODO FIXME
    #else
    // the OKS needs to be read using the gpio expander.
    // need to decide if a task will read them, or will be read on demand.
    #error "need to review the sequence for DEMO and PROTO"
    bool ignorefail = false; // HACK THIS NEEDS TO BE FIXED TODO FIXME
    #endif
    // Check the state of BLADE_POWER_EN.
    bool blade_power_enable = (read_gpio_pin(BLADE_POWER_EN) == 1);

    // now check the actual state of the power supplies
    uint16_t supply_bitset = check_ps_oks();
    bool supply_off = false; // are supplies off (besides the ones that are disabled)
    if ((supply_bitset & supply_ok_mask) != supply_ok_mask) {
      supply_off = true;
    }

    // MAIN POWER SUPPLY TASK STATE MACHINE
  // might be good to define the state machine as a drawing

    enum power_system_state nextState;
    switch (currentState) {
      case POWER_INIT: {
        // only run on first boot
        nextState = POWER_OFF;
        break;
      }
      case POWER_ON: {
        if (supply_off && !ignorefail) {
          // log erroring supplies
          failed_mask = (~supply_bitset) & supply_ok_mask;
          printfail(failed_mask, supply_ok_mask, supply_bitset);
          errbuffer_power_fail(failed_mask);
          // turn off all supplies
          disable_ps();
          power_supply_alarm = true;
          nextState = POWER_FAILURE;
        }
        else if (external_alarm) {
          log_info(LOG_PWRCTL, "external alarm power down\r\n");
          errbuffer_put(EBUF_POWER_OFF_TEMP, 0);
          // turn off all supplies
          disable_ps();
          nextState = POWER_FAILURE;
        }
        else if (!blade_power_enable || cli_powerdown_request) {
          log_info(LOG_PWRCTL, "power-down requested\r\n");
          nextState = POWER_DOWN;
        }
        else {
          nextState = POWER_ON;
        }
        break;
      }
      case POWER_DOWN: {
        disable_ps();
        errbuffer_put(EBUF_POWER_OFF, 0);
        log_info(LOG_PWRCTL, "power-down completed\r\n");
        nextState = POWER_OFF;
        break;
      }
      case POWER_OFF: {
        // start power-on sequence
        if (blade_power_enable && !cli_powerdown_request && !external_alarm &&
            !power_supply_alarm) {
          log_info(LOG_PWRCTL, "power-up requested\r\n");
          turn_on_ps_at_prio(1);
          errbuffer_put(EBUF_POWER_ON, 0);
          nextState = POWER_CLOCK;
        }
        else {
          nextState = POWER_OFF;
        }
        break;
      }
      case POWER_CLOCK: {
        if (((supply_bitset & supply_ok_mask_clock) != supply_ok_mask_clock) && !ignorefail) {
          failed_mask = (~supply_bitset) & supply_ok_mask_clock;
          printfail(failed_mask, supply_ok_mask_clock, supply_bitset);
          errbuffer_power_fail(failed_mask);
          disable_ps();
          power_supply_alarm = true;
          nextState = POWER_FAILURE;
        }
        else {
          turn_on_ps_at_prio( 2);
          nextState = POWER_KU;
          #if defined(PROTO)
            nextState = POWER_VU;
          #endif
        }

        break;
      }
      case POWER_KU: {
        if (((supply_bitset & supply_ok_mask_ku) != supply_ok_mask_ku) && !ignorefail) {
          failed_mask = (~supply_bitset) & supply_ok_mask_ku;
          printfail(failed_mask, supply_ok_mask_ku, supply_bitset);
          errbuffer_power_fail(failed_mask);
          disable_ps();
          power_supply_alarm = true;
          nextState = POWER_FAILURE;
        }
        else {
          turn_on_ps_at_prio(3);
          nextState = POWER_ZU;
        }

        break;
      }
      case POWER_ZU: {
        if (((supply_bitset & supply_ok_mask_zu) != supply_ok_mask_zu) && !ignorefail) {
          failed_mask = (~supply_bitset) & supply_ok_mask_zu;
          printfail(failed_mask, supply_ok_mask_zu, supply_bitset);
          errbuffer_power_fail(failed_mask);
          disable_ps();
          power_supply_alarm = true;
          nextState = POWER_FAILURE;
        }
        else {
          turn_on_ps_at_prio(4);
          nextState = POWER_FIREFLY;
        }

        break;
      }
      case POWER_VU: {
        if (((supply_bitset & supply_ok_mask_vu) != supply_ok_mask_vu) && !ignorefail) {
          failed_mask = (~supply_bitset) & supply_ok_mask_vu;
          printfail(failed_mask, supply_ok_mask_vu, supply_bitset);
          errbuffer_power_fail(failed_mask);
          disable_ps();
          power_supply_alarm = true;
          nextState = POWER_FAILURE;
        }
        else {
          turn_on_ps_at_prio(4);
          nextState = POWER_FIREFLY;
        }

        break;
      }
      case POWER_FIREFLY: {
        if (((supply_bitset & supply_ok_mask_ff) != supply_ok_mask_ff) && !ignorefail) {
          failed_mask = (~supply_bitset) & supply_ok_mask_ff;
          printfail(failed_mask, supply_ok_mask_ff, supply_bitset);
          errbuffer_power_fail(failed_mask);
          disable_ps();
          power_supply_alarm = true;
          nextState = POWER_FAILURE;
        }
        else {
          blade_power_ok(true);
          nextState = POWER_ON;
        }

        break;
      }
      case POWER_FAILURE: {                           // we go through POWER_OFF state before turning on.
        if (!power_supply_alarm && !external_alarm) { // errors cleared
          nextState = POWER_OFF;
          errbuffer_power_fail_clear();
        }
        else { // still in failed state
          nextState = POWER_FAILURE;
        }
        break;
      }
      default: {
        // configASSERT(1 == 0);
        nextState = POWER_INIT; // shut up debugger
        break;
      }
    }

    // update the ps_state variables, for external display
    // as well as for usage in ensuring the I2C pullups are on.
    supply_bitset = check_ps_oks();
    for (int i = 0; i < N_PS_OKS_; i++) {
      if ((1U << i) & supply_bitset) {
        // OK bit is on -- PS is on
        setPSStatus(i, PWR_ON);
      }
      // OK bit is not on and ...
      else if (!((1U << i) & supply_ok_mask)) {
        // ... it should _not_ be on. Disabled intentionally
        setPSStatus(i, PWR_DISABLED);
      }
      else {
        // ... it _should_ be on based on the mask
        switch (currentState) {
          case POWER_OFF:
          case POWER_INIT:
            // ... but the power state is "off" or
            // we are just initializing / turning on
            setPSStatus(i, PWR_OFF);
            break;
          case POWER_ON:
          case POWER_FAILURE:
            // ... but the power state is is "on," so ...
            if ((1U << i) & failed_mask) {
              // ... either the supply failed
              setPSStatus(i, PWR_FAILED);
            }
            else {
              // ... or it's just off because there was a power failure,
              // but this supply is not the root cause.
              setPSStatus(i, PWR_OFF);
            }
            break;
          default:
            break;
        }
      }
    }
    if (currentState != nextState) {
      log_debug(LOG_PWRCTL, "%s: change from state %s to %s\r\n", pcTaskGetName(NULL),
                power_system_state_names[currentState], power_system_state_names[nextState]);
    }
    currentState = nextState;

    // monitor stack usage for this task
    static UBaseType_t vv = 4096;
    CHECK_TASK_STACK_USAGE(vv);

    // wait here for the x msec, where x is 2nd argument below.
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(25));
  }
}
