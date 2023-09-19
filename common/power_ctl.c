/*
 * power_ctl.c
 *
 *  Created on: May 9, 2019
 *      Author: wittich
 */

#include <string.h>

#include "common/power_ctl.h"
#include "common/pinsel.h"
#include "common/utils.h"

#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif // USE_FREERTOS

// clang-format off
#if defined(REV1) 
// ------------------------------------------
//
// REV 1 
//
// ------------------------------------------
// if you update this you need to update N_PS_ENABLES
static const struct gpio_pin_t enables[] = {
    {  CTRL_F1_VCCINT_PWR_EN, "CTRL_F1_VCCINT_PWR_EN", 1},
    {  CTRL_F2_VCCINT_PWR_EN, "CTRL_F2_VCCINT_PWR_EN", 1},
    {  CTRL_VCC_1V8_PWR_EN, "CTRL_VCC_1V8_PWR_EN", 2},
    {  CTRL_VCC_3V3_PWR_EN, "CTRL_VCC_3V3_PWR_EN", 2},
    {  CTRL_F2_MGTY1_VCCAUX_PWR_EN, "CTRL_F2_MGTY1_VCCAUX_PWR_EN", 3},
    {  CTRL_F2_MGTY2_VCCAUX_PWR_EN, "CTRL_F2_MGTY2_VCCAUX_PWR_EN", 3},
    {  CTRL_F1_MGTY_VCCAUX_PWR_EN, "CTRL_F1_MGTY_VCCAUX_PWR_EN", 3},
    {  CTRL_F1_MGTH_VCCAUX_PWR_EN, "CTRL_F1_MGTH_VCCAUX_PWR_EN", 3},
    {  CTRL_F2_MGTY1_AVCC_PWR_EN, "CTRL_F2_MGTY1_AVCC_PWR_EN", 4},
    {  CTRL_F2_MGTY2_AVCC_PWR_EN, "CTRL_F2_MGTY2_AVCC_PWR_EN", 4},
    {  CTRL_F1_MGTY_AVCC_PWR_EN, "CTRL_F1_MGTY_AVCC_PWR_EN", 4},
    {  CTRL_F1_MGTH_AVCC_PWR_EN, "CTRL_F1_MGTH_AVCC_PWR_EN", 4},  
    {  CTRL_F1_MGTY_AVTT_PWR_EN, "CTRL_F1_MGTY_AVTT_PWR_EN",  5},
    {  CTRL_F1_MGTH_AVTT_PWR_EN, "CTRL_F1_MGTH_AVTT_PWR_EN", 5},
    {  CTRL_F2_MGTY1_AVTT_PWR_EN, "CTRL_F2_MGTY1_AVTT_PWR_EN", 5},
    {  CTRL_F2_MGTY2_AVTT_PWR_EN, "CTRL_F2_MGTY2_AVTT_PWR_EN", 5}
};

//if you update this you need to update N_PS_OKS too
// Notice that the VCCAUX is not included here; the
// TPS5218 supply does not have any such output
const
struct gpio_pin_t oks[] = {
    { F1_VCCINT_PG_A, "F1_VCCINT_PG_A", 1},
    { F1_VCCINT_PG_B, "F1_VCCINT_PG_B", 1},
    { F2_VCCINT_PG_A, "F2_VCCINT_PG_A", 1},
    { F2_VCCINT_PG_B, "F2_VCCINT_PG_B", 1},
    { VCC_1V8_PG, "VCC_1V8_PG",   2},
    { VCC_3V3_PG, "VCC_3V3_PG",  2},
    { F2_MGTY1_AVCC_OK, "F2_MGTY1_AVCC_OK", 4},
    { F2_MGTY2_AVCC_OK, "F2_MGTY2_AVCC_OK", 4},ignore_mask
    { F1_MGTY_AVCC_OK,  "F1_MGTY_AVCC_OK", 4},
    { F1_MGTH_AVCC_OK, "F1_MGTH_AVCC_OK", 4},
    { F1_MGTY_AVTT_OK, "F1_MGTY_AVTT_OK", 5},
    { F1_MGTH_AVTT_OK,  "F1_MGTH_AVTT_OK", 5},
    { F2_MGTY1_AVTT_OK, "F2_MGTY1_AVTT_OK", 5},
    { F2_MGTY2_AVTT_OK, "F2_MGTY2_AVTT_OK", 5}
};
#elif defined(REV2) // REV2
// ------------------------------------------
//
// REV 2
//
// ------------------------------------------
// if you update this you need to update N_PS_ENABLES
static const struct gpio_pin_t enables[] = {
    {  EN_F1_INT, "EN_F1_INT", 1},
    {  EN_F2_INT, "EN_F2_INT", 1},
    {  EN_1V8, "EN_1V8", 2},
    {  EN_3V3, "EN_3V3" ,2},
    {  EN_F1_VCCAUX, "EN_F1_VCCAUX", 3},
    {  EN_F2_VCCAUX,  "EN_F2_VCCAUX", 3},
    {  EN_F1_AVCC, "EN_F1_AVCC", 4},
    {  EN_F2_AVCC, "EN_F2_AVCC", 4},
    {  EN_F1_AVTT, "EN_F1_AVTT", 5},
    {  EN_F2_AVTT, "EN_F2_AVTT", 5},
};

// if you update this you need to update N_PS_OKS too
// note we do _not_ include the PG for the 4V0 supply, though it exists.
// this is because the supply is turned on automatically at L2
// but only enabled for the fireflies at L6. At L6 we don't actually 
// turn on the supplies, but instead enable them for the fireflies.
const
struct gpio_pin_t oks[N_PS_OKS] = {
    { PG_F1_INT_A, "PG_F1_INT_A", 1},
    { PG_F1_INT_B, "PG_F1_INT_B", 1},
    { PG_F2_INT_A, "PG_F2_INT_A", 1},
    { PG_F2_INT_B, "PG_F2_INT_B", 1},
    { PG_1V8, "PG_1V8",   2},
    { PG_3V3, "PG_3V3",   2},
    { PG_F1_VCCAUX, "PG_F1_VCCAUX", 3},
    { PG_F2_VCCAUX, "PG_F2_VCCAUX", 3},
    { PG_F1_AVCC, "PG_F1_AVCC", 4},
    { PG_F2_AVCC, "PG_F2_AVCC", 4},
    { PG_F1_AVTT,  "PG_F1_AVTT", 5},
    { PG_F2_AVTT, "PG_F2_AVTT", 5},
    //{ PG_4V0, "PG_4V0", 6},  // enable_3v8(true/false) won't change PG_4V0. Only within 10s after 4.0V off, PG_4V0 can be 0x0.
};
#elif defined(DEVBOARD)

static const struct gpio_pin_t enables[] = {
    { BLADE_POWER_OK, "BLADE_POWER_EN", 1},
    { CLK_PM_CTRL0, "CLK_PM_CTRL0", 1},

    { KUP_CORE_RUN,     "KUP_CORE_RUN", 2},
    { KUP_DDR4_TERM_EN, "KUP_DDR4_TERM_EN", 2},
    { KUP_PM_CTRL0,     "KUP_PM_CTRL0", 2},
    { KUP_P3V3_IO_RUN,  "KUP_P3V3_IO_RUN", 2},

    { ZUP_CORE_RUN, "ZUP_CORE_RUN", 3},
    { ZUP_PS_DDR4_TERM_EN, "ZUP_PS_DDR4_TERM_EN", 3},
    { ZUP_PL_DDR4_TERM_EN, "ZUP_PL_DDR4_TERM_EN", 3},
    { ZUP_PM_CTRL0, "ZUP_PM_CTRL0", 3},

    { FIREFY_P3V3_RUN, "FIREFY_P3V3_RUN", 4},
    
};
// oks should be a Power Good indicator
// for the moment just check the status of the ctrl pin
const
struct gpio_pin_t oks[N_PS_OKS] = {
    { CLK_PM_CTRL0, "CLK_PM_CTRL0", 1},

    { KUP_CORE_RUN,     "KUP_CORE_RUN", 2},
    { KUP_DDR4_TERM_EN, "KUP_DDR4_TERM_EN", 2},
    { KUP_PM_CTRL0,     "KUP_PM_CTRL0", 2},
    { KUP_P3V3_IO_RUN,  "KUP_P3V3_IO_RUN", 2},

    { ZUP_CORE_RUN, "ZUP_CORE_RUN", 3},
    { ZUP_PS_DDR4_TERM_EN, "ZUP_PS_DDR4_TERM_EN", 3},
    { ZUP_PL_DDR4_TERM_EN, "ZUP_PL_DDR4_TERM_EN", 3},
    { ZUP_PM_CTRL0, "ZUP_PM_CTRL0", 3},

    { FIREFY_P3V3_RUN, "FIREFY_P3V3_RUN", 4},
};
#elif defined(DEMO)
#warning "pins for Demo havne't been defined"
#elif defined(PROTO)
#warning "pins for Demo havne't been defined"
#else
#error "Unknown board revision"
#endif // REV2

// clang-format on
#define PS_NUM_PRIORITIES 6

// these arrays hold the current and old status of these power supplies
static enum ps_state states[N_PS_OKS] = {PWR_UNKNOWN};
enum ps_state getPSStatus(int i)
{
  if (i < 0 || i >= N_PS_OKS)
    return PWR_UNKNOWN;
  return states[i];
}

void setPSStatus(int i, enum ps_state theState)
{
  if (i < 0 || i >= N_PS_OKS)
    return;
  states[i] = theState;
}

// turn off all power supplies in the proper order
// de-assert BLADE_POWER_OK on successful exit.
bool disable_ps(void)
{
  // first set the supplies to off to tell the
  // other tasks to prepare
  for (int o = 0; o < N_PS_OKS; ++o)
    if (states[o] != PWR_DISABLED)
      states[o] = PWR_OFF;
  // this long delay (probably too long) allows all I2C tasks
  // to finish their activity. For Rev2 of the CM (when the I2C
  // pullups are from management power) this delay can be reduced or
  // removed.
  vTaskDelay(pdMS_TO_TICKS(500));

  // disable in reverse order
  for (int prio = PS_NUM_PRIORITIES; prio > 0; --prio) {
    // disable the supplies at the relevant priority and reverse order
    for (int e = N_PS_ENABLES; e > 0 ; --e) {
      if (enables[e].priority == prio) {
        write_gpio_pin(enables[e].pin_number, 0x0);
      }
    } // loop over enables
    bool ready_to_proceed = false;
    while (!ready_to_proceed) {
      bool all_ready = true;
      for (int o = 0; o < N_PS_OKS; ++o) {
        if (oks[o].priority >= prio) {
          uint8_t val = read_gpio_pin(oks[o].pin_number);
          if (val == 1) { // all supplies are supposed to be off now
            all_ready = false;
            states[o] = PWR_UNKNOWN;
          }
        }
      } // loop over 'ok' bits
      if (all_ready)
        ready_to_proceed = true;
    }
    // lowest_enabled_ps_prio = prio;
  } // loop over priorities

  // turn off POWER_OK when we are done
  write_gpio_pin(BLADE_POWER_OK, 0x0);

  return true;
}

// check the power supplies and turn them on one by one
// Assert BLADE_POWER_OK if you are successful.
// Return immediately if BLADE_POWER_EN is not asserted by the SM.
// Which supplies to enable is based on the ps_en_mask (passed in),
// which references entries in the enables[] array.
// the order to power on is defined by priority column in enables
//   start with priotity 1 and end with priority PS_NUM_PRIORITIES
bool turn_on_ps(uint16_t ps_en_mask)
{
  #ifndef DEVBOARD
  // if blade_power_en is false, return with failure
  bool blade_power_en = (read_gpio_pin(BLADE_POWER_EN) == 1);
  if (!blade_power_en) {
    write_gpio_pin(BLADE_POWER_OK, 0x0);
    return false;
  }
  #endif
  // loop over the enables
  for (int prio = 1; prio <= PS_NUM_PRIORITIES; ++prio) {
    // enable the supplies at the relevant priority
    for (int e = 0; e < N_PS_ENABLES; ++e) {
      if (enables[e].priority == prio) {
        // check if this supply is to be enabled
        if (((1U << e) & ps_en_mask) == 0) // not in mask
          continue;
        write_gpio_pin(enables[e].pin_number, 0x1);
      }
    }
  }

  write_gpio_pin(BLADE_POWER_OK, 0x1);
  return true;
}

// Enable supply at some priority. Also send in vu and ku enable.
void turn_on_ps_at_prio(int prio)
{
  // loop over the enables
  for (int e = 0; e < N_PS_ENABLES; ++e) {
    // if this enable matches the requested priority
    if (enables[e].priority == prio) {
      // no checks to be done
      write_gpio_pin(enables[e].pin_number, 0x1);
    }
  }
}

void blade_power_ok(bool isok)
{
  uint8_t val = (isok == true) ? 0x1U : 0x0U;
  write_gpio_pin(BLADE_POWER_OK, val);
}
