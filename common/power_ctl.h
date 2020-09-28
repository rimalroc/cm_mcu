/*
 * power_ctl.h
 *
 *  Created on: May 9, 2019
 *      Author: wittich
 */

#ifndef COMMON_POWER_CTL_H_
#define COMMON_POWER_CTL_H_

#include <stdint.h>
#include <stdbool.h>

// these are all LED messages
#define PS_GOOD    (1)
#define PS_BAD     (2)
#define PS_ON      (3)
#define PS_OFF     (4)
#define PS_ERROR   (5) // error generated by pwr_ctl
#define PS_STATUS  (6)
// alarms
#define TEMP_ALARM (7)
#define TEMP_ALARM_CLEAR (8)
#define CURRENT_ALARM (9)
#define CURRENT_ALARM_CLEAR (10)
#define PS_ANYFAIL_ALARM (11)
#define PS_ANYFAIL_ALARM_CLEAR (12)

// alarms

#define HUH             (99)

// power supply state
enum ps_state { PWR_UNKNOWN, PWR_ON, PWR_OFF, PWR_DISABLED, PWR_FAILED };
enum ps_state getPSStatus(int i);
void setPSStatus(int i, enum ps_state theState);
int getLowestEnabledPSPriority();

// Number of enable and power good/OK pins
#define N_PS_ENABLES 16
#define N_PS_OKS 14
// Masks for the ENABLE bits and the OK/PG (power good)
// bits, for the pins defined in the enables[]
// and oks[] arrays.
#define PS_OKS_MASK ((1U << N_PS_OKS) - 1)
#define PS_OKS_KU_MASK  0x0F03U
#define PS_OKS_VU_MASK  0x30CCU
#define PS_OKS_GEN_MASK 0x0030U
#define PS_ENS_MASK     ((1U << N_PS_ENABLES) - 1)
#define PS_ENS_GEN_MASK 0x000CU
#define PS_ENS_VU_MASK  0xC332U
#define PS_ENS_KU_MASK  0x3CC1U

// OK masks for various stages of the turn-on.
// these are indices into the oks[] array 
// L1-L5, note NO L3!!! no PG on the L3 supplies
#define PS_OKS_KU_MASK_L1 0x0003U
#define PS_OKS_KU_MASK_L2 0x0030U // these are common to VU and KU
#define PS_OKS_KU_MASK_L4 0x0300U
#define PS_OKS_KU_MASK_L5 0x0C00U
#define PS_OKS_VU_MASK_L1 0x000CU
#define PS_OKS_VU_MASK_L2 PS_OKS_KU_MASK_L2
#define PS_OKS_VU_MASK_L4 0x00C0U
#define PS_OKS_VU_MASK_L5 0x3000U

bool turn_on_ps(uint16_t);
bool check_ps(void);
bool disable_ps(void);
void turn_on_ps_at_prio(bool vu_enable, bool ku_enable, int prio);
void blade_power_ok(bool isok);


#endif /* COMMON_POWER_CTL_H_ */
