#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"

#include "pinsel_MPI.h"

#ifdef DEBUG
extern void __error__(char *pcFilename, uint32_t ui32Line);
#endif // DEBUG


void pinsel(int pin, uint32_t *x_gpio_port, uint8_t *x_gpio_pin)
{
  uint8_t gpio_pin = -1;
  uint32_t gpio_port = -1;

  switch (pin) {
#define X(NAME, PPIN, PPORT, LOCALPIN, INPUT) \
  case NAME: {                                \
    gpio_port = GPIO_PORT##PPORT##_BASE;      \
    gpio_pin = GPIO_PIN_##LOCALPIN;           \
    break;                                    \
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
    default: {
      gpio_port = -1;
      gpio_pin = -1;
#ifdef DEBUG
      __error__(__FILE__, __LINE__);
#endif // DEBUG
      break;
    }
  }
  *x_gpio_port = gpio_port;
  *x_gpio_pin = gpio_pin;
}
