#ifndef PINSEL_MPI_H
#define PINSEL_MPI_H

#include <stdint.h>

// data structures to hold GPIO PIN information
struct gpio_pin_t {
  uint8_t pin_number;
  char *name;
  int priority;
};

#define X(name, pin, port, localpin, input) \
  name = pin,
enum pins {
#ifdef DEVBOARD
#include "common/gpio_pins_devboard.def"
#elif defined(DEMO)
#warning "pins for Demo havne't been defined"
#elif defined(PROTO)
#warning "pins for Demo havne't been defined"
#else
#error "need to define either DEVBOARD, DEMO or PROTO"
#endif
};

//#define isFPGAF1_PRESENT() (read_gpio_pin(_F1_INSTALLED) == 0)
//#define isFPGAF2_PRESENT() (read_gpio_pin(_F2_INSTALLED) == 0)



void pinsel(int pin, uint32_t *x_gpio_port, uint8_t *x_gpio_pin);

extern const char *const pin_names[];

#endif // PINSEL_MPI_H
