#include "main.h"
#include <string.h>

#define TOGGLE_FAST() LED_GPIO_Port->ODR ^= LED_Pin

#define STEPWIDTH_DEFAULT 1
#define HALFSTEP_MASK (1<<22)
#define DIR_MASK (1<<31)

/*
  Timing diagram:

  STEPWIDTH   |<---->|
               ______           ______
  STEP	     _/      \_________/      \__
             ________                  __
  DIR	             \________________/

  Direction signal changes on the falling edge of the step pulse.

*/

GPIO_TypeDef* table_gpios_pulses[4]  = {XP_GPIO_Port, YP_ORIG_GPIO_Port, ZP_GPIO_Port, AP_ORIG_GPIO_Port};
uint32_t   table_pins_pulses_set[4]  = {XP_Pin,       YP_ORIG_Pin,       ZP_Pin,       AP_ORIG_Pin};
uint32_t table_pins_pulses_reset[4]  = {XP_Pin << 16, YP_ORIG_Pin << 16, ZP_Pin << 16, AP_ORIG_Pin << 16};

GPIO_TypeDef* table_gpios_dir[4]  = {XD_GPIO_Port, YD_GPIO_Port, ZD_GPIO_Port, AD_GPIO_Port};
uint32_t   table_pins_dir_set[4]  = {XD_Pin,       YD_Pin,       ZD_Pin,       AD_Pin};
uint32_t table_pins_dir_reset[4]  = {XD_Pin << 16, YD_Pin << 16, ZD_Pin << 16, AD_Pin << 16};


static inline void step_hi(int axis)
{
    table_gpios_pulses[axis]->BSRR = table_pins_pulses_set[axis];
}

static inline void step_lo(int axis)
{
    table_gpios_pulses[axis]->BSRR = table_pins_pulses_reset[axis];
}

static inline void dir_hi(int axis)
{
    table_gpios_dir[axis]->BSRR = table_pins_dir_set[axis];
}

static inline void dir_lo(int axis)
{
    table_gpios_dir[axis]->BSRR = table_pins_dir_reset[axis];
}

static int32_t position[AXES] = { 0 }; // not volatile?

static int32_t oldpos[AXES] = { 0 },
               oldvel[AXES] = { 0 };

static int dirchange[AXES] = { 0 };
typedef struct {
	int32_t velocity[AXES];
} stepgen_input_struct;

static stepgen_input_struct stepgen_input = { {0} }; // not volatile?

static int do_step_hi[AXES] = { 1 };
static int stepwidth[AXES] = { 0 };

void stepgen_get_position(volatile int32_t *buf)
{
    __disable_irq();
    buf[0] = position[0];
    buf[1] = position[1];
    buf[2] = position[2];
    buf[3] = position[3];
    __enable_irq();
}

void stepgen_update_input(const volatile int32_t *buf)
{
    __disable_irq();
    stepgen_input.velocity[0] = buf[0];
    stepgen_input.velocity[1] = buf[1];
    stepgen_input.velocity[2] = buf[2];
    stepgen_input.velocity[3] = buf[3];
    __enable_irq();
}

static int step_width = STEPWIDTH_DEFAULT;

void stepgen_update_stepwidth(int width)
{
    step_width = width;
}

void stepgen_reset(void)
{
    int i;

    __disable_irq();

    for (i = 0; i < AXES; i++) {
        position[i] = 0;
        oldpos[i] = 0;
        oldvel[i] = 0;

        stepgen_input.velocity[i] = 0;
        do_step_hi[i] = 1;
    }

    __enable_irq();

    for (i = 0; i < AXES; i++) {
        step_lo(i);
        dir_lo(i);
    }
}

void kernel_update(void)
{
    uint32_t stepready;
    int i;

    for (i = 0; i < AXES; i++)
    {
        /* check if a step pulse can be generated */
        stepready = (position[i] ^ oldpos[i]) & HALFSTEP_MASK;

        /* generate a step pulse */
        if (stepready) {
            oldpos[i] = position[i];
            stepwidth[i] =  step_width + 1;
            do_step_hi[i] = 0;
        }

        if (stepwidth[i]) {
            if (--stepwidth[i]) {
                step_hi(i);
            } else {
                do_step_hi[i] = 1;
                step_lo(i);
            }
        }

        /* check for direction change */
        if (!dirchange[i]) {
            if ((stepgen_input.velocity[i] ^ oldvel[i]) & DIR_MASK) {
                dirchange[i] = 1;
                oldvel[i] = stepgen_input.velocity[i];
            }
        }

        /* generate direction pulse after step hi-lo transition */
        if (do_step_hi[i] && dirchange[i]) {
            dirchange[i] = 0;
            if (oldvel[i] >= 0)
                dir_lo(i);
            if (oldvel[i] < 0)
                dir_hi(i);
        }

        /* update position counter */
        position[i] += stepgen_input.velocity[i];
    }
}
