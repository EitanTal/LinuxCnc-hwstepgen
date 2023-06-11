#include "main.h"

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

static inline void step_hi(int val)
{

}

static inline void step_lo(int val)
{

}

static inline void dir_hi(int val)
{

}

static inline void dir_lo(int val)
{

}

static volatile int32_t position[AXES] = { 0 };

static int32_t oldpos[AXES] = { 0 },
               oldvel[AXES] = { 0 };

static int dirchange[AXES] = { 0 };
typedef struct {
	int32_t velocity[AXES];
} stepgen_input_struct;

static volatile stepgen_input_struct stepgen_input = { {0} };

static int do_step_hi[AXES] = { 1 };
static int stepwidth[AXES] = { 0 };

void stepgen_get_position(volatile void *buf)
{
    disable_int();
    memcpy(buf, (const void *)position, sizeof(position));
    enable_int();
}

void stepgen_update_input(const volatile  void *buf)
{
    disable_int();
    memcpy((void *)&stepgen_input, buf, sizeof(stepgen_input));
    enable_int();
}

static int step_width = STEPWIDTH_DEFAULT;

void stepgen_update_stepwidth(int width)
{
    step_width = width;
}

void stepgen_reset(void)
{
        int i;

        disable_int();

        for (i = 0; i < AXES; i++) {
                position[i] = 0;
                oldpos[i] = 0;
                oldvel[i] = 0;

                stepgen_input.velocity[i] = 0;
                do_step_hi[i] = 1;
        }

        enable_int();

        for (i = 0; i < AXES; i++) {
                step_lo(i);
                dir_lo(i);
        }
}

void kernel_update(void)
{
    TOGGLE_FAST();

    if (0)
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
}