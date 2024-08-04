#include "main.h"
#if 0
// Steps for the next timeframe
static int steps_future[AXES];

// Steps for current timeframe
static int steps_present[AXES];

// Current position (aka phase) in the current timeframe
int timeframe_position;

// Some constants...
enum
{
    MICROSECONDS_PER_TIMEFRAME = 1000,
    TICKS_PER_TIMEFRAME = 250,
    CLOCKTICKS_PER_KERNELTICK = 500,
    CLOCKTICKS_PER_SECOND = 160000000,
    KERNELTICKS_PER_SECOND = 320000,
    KERNELTICKS_PER_TIMEFRAME = 320,
    //KERNELTICKS_PER_TIMEFRAME_ACTUAL = 320-2, // approx 99%. To make sure time drift is negative, so that future pulses won't pile up on top of other future pulses
};

/**
 * @brief Requests pulses for the next timeframe. Also synchronizes timeframes
 *
 * Synchronization: timeframe_position should be at the 50% mark when this function is called.
 * Timeframe length will stretch or shrink slightly for this to happen
 *
 * @param buf array of pulses to request for each axis
 */
void stepper_request_pulses(const volatile int32_t *buf)
{
    __disable_irq();
    stepgen_input.velocity[0] = buf[0];
    stepgen_input.velocity[1] = buf[1];
    stepgen_input.velocity[2] = buf[2];
    stepgen_input.velocity[3] = buf[3];

        /* check for direction change */
        for (int i = 0; i < 4; i++) {
        if (!dirchange[i]) {
            if ((stepgen_input.velocity[i] ^ oldvel[i]) & DIR_MASK) {
                dirchange[i] = DIRECTION_SETUP_CYCLES;
                oldvel[i] = stepgen_input.velocity[i];
            }
        }
        }
    __enable_irq();
}

void end_of_timeframe(void)
{
    __disable_irq();
    memcpy( steps_present, steps_future, sizeof(steps_present));
    memset( steps_future, 0, sizeof(steps_future));
    timeframe_position = 0;
    __enable_irq();
}

void kernel_update(void)
{
    uint32_t stepready;
    int i;

    for (i = 0; i < AXES; i++)
    {
        /* check if a step pulse can be generated */
        stepready = (position[i] ^ oldpos[i]) & HALFSTEP_MASK;

        /* request a new step pulse. Do not request a new pulse if direction hasn't changed yet */
        if (stepready && !dirchange[i]) {
            oldpos[i] = position[i];
            stepwidth[i] =  step_width + 1;
            do_step_hi[i] = 0;
        }

        /* pulse progress */
        if (stepwidth[i]) {
            if (--stepwidth[i]) {
                step_hi(i);
            } else {
                do_step_hi[i] = 1;
                step_lo(i);
            }
        }

        /* update direction signal after step hi-lo transition */
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
#endif
