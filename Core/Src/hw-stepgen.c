#include "hw-stepgen.h"

static int dir_x; // +1 or -1

void timer_pulse_x(void)
{
    g_pos_x += dir_x;
}


void update(void)
{
    // (repeat for y, z, a)

    // Leave if the velocity is unchanged

    // stop the timer, or wait until the timer finishes the pulse, then stop the timer

    // collect the current timer position

    // calculate the new period. For Zero velocity, the period is 64K, but the timer is not enabled

    // calculate the starting point. For Zero velocity, the starting point is % of 64K

    // check for a direction change?

    // set the timer settings

    // start the timer if the velocity is not zero

}
