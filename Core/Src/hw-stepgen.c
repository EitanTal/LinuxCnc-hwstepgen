#include "hw-stepgen.h"
#include "main.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;


static int dir_x = 1; // +1 or -1

int32_t g_pos_x = 0;
int32_t g_pos_y = 0;
double g_vel_x;
double g_vel_y;

#define ZERO_VELOCITY -1
#define CLK_FREQUENCY_HZ 64000000
#define CLK_FREQUENCY_MHZ 64
#define PULSE_LEN_US      3

//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3) timer_pulse_x();
}

void timer_pulse_x(void)
{
    g_pos_x += dir_x;
}

#define CEIL_DIV(a,b)  ((a+b-1.0) / b)

void update(void)
{
    // (repeat for y, z, a)

    // Leave if the velocity is unchanged
    static double old_vel_x = 0.0;
    if (old_vel_x == g_vel_x) return;
    old_vel_x = g_vel_x;

    // stop the timer, or wait until the timer finishes the pulse, then stop the timer
    //HAL_TIM_Base_Stop(&htim3);
    htim3.Instance->CR1 &= ~(TIM_CR1_CEN);
    while (htim3.Instance->CNT < htim3.Instance->CCR4) // maybe do a one-pulse mode?
    {
        //HAL_TIM_Base_Start(&htim3);
        htim3.Instance->CR1 &= ~(TIM_CR1_CEN);
        // ...
        //HAL_TIM_Base_Stop(&htim3);
        htim3.Instance->CR1 |= (TIM_CR1_CEN);
    }

    // collect the current timer position
    float pos = htim3.Instance->CNT / (float)htim3.Instance->ARR;

    // calculate the new period. For Zero velocity, the period is 64K, but the timer is not enabled
    // for slow velocity, increase the prescaler, even to the point of a longer pulse. Cap extremely slow velocities, else the pulse gets too long making it difficult to wait until it ends
    float ticks_per_pulse = ZERO_VELOCITY;
    if (fabs(g_vel_x) > 0.1)
    {
        static const float ticks_per_second = CLK_FREQUENCY_HZ;
        ticks_per_pulse = ticks_per_second / fabs(g_vel_x); // ! Consider rounding?
    }
    int prescaler = 0;
    if (ticks_per_pulse >= 0xFFFF) // ! actual cycle length. ARR register takes in that, minus one.
    {
        prescaler = ticks_per_pulse / 0xFFFF;
        ticks_per_pulse = ticks_per_pulse / (prescaler+1); // ! Consider rounding?
    }
    if (ticks_per_pulse > ZERO_VELOCITY)
    {
        htim3.Instance->ARR = ticks_per_pulse -1; // ! watch out for ticks_per_pulse == zero
        htim3.Instance->PSC = prescaler; // ! oops, changes to prescaler only apply after the next cycle
    }
    int pulse_len_ticks = CEIL_DIV(CLK_FREQUENCY_MHZ * PULSE_LEN_US, (prescaler+1)); // ! ceiling div? Watch out for zero length
    htim3.Instance->CCR4 = pulse_len_ticks;

    // calculate the starting point. For Zero velocity, the starting point is % of 64K (Or leave unchanged?)
    if (ticks_per_pulse > ZERO_VELOCITY)
    {
        int new_cnt = pos * ticks_per_pulse;
        if (new_cnt <= pulse_len_ticks) // can't start on a pulse.
        {
            new_cnt = pulse_len_ticks+1;
        }
        htim3.Instance->CNT = new_cnt; // ! TODO: Check & compensate for function latency?
    }

    // check for a direction change. Delay if needed
    static int old_direction = GPIO_PIN_RESET;
    GPIO_PinState new_direction = (g_vel_x >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    if (new_direction != old_direction)
    {
        HAL_GPIO_WritePin(XD_GPIO_Port, XD_Pin, new_direction);
        old_direction = new_direction;
        dir_x = (g_vel_x >= 0) ? +1 : -1;
        // ! TODO implement delay. via CNT, maybe?
    }

    // set the timer settings (No need, we set them already)

    // start the timer if the velocity is not zero
    if (ticks_per_pulse > ZERO_VELOCITY)
    {
        //HAL_TIM_Base_Start(&htim3);
      htim3.Instance->CR1 |= (TIM_CR1_CEN);
    }
}
