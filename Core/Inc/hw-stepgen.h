#pragma once

#include <stdint.h>

// Pulses per second (I think? We'll see when I get linuxCNC traffic)
extern double g_vel_x;
extern double g_vel_y;
// z
// a

extern int32_t g_pos_x;
extern int32_t g_pos_y;
// z,a

/**
 * @brief Called every time joint wants to set a new velocity. New velocity commands are collected independenty?
 *
 */
void update(void);

/**
 * @brief Called every time a pulse is sent to X. Important for internal tracking
 *
 */
void timer_pulse_x(void);

// duplicates for y, z

