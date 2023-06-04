#pragma once

double g_vel_x;
double g_vel_y;
// z
// a

int32_t g_pos_x;
int32_t g_pos_y;
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

