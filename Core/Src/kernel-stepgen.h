#pragma once

void stepgen_get_position(volatile void *buf);

void stepgen_update_input(const volatile void *buf);

void stepgen_update_stepwidth(int width);

void stepgen_reset(void);

void kernel_update(void);
