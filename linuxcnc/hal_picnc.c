/*    Copyright (C) 2013 GP Orcullo
 *
 *    Portions of this code is based on stepgen.c
 *    by John Kasunich, Copyright (C) 2003-2007
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
//#include "bcm2835.h"

#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include "hal_picnc.h"

#define MODNAME "hal_picnc"
#define PREFIX "hal_picnc"

#define SPINDLES  1
#define SPINDLES_MAX  3
#define INPUTS    4
#define OUTPUTS   4

#define SPI_PINS_ARE_INPUTS 0

MODULE_AUTHOR("GP Orcullo");
MODULE_DESCRIPTION("Driver for Raspberry Pi PICnc board");
MODULE_LICENSE("GPL v2");

static int stepwidth = 1;
RTAPI_MP_INT(stepwidth, "Step width in 1/BASEFREQ");

static long pwmfreq = 1000;
RTAPI_MP_LONG(pwmfreq, "PWM frequency in Hz");

typedef struct {
	hal_float_t *position_cmd[NUMAXES],
		    *position_fb[NUMAXES],
		    *pwm_duty[SPINDLES],
		    *adc_in[SPINDLES];
	hal_bit_t   *out[OUTPUTS], *inp[INPUTS],
		    *inp_inv[INPUTS],
		    *ready, *fault;
	hal_float_t scale[NUMAXES],
		    maxaccel[NUMAXES],
		    adc_scale[SPINDLES],
		    pwm_scale[SPINDLES];
	hal_u32_t   *test;
} data_t;

static data_t *data;

static int comp_id;
static const char *modname = MODNAME;
static const char *prefix = PREFIX;

volatile unsigned *gpio;

static int mem_fd = -1;
static int mem_fd_spi = -1;

volatile int32_t txBuf[BUFSIZE], rxBuf[BUFSIZE];
static hal_u32_t pwm_period = 0;

static double dt = 0,				/* update_freq period in seconds */
	      recip_dt = 0,			/* reciprocal of period, avoids divides */
	      scale_inv[NUMAXES] = { 1.0 },	/* inverse of scale */
	      old_vel[NUMAXES] = { 0 },
	      old_pos[NUMAXES] = { 0 },
	      old_scale[NUMAXES] = { 0 },
	      max_vel;
static long old_dtns = 0;			/* update_freq funct period in nsec */
static hal_s32_t accum_diff = 0,
	   old_count[NUMAXES] = { 0 };
static rtapi_s64 accum[NUMAXES] = { 0 };		/* 64 bit DDS accumulator */

static void read_spi(void *arg, long period);
static void write_spi(void *arg, long period);
static void update(void *arg, long period);
void transfer_data();
static void reset_board();
static int map_gpio_and_spi();
static void setup_gpio();
static void restore_gpio();


static int spidev_rate = 16000000; //RTAPI_MP_ARRAY_INT(spidev_rate, MAX_BOARDS, "SPI clock rate in Hz");
static char *spidev_path = "/dev/spidev0.0"; //RTAPI_MP_ARRAY_STRING(spidev_path, MAX_BOARDS, "path to spi device");


static int spidev_set_lsb_first(int fd, uint8_t lsb_first) {
    return ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb_first);
}

static int spidev_set_mode(int fd, uint8_t mode) {
    return ioctl(fd, SPI_IOC_WR_MODE, &mode);
}

static int spidev_set_max_speed_hz(int fd, uint32_t speed_hz) {
    return ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz);
}

static int spidev_set_bits_per_word(int fd, uint8_t bits) {
    return ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
}

static int spidev_get_bits_per_word(int fd) {
    uint8_t bits;
    int r;
    r = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if(r < 0) return -1;
    return bits;
}

static int spidev_open_and_configure(char *dev, int rate) {
    int fd = open(dev, O_RDWR);

    if(fd < 0) return -errno;

    int r = spidev_set_lsb_first(fd, false);
    if(r < 0) goto fail_errno;

    r = spidev_set_mode(fd, 0);
    if(r < 0) goto fail_errno;

    r = spidev_set_bits_per_word(fd, 8);
    if(r < 0) goto fail_errno;

    r = spidev_set_max_speed_hz(fd, rate);
    if(r < 0) goto fail_errno;

    return fd;

fail_errno:
    r = -errno;
    close(fd);
    return r;
}

void spi_exchange(void)
{
    static struct spi_ioc_transfer t =
	{
		.bits_per_word = 8,
		.cs_change = 0,
		.delay_usecs = 0,
		.len = 32,
		.pad = 0,
		//.rx_buf = (uintptr_t)&rxBuf,
		.rx_nbits = 0,
		.speed_hz = 16000000,
		//.tx_buf = (uintptr_t)&txBuf,
		.tx_nbits = 0,
		.word_delay_usecs = 0
	};
	t.rx_buf = (uintptr_t)&rxBuf;
	t.tx_buf = (uintptr_t)&txBuf;

    int r = ioctl(mem_fd_spi, SPI_IOC_MESSAGE(1), &t);
    if(r < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "PICNC: SPI_IOC_MESSAGE: %s\n", strerror(errno));
    }
}

int rtapi_app_main(void)
{
	char name[HAL_NAME_LEN + 1];
	int n, retval;

	/* initialise driver */
	comp_id = hal_init(modname);
	if (comp_id < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n",
			modname);
		return -1;
	}

	/* allocate shared memory */
	data = hal_malloc(sizeof(data_t));
	if (data == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n",
			modname);
		hal_exit(comp_id);
		return -1;
	}

	/* configure board */
	retval = map_gpio_and_spi();
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: cannot map GPIO or SPI memory\n", modname);
		return retval;
	}

	setup_gpio();
	reset_board();

	pwm_period = ((SYS_FREQ/PWM_PRESCALE)/pwmfreq) - 1;	/* PeripheralClock/pwmfreq - 1 */

	txBuf[0] = 0x4746433E;			/* this is config data (>CFG) */
	txBuf[1] = stepwidth;
	txBuf[2] = pwm_period;
	transfer_data();			/* send config data */

	max_vel = BASEFREQ/(4.0 * stepwidth);	/* calculate velocity limit */

	/* export pins and parameters */
	for (n=0; n<NUMAXES; n++) {
		retval = hal_pin_float_newf(HAL_IN, &(data->position_cmd[n]),
			comp_id, "%s.axis.%01d.position-cmd", prefix, n);
		if (retval < 0) goto error;
		*(data->position_cmd[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->position_fb[n]),
			comp_id, "%s.axis.%01d.position-fb", prefix, n);
		if (retval < 0) goto error;
		*(data->position_fb[n]) = 0.0;

		retval = hal_param_float_newf(HAL_RW, &(data->scale[n]),
			comp_id, "%s.axis.%01d.scale", prefix, n);
		if (retval < 0) goto error;
		data->scale[n] = 1.0;

		retval = hal_param_float_newf(HAL_RW, &(data->maxaccel[n]),
			comp_id, "%s.axis.%01d.maxaccel", prefix, n);
		if (retval < 0) goto error;
		data->maxaccel[n] = 1.0;
	}

	for (n=0; n<SPINDLES; n++) {
		retval = hal_pin_float_newf(HAL_IN, &(data->pwm_duty[n]),
			comp_id, "%s.pwm.%01d.duty", prefix, n);
		if (retval < 0) goto error;
		*(data->pwm_duty[n]) = 0.0;

		retval = hal_param_float_newf(HAL_RW, &(data->pwm_scale[n]),
			comp_id, "%s.pwm.%01d.scale", prefix, n);
		if (retval < 0) goto error;
		data->pwm_scale[n] = 1.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->adc_in[n]),
			comp_id, "%s.adc.%01d.val", prefix, n);
		if (retval < 0) goto error;
		*(data->adc_in[n]) = 0.0;

		retval = hal_param_float_newf(HAL_RW, &(data->adc_scale[n]),
			comp_id, "%s.adc.%01d.scale", prefix, n);
		if (retval < 0) goto error;
		data->adc_scale[n] = 1.0;
	}

	for (n=0; n<INPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_OUT, &(data->inp[n]), comp_id,
			"%s.input.%01d.pin", prefix, n);
		if (retval < 0) goto error;
		*(data->inp[n]) = 0;

		retval = hal_pin_bit_newf(HAL_OUT, &(data->inp_inv[n]), comp_id,
			"%s.input.%01d.pin_inv", prefix, n);
		if (retval < 0) goto error;
		*(data->inp_inv[n]) = 1;
	}

	for (n=0; n<OUTPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_IN, &(data->out[n]), comp_id,
			"%s.output.%01d.pin", prefix, n);
		if (retval < 0) goto error;
		*(data->out[n]) = 0;
	}

	retval = hal_pin_bit_newf(HAL_OUT, &(data->ready), comp_id,
		"%s.ready", prefix);
	if (retval < 0) goto error;
	*(data->ready) = 0;

	retval = hal_pin_bit_newf(HAL_IO, &(data->fault), comp_id,
		"%s.fault", prefix);
	if (retval < 0) goto error;
	*(data->fault) = 0;

	retval = hal_pin_u32_newf(HAL_IN, &(data->test), comp_id,
		"%s.test", prefix);
	if (retval < 0) goto error;
	*(data->test) = 0;
error:
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: pin export failed with err=%i\n",
			modname, retval);
		hal_exit(comp_id);
		return -1;
	}

	/* export functions */
	rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
	retval = hal_export_funct(name, read_spi, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: read function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
	rtapi_snprintf(name, sizeof(name), "%s.write", prefix);
	/* no FP operations */
	retval = hal_export_funct(name, write_spi, data, 0, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: write function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
	rtapi_snprintf(name, sizeof(name), "%s.update", prefix);
	retval = hal_export_funct(name, update, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: update function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
	hal_ready(comp_id);
	return 0;
}

void rtapi_app_exit(void)
{
	restore_gpio();
	if (mem_fd > -1)   close(mem_fd);
	if (mem_fd_spi > -1)   close(mem_fd_spi);
	munmap((void *)gpio,BLOCK_SIZE);
	hal_exit(comp_id);
}


static inline void update_inputs(data_t *dat)
{
	int n;
	
	for (n=0; n<INPUTS; n++) {
		*(dat->inp[n])  = (get_inputs() & (1l << n)) ? 1 : 0;
		*(dat->inp_inv[n]) = !(*(dat->inp[n]));
	}

	*(dat->adc_in[0]) = dat->adc_scale[0] * ((hal_u32_t)get_adc(0) >> 16);
	// ! implement
	//*(dat->adc_in[1]) = dat->adc_scale[1] * (get_adc(0) & 0xFFFF);
	//*(dat->adc_in[2]) = dat->adc_scale[2] * ((hal_u32_t)get_adc(1) >> 16);
}

static void read_spi(void *arg, long period)
{
	int i;
	static int startup = 0;
	data_t *dat = (data_t *)arg;
	unsigned long timeout = REQ_TIMEOUT;

	/* send garbage command, just to read */
	txBuf[0] = 0x44414552;
#if 1
	/* send request */
	BCM2835_GPSET0 = (1l << 23);

	/* wait until ready */
	while (((BCM2835_GPLEV0 & (1l << 25)) == 0) && (timeout--));

	*(dat->test) = timeout;

	/* clear request */
	BCM2835_GPCLR0 = (1l << 23);
#endif
	if (timeout) transfer_data();

	/* sanity check: last reported received command was >CMD, or a one-off >CFG */
	if ((rxBuf[0] & 0xFFFFFF00) == (0x444D43FF ^ ~0)) {
		*(dat->ready) = 1;
	} else {
		*(dat->ready) = 0;
		if (!startup)
			startup = 1;
		else
			*(dat->fault) = 1;
	}

	/* check for change in period */
	if (period != old_dtns) {
		old_dtns = period;
		dt = period * 0.000000001;
		recip_dt = 1.0 / dt;
	}

	/* check for scale change */
	for (i = 0; i < NUMAXES; i++) {
		if (dat->scale[i] != old_scale[i]) {
			old_scale[i] = dat->scale[i];
			/* scale must not be 0 */
			if ((dat->scale[i] < 1e-20) && (dat->scale[i] > -1e-20))
				dat->scale[i] = 1.0;
			scale_inv[i] = (1.0 / STEP_MASK) / dat->scale[i];
		}
	}

	/* update outputs */
	for (i = 0; i < NUMAXES; i++) {
		/* the DDS uses 32 bit counter, this code converts
		   that counter into 64 bits */
		accum_diff = get_position(i) - old_count[i];
		old_count[i] = get_position(i);
		accum[i] += accum_diff;

		*(dat->position_fb[i]) = (float)(accum[i]) * scale_inv[i];
	}

	/* update input status */
	update_inputs(dat);
}

static void write_spi(void *arg, long period)
{
	transfer_data();
}

static inline void update_outputs(data_t *dat)
{
	float duty;
	int n;
	hal_u32_t x[SPINDLES_MAX];
	hal_s32_t y;
	
	/* update pic32 output */
	for (n = 0, y = 0; n < OUTPUTS; n++)
		y |= (*(dat->out[n]) ? 1l : 0) << n;

	txBuf[1 + NUMAXES] = y;

	/* update pwm */
	for (n = 0; n < SPINDLES; n++) {
		duty = *(dat->pwm_duty[n]) / dat->pwm_scale[n];
		if (duty < 0.0) duty = 0.0;
		if (duty > 1.0) duty = 1.0;

		x[n] = (duty * (1.0 + pwm_period));
		x[n] = (x[n] > 0xFFFF) ? 0xFFFF : x[n];
	}
	txBuf[2+NUMAXES] = x[1] << 16 | x[0];
	txBuf[3+NUMAXES] = x[2] << 16;
}

static void update(void *arg, long period)
{
	int i;
	data_t *dat = (data_t *)arg;
	double max_accl, vel_cmd, dv, new_vel,
	       dp, pos_cmd, curr_pos, match_accl, match_time, avg_v,
	       est_out, est_cmd, est_err;

	for (i = 0; i < NUMAXES; i++) {
		/* set internal accel limit to its absolute max, which is
		   zero to full speed in one thread period */
		max_accl = max_vel * recip_dt;

		/* check for user specified accel limit parameter */
		if (dat->maxaccel[i] <= 0.0) {
			/* set to zero if negative */
			dat->maxaccel[i] = 0.0;
		} else {
			/* parameter is non-zero, compare to max_accl */
			if ((dat->maxaccel[i] * fabs(dat->scale[i])) > max_accl) {
				/* parameter is too high, lower it */
				dat->maxaccel[i] = max_accl / fabs(dat->scale[i]);
			} else {
				/* lower limit to match parameter */
				max_accl = dat->maxaccel[i] * fabs(dat->scale[i]);
			}
		}

		/* calculate position command in counts */
		pos_cmd = *(dat->position_cmd[i]) * dat->scale[i];
		/* calculate velocity command in counts/sec */
		vel_cmd = (pos_cmd - old_pos[i]) * recip_dt;
		old_pos[i] = pos_cmd;

		/* apply frequency limit */
		if (vel_cmd > max_vel) {
			vel_cmd = max_vel;
		} else if (vel_cmd < -max_vel) {
			vel_cmd = -max_vel;
		}

		/* determine which way we need to ramp to match velocity */
		if (vel_cmd > old_vel[i])
			match_accl = max_accl;
		else
			match_accl = -max_accl;

		/* determine how long the match would take */
		match_time = (vel_cmd - old_vel[i]) / match_accl;
		/* calc output position at the end of the match */
		avg_v = (vel_cmd + old_vel[i]) * 0.5;
		curr_pos = (double)(accum[i]) * (1.0 / STEP_MASK);
		est_out = curr_pos + avg_v * match_time;
		/* calculate the expected command position at that time */
		est_cmd = pos_cmd + vel_cmd * (match_time - 1.5 * dt);
		/* calculate error at that time */
		est_err = est_out - est_cmd;

		if (match_time < dt) {
			/* we can match velocity in one period */
			if (fabs(est_err) < 0.0001) {
				/* after match the position error will be acceptable */
				/* so we just do the velocity match */
				new_vel = vel_cmd;
			} else {
				/* try to correct position error */
				new_vel = vel_cmd - 0.5 * est_err * recip_dt;
				/* apply accel limits */
				if (new_vel > (old_vel[i] + max_accl * dt)) {
					new_vel = old_vel[i] + max_accl * dt;
				} else if (new_vel < (old_vel[i] - max_accl * dt)) {
					new_vel = old_vel[i] - max_accl * dt;
				}
			}
		} else {
			/* calculate change in final position if we ramp in the
			opposite direction for one period */
			dv = -2.0 * match_accl * dt;
			dp = dv * match_time;
			/* decide which way to ramp */
			if (fabs(est_err + dp * 2.0) < fabs(est_err)) {
				match_accl = -match_accl;
			}
			/* and do it */
			new_vel = old_vel[i] + match_accl * dt;
		}

		/* apply frequency limit */
		if (new_vel > max_vel) {
			new_vel = max_vel;
		} else if (new_vel < -max_vel) {
			new_vel = -max_vel;
		}

		old_vel[i] = new_vel;
		/* calculate new velocity cmd */
		update_velocity(i, (new_vel * VELSCALE));
	}

	update_outputs(dat);

	/* this is a command (>CMD) */
	txBuf[0] = 0x444D433E;
}

void transfer_data()
{
#if 0	
	char *buf;
	int i;

	/* activate transfer */
	BCM2835_SPICS = SPI_CS_TA;

	/* send txBuf */
	buf = (char *)txBuf;
	for (i=0; i<SPIBUFSIZE; i++) {
		BCM2835_SPIFIFO = *buf++;
	}

	/* wait until transfer is finished */
	while (!(BCM2835_SPICS & SPI_CS_DONE));

	/* clear DONE bit */
	BCM2835_SPICS = SPI_CS_DONE;

	/* read buffer */
	buf = (char *)rxBuf;
	for (i=0; i<SPIBUFSIZE; i++) {
		*buf++ = BCM2835_SPIFIFO;
	}
#else
	spi_exchange();
#endif
}

static int setup_gpiomem_access(void)
{
  if ((mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC)) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,"HAL_PI_GPIO: can't open /dev/gpiomem:  %d - %s\n"
        "If the error is 'permission denied' then try adding the user who runs\n"
        "LinuxCNC to the gpio group: sudo gpasswd -a username gpio\n", errno, strerror(errno));
    return -1;
  }

  gpio = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0);

  if (gpio == MAP_FAILED) {
    close(mem_fd);
    mem_fd = -1;
    rtapi_print_msg(RTAPI_MSG_ERR, "HAL_PICNC: mmap failed: %d - %s\n", errno, strerror(errno));
    return -1;
  }

  return 0;
}

static int setup_spimem_access(void)
{
	#if 0
  if ((mem_fd_spi = open("/dev/spidev0.0", O_RDWR|O_SYNC)) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,"HAL_PI_GPIO: can't open /dev/spidev0.0:  %d - %s\n"
        "If the error is 'permission denied' then try adding the user who runs\n"
        "LinuxCNC to the gpio group: sudo gpasswd -a username gpio\n", errno, strerror(errno));
    return -1;
  }

  spi = mmap(NULL, BCM2835_BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd_spi, 0);

  if (spi == MAP_FAILED) {
    close(mem_fd_spi);
    mem_fd_spi = -1;
    rtapi_print_msg(RTAPI_MSG_ERR, "HAL_PICNC: mmap failed: %d - %s\n", errno, strerror(errno));
    return -1;
  }

  return 0;
  #else
  mem_fd_spi = spidev_open_and_configure(spidev_path, spidev_rate);
  if (mem_fd_spi < 0)
  {
	rtapi_print_msg(RTAPI_MSG_ERR, "HAL_PICNC: spi init failed: %d - %s\n", errno, strerror(errno));
	return -1;
  }
  return 0;
  #endif
}

int map_gpio_and_spi()
{
	if(setup_gpiomem_access() != 0)
		return -1;
	if (setup_spimem_access() != 0)
		return -1;
	return 0;
}

/*    GPIO USAGE
 *
 *	GPIO	Dir	Signal		Note
 *
 *	25	IN	DATA READY	
 *	9	IN	MISO		SPI
 *	23	OUT	DATA REQUEST	
 *	10	OUT	MOSI		SPI
 *	11	OUT	SCLK		SPI
 *	7	OUT	RESET		active low, then Hi-Z
 *
 */

void setup_gpio()
{
	hal_u32_t x;

	/* data ready GPIO 25, input */
	x = BCM2835_GPFSEL2;
	x &= ~(0b111 << (5*3));
	BCM2835_GPFSEL2 = x;

	/* data request GPIO 23, output */
	x = BCM2835_GPFSEL2;
	x &= ~(0b111 << (3*3));
	x |= (0b001 << (3*3));
	BCM2835_GPFSEL2 = x;

	/* reset GPIO 7, output */
	x = BCM2835_GPFSEL0;
	x &= ~(0b111 << (7*3));
	x |= (0b001 << (7*3));
	BCM2835_GPFSEL0 = x;
#if SPI_PINS_ARE_INPUTS
	/* change SPI pins */
	x = BCM2835_GPFSEL0;
	x &= ~(0b111 << (9*3));
	x |=   (0b100 << (9*3));
	BCM2835_GPFSEL0 = x;

	x = BCM2835_GPFSEL1;
	x &= ~(0b111 << (0*3) | 0b111 << (1*3));
	x |= (0b100 << (0*3) | 0b100 << (1*3));
	BCM2835_GPFSEL1 = x;
#endif
#if 0
	/* set up SPI */
	BCM2835_SPICLK = SPICLKDIV;
	BCM2835_SPICS = 0;

	/* clear FIFOs */
	BCM2835_SPICS |= SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX;

	/* clear done bit */
	BCM2835_SPICS |= SPI_CS_DONE;
#endif
}

void restore_gpio()
{
	hal_u32_t x;

	/* change all used pins back to inputs */

	/* GPIO 7 */
	x = BCM2835_GPFSEL0;
	x &= ~(0b111 << (7*3));
	BCM2835_GPFSEL0 = x;

	/* GPIO 23 */
	x = BCM2835_GPFSEL2;
	x &= ~(0b111 << (3*3));
	BCM2835_GPFSEL2 = x;

	/* GPIO 25 */
	x = BCM2835_GPFSEL2;
	x &= ~(0b111 << (5*3));
	BCM2835_GPFSEL2 = x;
#if SPI_PINS_ARE_INPUTS
	/* change SPI pins to inputs*/
	x = BCM2835_GPFSEL0;
	x &= ~(0b111 << (9*3));
	BCM2835_GPFSEL0 = x;

	x = BCM2835_GPFSEL1;
	x &= ~(0b111 << (0*3) | 0b111 << (1*3));
	BCM2835_GPFSEL1 = x;
#endif
}

void reset_board()
{
	hal_u32_t x,i;

	/* GPIO 7 is configured as a tri-state output pin */

	/* set as output GPIO 7 */
	x = BCM2835_GPFSEL0;
	x &= ~(0b111 << (7*3));
	x |= (0b001 << (7*3));
	BCM2835_GPFSEL0 = x;

	/* board reset is active low */
	for (i=0; i<0x10000; i++)
		BCM2835_GPCLR0 = (1l << 7);

	/* wait until the board is ready */
	for (i=0; i<0x300000; i++)
		BCM2835_GPSET0 = (1l << 7);

	/* reset GPIO 7 back to input */
	x = BCM2835_GPFSEL0;
	x &= ~(0b111 << (7*3));
	BCM2835_GPFSEL0 = x;
}
