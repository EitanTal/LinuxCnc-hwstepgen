#include <stdint.h>
#include <string.h>
#include "main.h"
#include "kernel-stepgen.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim1;

#define SPI_TRANSACTION_SIZE 32

#define RECORD_DATA()     (HAL_GPIO_ReadPin(DATA_REQUEST_GPIO_Port, DATA_REQUEST_Pin) == GPIO_PIN_SET)
#define EXTERNAL_RESET()  (HAL_GPIO_ReadPin(EXT_RESET_GPIO_Port,   EXT_RESET_Pin) == GPIO_PIN_RESET)
#define DATA_READY()      (HAL_GPIO_WritePin(DATA_READY_GPIO_Port, DATA_READY_Pin, GPIO_PIN_SET))
#define DATA_NOT_READY()  (HAL_GPIO_WritePin(DATA_READY_GPIO_Port, DATA_READY_Pin, GPIO_PIN_RESET))
#define LED_TOGGLE()      LED_GPIO_Port->ODR ^= LED_Pin
#if 0
#define FAILURE_CONDITION()  for (;;) LED_TOGGLE()
#else
#define FAILURE_CONDITION() hard_reset_board()
#endif
enum
{
    CMD_UPDATE = 'DMC>', //'>CMD',
    CMD_CONFIG = 'GFC>', //'>CFG',
    CMD_GARBAGE = 'DAER', //'READ'
};

typedef struct
{
    uint32_t    command;
    int32_t     velocity[AXES];
    uint32_t    output;
    uint32_t    pwm1;
    uint32_t    pwm2;
} S_RX_UPDATE;

typedef struct
{
    uint32_t    command;
} S_RX_PACKET;

typedef struct
{
    uint32_t    command;
    uint32_t    stepwidth;
    uint32_t    pwmfreq;
} S_RX_CONFIG;

typedef struct
{
    uint32_t    last_command_inverted;
    int32_t     positions[AXES];
    uint32_t    input;
} S_TX_REPLY;

volatile uint8_t totalbuffer_tx[SPI_TRANSACTION_SIZE * 2];
volatile uint8_t totalbuffer_rx[SPI_TRANSACTION_SIZE * 2];

volatile uint8_t * spi_tx;
volatile uint8_t * spi_rx;

int pending_spi_1;
int pending_spi_2;
uint32_t g_last_cmd;
uint8_t which_half_buffer = 0;

static inline void update_outputs(int new_status)
{
  new_status = new_status << 6; // output bits start from 6

  static int old_status = 0;

  int bits_just_turned_on = (~old_status & new_status);
  int bits_just_turned_off = (old_status & ~new_status);

  OUT1_GPIO_Port->BSRR = (bits_just_turned_on | (bits_just_turned_off << 16));
  old_status = new_status;
}

static inline void update_pwm_duty(uint32_t pwm12, uint32_t pwm3)
{
    // pwm3 is currently unused, pwm2 (high 16 bits of pwm12) also unused
    htim1.Instance->CCR1 = (pwm12 & 0xFFFF);
}

static void reset_spi(void)
{
    which_half_buffer = 0;
    HAL_SPI_Abort(&hspi1);
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&totalbuffer_tx, (uint8_t*)&totalbuffer_rx, sizeof(totalbuffer_tx));
}

static void board_to_idle(void)
{
    stepgen_reset();
    update_outputs(0);
    update_pwm_duty(0,0);
    reset_spi();
}

static void hard_reset_board(void)
{
    stepgen_reset();
    update_outputs(0);
    update_pwm_duty(0,0);

    HAL_NVIC_SystemReset();
}

static void snapshot(void)
{
    S_TX_REPLY* a = (S_TX_REPLY*)spi_tx;
    if (a)
    {
      stepgen_get_position(&a->positions);
      int InPortStatus = IN1_GPIO_Port->IDR;
      InPortStatus = InPortStatus >> 12;
      InPortStatus &= 0xF; // PB12...PB15
      a->input = InPortStatus;
    }
}

static int process_spi(void)
{
    volatile S_RX_PACKET * a = (volatile S_RX_PACKET *)spi_rx;
    uint32_t cmd = a->command;
    g_last_cmd = cmd;
    if (cmd == CMD_UPDATE)
    {
        S_RX_UPDATE aa;
        memcpy((uint8_t*)&aa, (uint8_t*)spi_rx, sizeof(aa) );
        stepgen_update_input(&aa.velocity);
        update_pwm_duty(aa.pwm1, 0);
        update_outputs(aa.output);
    }
    else if (cmd == CMD_CONFIG)
    {
#if 0
        S_RX_CONFIG* a = (S_RX_CONFIG*)spi_rx;
#else
        uint8_t tmp_buf[32] = {0};
        memcpy((uint8_t*)tmp_buf, (uint8_t*)spi_rx, 32);
        S_RX_CONFIG* a = (S_RX_CONFIG*)tmp_buf;
#endif
        stepgen_update_stepwidth(a->stepwidth);
        htim1.Instance->ARR = (a->pwmfreq);
    }
    else if (cmd == CMD_GARBAGE)
    {
        // Garbage commands are sent from PC for no other reason other than collecting the snapshot data.
        // The snapshot event itself is kicked off via a side channel.
    }
    else
    {
        LED_TOGGLE(); // Debug: Show an unknown command
        return 0;
    }
    return 1;
}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    pending_spi_1 = 1;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    pending_spi_2 = 1;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    FAILURE_CONDITION();
}

void kernel_main_entry(void)
{
    board_to_idle();

    // one-off setup:
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    htim1.Instance->CCER = 0x1;

    int counter = 0;
    int idle_counter = 0;
    for (;;)
    {
        if (RECORD_DATA())
        {
            snapshot();
            DATA_READY();
            int data_fetch_timeout = 0x500;
            while (RECORD_DATA() && data_fetch_timeout) { data_fetch_timeout--;} // busywait until the PC ACKED
        }
        else
        {
            DATA_NOT_READY();
        }

        static int pending_spi = 0;
        if (which_half_buffer == 0 && pending_spi_1)
        {
            pending_spi_1 = 0;
            pending_spi = 1;
            spi_tx = &totalbuffer_tx[SPI_TRANSACTION_SIZE];
            spi_rx = &totalbuffer_rx[0];
        }

        if (which_half_buffer == 1 && pending_spi_2)
        {
            pending_spi_2 = 0;
            pending_spi = 1;
            spi_tx = &totalbuffer_tx[0];
            spi_rx = &totalbuffer_rx[SPI_TRANSACTION_SIZE];
        }

        if (pending_spi)
        {
            pending_spi = 0;
            which_half_buffer = !which_half_buffer;
            if (process_spi()) idle_counter = 200000;

            // prepare for future response:
            uint32_t last_cmd = *(uint32_t*)spi_rx;
            // 1st byte of spi_tx is always 0xA5
            *(uint32_t*)spi_tx = ~((last_cmd & 0xFFFFFF00) | 0x5A);
        }

        /* shutdown stepgen if no activity */
        if (idle_counter) {
            if (--idle_counter) { ; } // still going
            else { board_to_idle(); }
        }

        if (EXTERNAL_RESET())
        {
            hard_reset_board();
            while (EXTERNAL_RESET()) {LED_TOGGLE();} // busywait until external reset is cleared
        }

        if (!(counter++ % (idle_counter ? 0x2000 : 0x20000))) {
            LED_TOGGLE();
        }
    }
}

