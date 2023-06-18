#include <stdint.h>
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
    CMD_GARBAGE = 'DMCX',
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

volatile uint8_t spi_tx[SPI_TRANSACTION_SIZE]; // volatile because DMA writes to this
volatile uint8_t spi_rx[SPI_TRANSACTION_SIZE];

int pending_spi;
uint32_t g_last_cmd;

static inline void update_outputs(int new_status)
{
  typedef struct
  {
    uint32_t BitsIDontCareAbout : 6;
    uint32_t BitsOfInterest : 4;
  } BitsICareAbout;
  BitsICareAbout * bits = (BitsICareAbout *)&OUT1_GPIO_Port->ODR;
  bits->BitsOfInterest = new_status;
}

static inline void update_pwm_duty(uint32_t pwm12, uint32_t pwm3)
{
    // pwm3 is currently unusued, pwm2 (high 16 bits of pwm12) also unused
    htim1.Instance->CCR1 = (pwm12 & 0xFFFF);
}

static void reset_spi(void)
{
    HAL_SPI_Abort(&hspi1);
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&spi_tx, (uint8_t*)&spi_rx, sizeof(spi_tx));
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
    S_TX_REPLY* a = (S_TX_REPLY*)&spi_tx;
    stepgen_get_position(&a->positions);
    int InPortStatus = IN1_GPIO_Port->IDR;
    InPortStatus = InPortStatus >> 12;
    InPortStatus &= 0xF; // PB12...PB15
    a->input = InPortStatus;
}

static void process_spi(void)
{
    volatile S_RX_PACKET * a = (volatile S_RX_PACKET *)spi_rx;
    volatile S_TX_REPLY * b  = (volatile S_TX_REPLY *)spi_tx;
    uint32_t cmd = a->command;
    g_last_cmd = cmd;
    if (cmd == CMD_UPDATE)
    {
        S_RX_UPDATE aa; //= (S_RX_UPDATE*)&spi_rx;
        memcpy(&aa, spi_rx, sizeof(aa) );
        stepgen_update_input(&aa.velocity);
        update_pwm_duty(aa.pwm1, 0);
        update_outputs(aa.output);
    }
    else if (cmd == CMD_CONFIG)
    {
        S_RX_CONFIG* a = (S_RX_CONFIG*)&spi_rx;
        stepgen_update_stepwidth(a->stepwidth);
        htim1.Instance->ARR = a->pwmfreq;
    }
    else if (cmd == CMD_GARBAGE)
    {
        // Garbage commands are sent from PC for no other reason other than collecting the snapshot data.
        // The snapshot event itself is kicked off via a side channel.
    }
    else
    {
        LED_TOGGLE(); // Debug: Show an unknown command
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    pending_spi = 1;
    // Race condition: Cannot restart the SPI dma yet, as the DMA might be busy
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    FAILURE_CONDITION();
}

void kernel_main_entry(void)
{
    board_to_idle();
    //HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&spi_tx, (uint8_t*)&spi_rx, sizeof(spi_tx)); // kickoff the SPI

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

        if (pending_spi)
        {
            pending_spi = 0;
            idle_counter = 200000;
            process_spi();
            // re-kickoff. Last CMD must be prepared immediately before the neck kickoff
            *(uint32_t*)spi_tx = ~(*(uint32_t*)spi_rx);
            // This function commits the first byte of spi_tx, even for a future SPI transaction
            if ( HAL_OK !=  HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&spi_tx, (uint8_t*)&spi_rx, sizeof(spi_tx)))
            {
                FAILURE_CONDITION();
            };
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

        if (!(counter++ % (idle_counter ? 0x10000 : 0x20000))) {
            LED_TOGGLE();
        }
    }
}

