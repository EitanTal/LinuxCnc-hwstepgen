#include <stdint.h>
#include "main.h"
#include "kernel-stepgen.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim1;

#define SPI_TRANSACTION_SIZE 32

#define RECORD_DATA()     (HAL_GPIO_ReadPin(DATA_REQUEST_GPIO_Port, DATA_REQUEST_Pin) == GPIO_PIN_SET)
#define DATA_READY()      (HAL_GPIO_WritePin(DATA_READY_GPIO_Port, DATA_READY_Pin, GPIO_PIN_SET))
#define DATA_NOT_READY()  (HAL_GPIO_WritePin(DATA_READY_GPIO_Port, DATA_READY_Pin, GPIO_PIN_RESET))
#define LED_TOGGLE()      LED_GPIO_Port->ODR ^= LED_Pin

enum
{
    CMD_UPDATE = 'DMC>', //'>CMD',
    CMD_CONFIG = 'GFC>', //'>CFG',
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

static void reset_board(void)
{

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
    S_RX_PACKET * a = (S_RX_PACKET *)spi_rx;
    S_TX_REPLY * b  = (S_TX_REPLY *)spi_tx;
    uint32_t cmd = a->command;
    b->last_command_inverted = ~cmd;
    if (cmd == CMD_UPDATE)
    {
        S_RX_UPDATE* a = (S_RX_UPDATE*)&spi_rx;
        stepgen_update_input(&a->velocity);
        htim1.Instance->CCR1 = (a->pwm1 & 0xFFFF);
        OUT1_GPIO_Port->ODR = OUT1_GPIO_Port->ODR = ((OUT1_GPIO_Port->ODR & ~((0xF) << 6)) | ((a->output & 0xF) << 6));
    }
    else if (cmd == CMD_CONFIG)
    {
        S_RX_CONFIG* a = (S_RX_CONFIG*)&spi_rx;
        stepgen_update_stepwidth(a->stepwidth);
        htim1.Instance->ARR = a->pwmfreq;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    pending_spi = 1;
}

void kernel_main_entry(void)
{
    reset_board();
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&spi_tx, (uint8_t*)&spi_rx, sizeof(spi_tx));
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
            if (hspi1.State != HAL_SPI_STATE_BUSY_TX_RX)
            {
                HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&spi_tx, (uint8_t*)&spi_rx, sizeof(spi_tx));
            }
            DATA_READY();
        }
        else
            DATA_NOT_READY();

        if (pending_spi)
        {
            pending_spi = 0;
            idle_counter = 200000;
            process_spi();
        }

        if (hspi1.State != HAL_SPI_STATE_BUSY_TX_RX)
        {
            HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&spi_tx, (uint8_t*)&spi_rx, sizeof(spi_tx));
        }

        /* shutdown stepgen if no activity */
        if (idle_counter)
            idle_counter--;
        else
            reset_board();
#if 1
        if (!(counter++ % (idle_counter ? 0x10000 : 0x20000))) {
            LED_TOGGLE();
        }
#endif
    }
}

