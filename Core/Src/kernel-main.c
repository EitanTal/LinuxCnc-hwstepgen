#include <stdint.h>

#define AXES 4
#define SPI_TRANSACTION_SIZE 32

#define RECORD_DATA()     0
#define DATA_READY()      0
#define DATA_NOT_READY()  0

enum
{
    CMD_UPDATE = '>CMD',
    CMD_CONFIG = '>CFG',
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
    uint32_t    stepwidth;
    uint32_t    pwmfreq;
} S_RX_UPDATE;

typedef struct
{
    uint32_t    last_command_inverted;
    int32_t     positions[AXES];
    uint32_t    input;
} S_TX_REPLY;

volatile uint8_t spi_tx[SPI_TRANSACTION_SIZE];
volatile uint8_t spi_rx[SPI_TRANSACTION_SIZE];

int pending_spi;

void reset_board(void)
{

}

void kernel_main_entry(void)
{
    reset_board();
    int counter = 0;
    int idle_counter = 0;
    for (;;)
    {
        if (RECORD_DATA())
        {
            snapshot();
            DATA_READY();
        }
        else
            DATA_NOT_READY();

        if (pending_spi)
        {
            idle_counter = 200000;
            process_spi();
        }

        /* shutdown stepgen if no activity */
        if (idle_counter)
            idle_counter--;
        else
            reset_board();

        if (!(counter++ % (idle_counter ? 0x10000 : 0x20000))) {
            LED_TOGGLE;
        }
    }
}
