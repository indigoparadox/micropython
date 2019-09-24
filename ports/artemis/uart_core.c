#include <unistd.h>
#include "py/mpconfig.h"

#include "am_mcu_apollo.h"

void *phUART;

#define CHECK_ERRORS(x)                                                       \
    if ((x) != AM_HAL_STATUS_SUCCESS)                                         \
    {                                                                         \
        error_handler(x);                                                     \
    }

volatile uint32_t ui32LastError;

void error_handler(uint32_t ui32ErrorStatus)
{
    ui32LastError = ui32ErrorStatus;

    while (1);
}

uint8_t g_pui8TxBuffer[256];
//uint8_t g_pui8RxBuffer[2];

const am_hal_uart_config_t g_sUartConfig =
{
    .ui32BaudRate = 115200,
    .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
    .ui32Parity = AM_HAL_UART_PARITY_NONE,
    .ui32StopBits = AM_HAL_UART_ONE_STOP_BIT,
    .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

    .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 |
                       AM_HAL_UART_RX_FIFO_1_2),

    .pui8TxBuffer = g_pui8TxBuffer,
    .ui32TxBufferSize = sizeof(g_pui8TxBuffer),
    .pui8RxBuffer = 0,
    .ui32RxBufferSize = 0,
};

void am_uart_isr(void)
{
    uint32_t ui32Status, ui32Idle;
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
    am_hal_uart_interrupt_service(phUART, ui32Status, &ui32Idle);
}

#define AM_BSP_UART_PRINT_INST          0
#define AM_BSP_GPIO_COM_UART_TX         48
#define AM_BSP_GPIO_COM_UART_RX         49
const am_hal_gpio_pincfg_t g_AM_BSP_GPIO_COM_UART_TX =
{
    .uFuncSel            = AM_HAL_PIN_48_UART0TX,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

const am_hal_gpio_pincfg_t g_AM_BSP_GPIO_COM_UART_RX =
{
    .uFuncSel            = AM_HAL_PIN_49_UART0RX
};

void uart_init()
{
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    CHECK_ERRORS(am_hal_uart_initialize(0, &phUART));
    CHECK_ERRORS(am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_WAKE, false));
    CHECK_ERRORS(am_hal_uart_configure(phUART, &g_sUartConfig));

    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX);

    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));
    am_hal_interrupt_master_enable();
}

/*
 * Core UART functions to implement for a port
 */

// Receive single character
int mp_hal_stdin_rx_chr(void) {
    uint32_t ui32BytesRead = 0;
    uint8_t chr;

    const am_hal_uart_transfer_t sUartRead =
    {
        .ui32Direction = AM_HAL_UART_READ,
        .pui8Data = &chr,
        .ui32NumBytes = 1,
        .ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,
        .pui32BytesTransferred = &ui32BytesRead,
    };

    am_hal_uart_transfer(phUART, &sUartRead);

    return (int) chr;
}

// Send string of given length
void mp_hal_stdout_tx_strn(const char *str, mp_uint_t len) {
    uint32_t ui32BytesWritten = 0;

    const am_hal_uart_transfer_t sUartWrite =
    {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = (uint8_t *) str,
        .ui32NumBytes = len,
        .ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,
        .pui32BytesTransferred = &ui32BytesWritten,
    };

    am_hal_uart_transfer(phUART, &sUartWrite);
}
