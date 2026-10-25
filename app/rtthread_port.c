#include "rtthread.h"
#include "rthw.h"
#include "fsl_clock.h"
#include "fsl_lpuart.h"
#include "board.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_port.h"
#include "board_uart.h"

static uint8_t rt_heap[32 * 1024] __attribute__((aligned(8)));
volatile int32_t g_host_uart_init_status = -1;
volatile host_uart_debug_t g_host_uart_debug;

void rt_hw_debug_uart_snapshot(void)
{
    /* Copy reviewed read-only registers to RAM; never read DATA here because
       doing so would consume an incoming byte. */
    g_host_uart_debug.source_hz = CLOCK_GetLpuartClkFreq(3U);
    g_host_uart_debug.baud = LPUART3->BAUD;
    g_host_uart_debug.ctrl = LPUART3->CTRL;
    g_host_uart_debug.stat = LPUART3->STAT;
    g_host_uart_debug.modir = LPUART3->MODIR;
    g_host_uart_debug.global = LPUART3->GLOBAL;
    g_host_uart_debug.pincfg = LPUART3->PINCFG;
    g_host_uart_debug.fifo = LPUART3->FIFO;
    g_host_uart_debug.water = LPUART3->WATER;
    g_host_uart_debug.rx_pcr = PORT4->PCR[2];
    g_host_uart_debug.tx_pcr = PORT4->PCR[5];
    g_host_uart_debug.pad_inputs = GPIO4->PDIR;
}

/* Bypass the faulty on-board MCU-Link UART bridge. */
LPUART_Type *rt_hw_debug_uart(void)
{
    return LPUART3;
}

static void external_uart_init(void)
{
    CLOCK_EnableClock(kCLOCK_GatePORT4);
    CLOCK_EnableClock(kCLOCK_GateGPIO4);
    RESET_ReleasePeripheralReset(kGPIO4_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kPORT4_RST_SHIFT_RSTn);
    const port_pin_config_t pin = {
        .pullSelect = kPORT_PullDisable,
        .pullValueSelect = kPORT_LowPullResistor,
        .slewRate = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .openDrainEnable = kPORT_OpenDrainDisable,
        .driveStrength = kPORT_LowDriveStrength,
        .driveStrength1 = kPORT_NormalDriveStrength,
        .mux = kPORT_MuxAlt3,
        .inputBuffer = kPORT_InputBufferEnable,
        .invertInput = kPORT_InputNormal,
        .lockRegister = kPORT_UnlockRegister
    };
    PORT_SetPinConfig(PORT4, 2U, &pin);
    PORT_SetPinConfig(PORT4, 5U, &pin);
    CLOCK_SetClockDiv(kCLOCK_DivLPUART3, 1U);
    CLOCK_AttachClk(kFRO_LF_DIV_to_LPUART3);
    RESET_PeripheralReset(kLPUART3_RST_SHIFT_RSTn);
    lpuart_config_t config;
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200U;
    config.enableTx = true;
    config.enableRx = true;
    g_host_uart_init_status = LPUART_Init(LPUART3, &config, CLOCK_GetLpuartClkFreq(3U));
    rt_hw_debug_uart_snapshot();
    if (g_host_uart_init_status != kStatus_Success) {
        /* Do not enter blocking console TX on an uninitialized UART. */
        while (1) { __NOP(); }
    }
}

static void rt_hw_uart_putc(LPUART_Type *uart, uint8_t ch)
{
    /* LPUART_WriteByte() writes DATA directly; it does not wait for the
       transmit register.  Waiting here prevents characters being lost. */
    while ((LPUART_GetStatusFlags(uart) & kLPUART_TxDataRegEmptyFlag) == 0U)
    {
    }
    LPUART_WriteByte(uart, ch);
}

void SysTick_Handler(void)
{
    rt_interrupt_enter();
    rt_tick_increase();
    rt_interrupt_leave();
}

void rt_hw_console_output(const char *str)
{
    rt_base_t level = rt_hw_interrupt_disable();
    LPUART_Type *uart = rt_hw_debug_uart();

    while (*str)
    {
        if (*str == '\n')
        {
            rt_hw_uart_putc(uart, '\r');
        }
        rt_hw_uart_putc(uart, (uint8_t)*str++);
    }

    /* Prevent the echo thread/tick interrupt from interleaving characters
       into a console line sent through the same polling UART. */
    rt_hw_interrupt_enable(level);
}

void rt_hw_board_init(void)
{
    BOARD_BootClockFROHF45M();
    BOARD_InitBootPins();
    external_uart_init();

    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);
    rt_system_heap_init(rt_heap, rt_heap + sizeof(rt_heap));
}
