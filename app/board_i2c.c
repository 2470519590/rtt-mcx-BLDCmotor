#include "board_i2c.h"

#include "fsl_clock.h"
#include "fsl_lpi2c.h"
#include "fsl_port.h"

#define BOARD_I2C3_READ_RETRIES (3U)

static bool s_i2c3_ready;
static unsigned s_i2c3_fail_streak;

static void board_i2c3_recover(void)
{
    uint32_t flags = LPI2C_MasterGetStatusFlags(LPI2C3);
    LPI2C_MasterClearStatusFlags(LPI2C3, flags);
    LPI2C3->MCR |= LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK;
}

void board_i2c3_init(uint32_t baudrate_hz)
{
    lpi2c_master_config_t config;
    port_pin_config_t pin_config = {0};

    CLOCK_EnableClock(kCLOCK_GatePORT3);
    RESET_ReleasePeripheralReset(kPORT3_RST_SHIFT_RSTn);
    pin_config.pullSelect = kPORT_PullDisable;
    pin_config.slewRate = kPORT_FastSlewRate;
    /* I2C lines must be open-drain. BOARD_InitI2C3Pins() configures the
       same pins this way; keep this late configuration consistent with it. */
    pin_config.openDrainEnable = kPORT_OpenDrainEnable;
    pin_config.driveStrength = kPORT_LowDriveStrength;
    pin_config.driveStrength1 = kPORT_NormalDriveStrength;
    pin_config.mux = kPORT_MuxAlt2;
    pin_config.inputBuffer = kPORT_InputBufferEnable;
    pin_config.invertInput = kPORT_InputNormal;
    pin_config.lockRegister = kPORT_UnlockRegister;
    PORT_SetPinConfig(PORT3, 27U, &pin_config);
    PORT_SetPinConfig(PORT3, 28U, &pin_config);

    CLOCK_SetClockDiv(kCLOCK_DivLPI2C3, 1U);
    CLOCK_AttachClk(kFRO_LF_DIV_to_LPI2C3);
    RESET_PeripheralReset(kLPI2C3_RST_SHIFT_RSTn);
    LPI2C_MasterGetDefaultConfig(&config);
    config.baudRate_Hz = (baudrate_hz == 0U) ? 400000U : baudrate_hz;
    LPI2C_MasterInit(LPI2C3, &config, CLOCK_GetLpi2cClkFreq(3U));
    s_i2c3_ready = true;
    s_i2c3_fail_streak = 0U;
}

status_t board_i2c3_read_register(uint16_t address, uint8_t reg,
                                  uint8_t *data, size_t length)
{
    lpi2c_master_transfer_t transfer = {0};
    status_t result = kStatus_InvalidArgument;

    if (!s_i2c3_ready || address > 0x7FU || data == NULL || length == 0U) {
        return kStatus_InvalidArgument;
    }

    transfer.slaveAddress = address;
    transfer.direction = kLPI2C_Read;
    transfer.subaddress = reg;
    transfer.subaddressSize = 1U;
    transfer.data = data;
    transfer.dataSize = length;
    transfer.flags = kLPI2C_TransferDefaultFlag;

    for (unsigned attempt = 0U; attempt < BOARD_I2C3_READ_RETRIES; ++attempt) {
        result = LPI2C_MasterTransferBlocking(LPI2C3, &transfer);
        if (result == kStatus_Success) {
            s_i2c3_fail_streak = 0U;
            return result;
        }
        /* AS5600 can NAK a transaction while its internal conversion state
           changes. Clear FIFO/error state before the bounded retry. */
        board_i2c3_recover();
    }
    if (++s_i2c3_fail_streak >= 3U) {
        /* Rebuild the peripheral after repeated NAKs. This is intentionally
           bounded: a permanently disconnected sensor must not cause a reset
           on every 2 ms control iteration. */
        s_i2c3_ready = false;
        LPI2C_MasterDeinit(LPI2C3);
        RESET_PeripheralReset(kLPI2C3_RST_SHIFT_RSTn);
        board_i2c3_init(400000U);
    }
    return result;
}
