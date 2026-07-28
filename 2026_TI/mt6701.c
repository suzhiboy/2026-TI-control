#include "mt6701.h"

static uint8_t mt6701_active_addr = MT6701_I2C_ADDR;

static void MT6701_ResetTransfer(void)
{
    DL_I2C_resetControllerTransfer(MT6701_I2C_INST);
    DL_I2C_flushControllerTXFIFO(MT6701_I2C_INST);
    DL_I2C_flushControllerRXFIFO(MT6701_I2C_INST);
}

static bool MT6701_WaitIdle(void)
{
    uint32_t timeout = MT6701_I2C_TIMEOUT;

    while (!(DL_I2C_getControllerStatus(MT6701_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) {
            return false;
        }
    }

    return true;
}

static bool MT6701_WaitNotBusy(void)
{
    uint32_t timeout = MT6701_I2C_TIMEOUT;

    while (DL_I2C_getControllerStatus(MT6701_I2C_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--timeout == 0U) {
            return false;
        }
    }

    return true;
}

static MT6701_Status MT6701_CheckError(void)
{
    if (DL_I2C_getControllerStatus(MT6701_I2C_INST) &
        DL_I2C_CONTROLLER_STATUS_ERROR) {
        MT6701_ResetTransfer();
        return MT6701_ERR_BUS;
    }

    return MT6701_OK;
}

static MT6701_Status MT6701_ProbeAddress(uint8_t addr)
{
    uint8_t reg = MT6701_ANGLE_REG_H;

    if (!MT6701_WaitIdle()) {
        return MT6701_ERR_TIMEOUT;
    }

    MT6701_ResetTransfer();
    DL_I2C_fillControllerTXFIFO(MT6701_I2C_INST, &reg, 1U);
    DL_I2C_startControllerTransfer(MT6701_I2C_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    delay_cycles(32);

    if (!MT6701_WaitNotBusy()) {
        MT6701_ResetTransfer();
        return MT6701_ERR_TIMEOUT;
    }

    return MT6701_CheckError();
}

static MT6701_Status MT6701_SelectAddress(void)
{
    MT6701_Status status;

    status = MT6701_ProbeAddress(mt6701_active_addr);
    if (status == MT6701_OK) {
        return MT6701_OK;
    }

    mt6701_active_addr = (mt6701_active_addr == MT6701_I2C_ADDR) ?
        MT6701_I2C_ADDR_ALT : MT6701_I2C_ADDR;

    status = MT6701_ProbeAddress(mt6701_active_addr);
    if (status == MT6701_OK) {
        return MT6701_OK;
    }

    mt6701_active_addr = MT6701_I2C_ADDR;
    return status;
}

static MT6701_Status MT6701_ReadRegs(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint32_t timeout;

    if (data == NULL || len == 0U) {
        return MT6701_ERR_NULL;
    }

    if (!MT6701_WaitIdle()) {
        return MT6701_ERR_TIMEOUT;
    }

    MT6701_ResetTransfer();

    DL_I2C_fillControllerTXFIFO(MT6701_I2C_INST, &reg, 1U);
    DL_I2C_startControllerTransferAdvanced(MT6701_I2C_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(32);

    if (!MT6701_WaitNotBusy()) {
        return MT6701_ERR_TIMEOUT;
    }

    if (MT6701_CheckError() != MT6701_OK) {
        return MT6701_ERR_BUS;
    }

    DL_I2C_startControllerTransferAdvanced(MT6701_I2C_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(32);

    for (uint8_t i = 0U; i < len; i++) {
        timeout = MT6701_I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(MT6701_I2C_INST)) {
            if (--timeout == 0U) {
                MT6701_ResetTransfer();
                return MT6701_ERR_TIMEOUT;
            }
        }
        data[i] = DL_I2C_receiveControllerData(MT6701_I2C_INST);
    }

    if (!MT6701_WaitNotBusy()) {
        MT6701_ResetTransfer();
        return MT6701_ERR_TIMEOUT;
    }

    if (MT6701_CheckError() != MT6701_OK) {
        return MT6701_ERR_BUS;
    }

    return MT6701_OK;
}

void MT6701_Init(MT6701_Data *encoder)
{
    if (encoder == NULL) {
        return;
    }

    encoder->raw = 0U;
    encoder->absolute_angle_deg = 0.0f;
    encoder->angle_deg = 0.0f;
    encoder->zero_deg = 0.0f;
    encoder->zero_valid = false;
    mt6701_active_addr = MT6701_I2C_ADDR;
}

MT6701_Status MT6701_ReadRaw(uint16_t *raw)
{
    uint8_t rx[2];
    MT6701_Status status;

    if (raw == NULL) {
        return MT6701_ERR_NULL;
    }

    status = MT6701_SelectAddress();
    if (status != MT6701_OK) {
        return status;
    }

    status = MT6701_ReadRegs(mt6701_active_addr, MT6701_ANGLE_REG_H, rx, 2U);
    if (status == MT6701_ERR_BUS) {
        mt6701_active_addr = (mt6701_active_addr == MT6701_I2C_ADDR) ?
            MT6701_I2C_ADDR_ALT : MT6701_I2C_ADDR;
        status = MT6701_ReadRegs(mt6701_active_addr, MT6701_ANGLE_REG_H, rx, 2U);
    }
    if (status != MT6701_OK) {
        return status;
    }

    *raw = (uint16_t)(((uint16_t)rx[0] << 6) | ((uint16_t)rx[1] >> 2));
    *raw &= 0x3FFFU;

    return MT6701_OK;
}

MT6701_Status MT6701_ReadAbsoluteAngle(float *angle_deg)
{
    uint16_t raw;
    MT6701_Status status;

    if (angle_deg == NULL) {
        return MT6701_ERR_NULL;
    }

    status = MT6701_ReadRaw(&raw);
    if (status != MT6701_OK) {
        return status;
    }

    *angle_deg = (float)raw * 360.0f / (float)MT6701_RAW_MAX;
    return MT6701_OK;
}

uint8_t MT6701_GetActiveAddress(void)
{
    return mt6701_active_addr;
}

MT6701_Status MT6701_Update(MT6701_Data *encoder)
{
    MT6701_Status status;

    if (encoder == NULL) {
        return MT6701_ERR_NULL;
    }

    status = MT6701_ReadRaw(&encoder->raw);
    if (status != MT6701_OK) {
        return status;
    }

    encoder->absolute_angle_deg =
        (float)encoder->raw * 360.0f / (float)MT6701_RAW_MAX;

    if (!encoder->zero_valid) {
        encoder->zero_deg = encoder->absolute_angle_deg;
        encoder->zero_valid = true;
    }

    encoder->angle_deg =
        MT6701_NormalizeAngle(encoder->absolute_angle_deg - encoder->zero_deg);

    return MT6701_OK;
}

float MT6701_NormalizeAngle(float angle_deg)
{
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}
