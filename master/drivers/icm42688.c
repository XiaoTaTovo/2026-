#include "drivers/icm42688.h"

#define ICM42688_REG_DEVICE_CONFIG (0x11U)
#define ICM42688_REG_ACCEL_DATA_X1 (0x1FU)
#define ICM42688_REG_PWR_MGMT0 (0x4EU)
#define ICM42688_REG_GYRO_CONFIG0 (0x4FU)
#define ICM42688_REG_ACCEL_CONFIG0 (0x50U)
#define ICM42688_REG_WHO_AM_I (0x75U)
#define ICM42688_WHO_AM_I_VALUE (0x47U)
#define ICM42688_READ_FLAG (0x80U)

static int16_t Icm42688_ReadS16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static bool Icm42688_PortValid(const Icm42688 *imu)
{
    return (imu != 0) && (imu->port.transfer != 0) &&
           (imu->port.chip_select != 0) && (imu->port.delay_ms != 0);
}

static bool Icm42688_WriteRegister(Icm42688 *imu,
                                   uint8_t reg,
                                   uint8_t value)
{
    uint8_t received = 0U;
    bool ok;

    imu->port.chip_select(true, imu->port.context);
    ok = imu->port.transfer((uint8_t)(reg & 0x7FU), &received,
                            imu->port.context);
    if (ok) {
        ok = imu->port.transfer(value, &received, imu->port.context);
    }
    imu->port.chip_select(false, imu->port.context);
    return ok;
}

static bool Icm42688_ReadRegisters(Icm42688 *imu,
                                   uint8_t reg,
                                   uint8_t *data,
                                   uint8_t length)
{
    uint8_t received = 0U;
    bool ok;

    if ((data == 0) || (length == 0U)) {
        return false;
    }
    imu->port.chip_select(true, imu->port.context);
    ok = imu->port.transfer((uint8_t)(reg | ICM42688_READ_FLAG),
                            &received, imu->port.context);
    for (uint8_t i = 0U; ok && (i < length); i++) {
        ok = imu->port.transfer(0U, &data[i], imu->port.context);
    }
    imu->port.chip_select(false, imu->port.context);
    return ok;
}

void Icm42688_InitObject(Icm42688 *imu, const Icm42688Port *port)
{
    if ((imu == 0) || (port == 0)) {
        return;
    }
    *imu = (Icm42688){0};
    imu->port = *port;
    imu->gyro_lsb_per_dps = 32.8f;
}

bool Icm42688_Initialize(Icm42688 *imu)
{
    if (!Icm42688_PortValid(imu)) {
        return false;
    }
    imu->initialized = false;
    if (!Icm42688_ReadRegisters(imu, ICM42688_REG_WHO_AM_I,
                                &imu->who_am_i, 1U)) {
        imu->read_errors++;
        return false;
    }
    if (imu->who_am_i != ICM42688_WHO_AM_I_VALUE) {
        imu->read_errors++;
        return false;
    }

    if (!Icm42688_WriteRegister(imu, ICM42688_REG_DEVICE_CONFIG, 0x01U)) {
        imu->read_errors++;
        return false;
    }
    imu->port.delay_ms(20U, imu->port.context);
    if (!Icm42688_WriteRegister(imu, ICM42688_REG_PWR_MGMT0, 0x0FU)) {
        imu->read_errors++;
        return false;
    }
    imu->port.delay_ms(20U, imu->port.context);

    /* Gyro: +/-1000 dps, 1 kHz. Accel: +/-16 g, 1 kHz. */
    if (!Icm42688_WriteRegister(imu, ICM42688_REG_GYRO_CONFIG0, 0x26U) ||
        !Icm42688_WriteRegister(imu, ICM42688_REG_ACCEL_CONFIG0, 0x06U)) {
        imu->read_errors++;
        return false;
    }
    imu->port.delay_ms(10U, imu->port.context);
    imu->initialized = true;
    return true;
}

bool Icm42688_ReadSample(Icm42688 *imu,
                         uint32_t now_ms,
                         Icm42688Sample *sample)
{
    uint8_t data[12];

    if (sample != 0) {
        *sample = (Icm42688Sample){0};
    }
    if ((imu == 0) || (sample == 0) || !imu->initialized ||
        !Icm42688_PortValid(imu)) {
        return false;
    }
    if (!Icm42688_ReadRegisters(imu, ICM42688_REG_ACCEL_DATA_X1, data,
                                (uint8_t)sizeof(data))) {
        imu->read_errors++;
        return false;
    }
    sample->accel_x = Icm42688_ReadS16(&data[0]);
    sample->accel_y = Icm42688_ReadS16(&data[2]);
    sample->accel_z = Icm42688_ReadS16(&data[4]);
    sample->gyro_x = Icm42688_ReadS16(&data[6]);
    sample->gyro_y = Icm42688_ReadS16(&data[8]);
    sample->gyro_z = Icm42688_ReadS16(&data[10]);
    sample->timestamp_ms = now_ms;
    sample->valid = true;
    return true;
}
