#include "drivers/icm42688.h"

#define ICM42688_REG_DEVICE_CONFIG (0x11U)//对这个寄存器写0x01，等20ms重启干净，又称软复位
#define ICM42688_REG_ACCEL_DATA_X1 (0x1FU)//读数据的第一个寄存器地址
#define ICM42688_REG_PWR_MGMT0 (0x4EU)//对这个寄存器写0x0F，等20ms，进入工作模式
#define ICM42688_REG_GYRO_CONFIG0 (0x4FU)//设置陀螺仪来测角速度的量程参数
#define ICM42688_REG_ACCEL_CONFIG0 (0x50U)//设置加速度计来测加速度的量程参数
#define ICM42688_REG_WHO_AM_I (0x75U)//身份验证寄存器的地址
#define ICM42688_WHO_AM_I_VALUE (0x47U)//每颗icm42688这个寄存器都返回固定的id，用于确定spi有没有接通
#define ICM42688_READ_FLAG (0x80U)
//区分读写的标志位，具体实现是按位或| (按位与是&)
//0x80 对应1000 0000
//0x7F 对应0111 1111

//读取的数字和dps：获取角速度计的数值之后对它除以灵敏度得到DPS

//dps：度每秒：通过查表可以知道对应量程的灵敏度，比如36.8，说明36.8个原始计数对应1度每秒
//原始数据除以灵敏度得到的数字就是角速度

static int16_t Icm42688_ReadS16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static bool Icm42688_PortValid(const Icm42688 *imu)
{
    return (imu != 0) && (imu->port.transfer != 0) &&
           (imu->port.chip_select != 0) && (imu->port.delay_ms != 0);
}

static void Icm42688_WriteRegister(Icm42688 *imu, uint8_t reg, uint8_t value)
{
    imu->port.chip_select(true, imu->port.context);
    (void)imu->port.transfer((uint8_t)(reg & 0x7FU), imu->port.context);
    (void)imu->port.transfer(value, imu->port.context);
    imu->port.chip_select(false, imu->port.context);
}//写寄存器：对象，寄存器地址，值

static void Icm42688_ReadRegisters(Icm42688 *imu,
                                   uint8_t reg,
                                   uint8_t *data,
                                   uint8_t length)
{
    imu->port.chip_select(true, imu->port.context);
    (void)imu->port.transfer((uint8_t)(reg | ICM42688_READ_FLAG),
                             imu->port.context);
    for (uint8_t i = 0U; i < length; i++) {
        data[i] = imu->port.transfer(0U, imu->port.context);
    }
    imu->port.chip_select(false, imu->port.context);
}//读寄存器：对象，寄存器地址，数据缓冲区，数据长度

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
    Icm42688_ReadRegisters(imu, ICM42688_REG_WHO_AM_I, &imu->who_am_i, 1U);
    if (imu->who_am_i != ICM42688_WHO_AM_I_VALUE) {
        imu->read_errors++;
        return false;
    }

    Icm42688_WriteRegister(imu, ICM42688_REG_DEVICE_CONFIG, 0x01U);
    imu->port.delay_ms(20U, imu->port.context);
    Icm42688_WriteRegister(imu, ICM42688_REG_PWR_MGMT0, 0x0FU);
    imu->port.delay_ms(20U, imu->port.context);

    /* Gyro: +/-1000 dps, 1 kHz. Accel: +/-16 g, 1 kHz. */
    Icm42688_WriteRegister(imu, ICM42688_REG_GYRO_CONFIG0, 0x26U);
    Icm42688_WriteRegister(imu, ICM42688_REG_ACCEL_CONFIG0, 0x06U);
    imu->port.delay_ms(10U, imu->port.context);
    imu->initialized = true;
    return true;
}

bool Icm42688_ReadSample(Icm42688 *imu,
                         uint32_t now_ms,
                         Icm42688Sample *sample)
{
    uint8_t data[12];

    if ((imu == 0) || (sample == 0) || !imu->initialized ||
        !Icm42688_PortValid(imu)) {
        return false;
    }
    Icm42688_ReadRegisters(imu, ICM42688_REG_ACCEL_DATA_X1, data, sizeof(data));
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
