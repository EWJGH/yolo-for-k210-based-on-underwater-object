#####################################################################################################
# @file         sh3001.py
# @author       正点原子团队(ALIENTEK)
# @version      V1.0
# @date         2024-01-17
# @brief        六轴传感器驱动
# @license      Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
#####################################################################################################
# @attention
#
# 实验平台:正点原子 K210开发板
# 在线视频:www.yuanzige.com
# 技术论坛:www.openedv.com
# 公司网址:www.alientek.com
# 购买地址:openedv.taobao.com
#
#####################################################################################################

from machine import I2C
import time
import math

class SH3001:
    # Initialize SH3001
    def __init__(self, i2c, attitude=False):
        self._i2c = i2c
        self._buf1 = bytearray(1)

        # Find SH3001 device on I2C bus
        id = 0x00
        for self._addr in [0b0110110, 0b0110111]:
            try:
                id = self._read_byte(0x0F)
            except OSError:
                pass
            if id == 0x61:
                break
        if id != 0x61:
            raise Exception("Can't find SH3001 device!")

        # Reset SH3001 internal modules
        self.reset_internal_modules()

        # Configure SH3001 accelerometer/gyroscope/thermometer parameters
        self.acc_odr = 0
        self.acc_config(self.ACC_ODR_500HZ, self.ACC_RANGE_16G, self.ACC_FREQ_ODRX025, self.ACC_FILTER_ENABLE)
        self.gyro_config(self.GYOR_ODR_500HZ, self.GYRO_RANGE_2000DPS, self.GYRO_RANGE_2000DPS, self.GYRO_RANGE_2000DPS, self.GYRO_FREQ_00, self.GYRO_FILTER_ENABLE)
        self.room_temp = 0
        self.temp_config(self.TEMP_ODR_63HZ, self.TEMP_ENABLE)

        # Configure SH3001 working mode
        self.mode_config(self.MODE_NORMAL)

        # Read compcoef
        self.cXY = 0
        self.cXZ = 0
        self.cYX = 0
        self.cYZ = 0
        self.cZX = 0
        self.cZY = 0
        self.jX = 0
        self.jY = 0
        self.jZ = 0
        self.xMulti = 0
        self.yMulti = 0
        self.zMulti = 0
        self.paramP0 = 0
        self.read_compcoef()

        # Initialize attitude calculation
        if attitude is True:
            self.ax_avg = 0
            self.ay_avg = 0
            self.az_avg = 0
            self.gx_avg = 0
            self.gy_avg = 0
            self.gz_avg = 0
            self.acc_new_weight = 0.35
            self.acc_old_weight = 1 - self.acc_new_weight
            self.ax_last = 0
            self.ay_last = 0
            self.az_last = 0
            self.q0 = 1
            self.q1 = 0
            self.q2 = 0
            self.q3 = 0
            self.gx_ierror = 0
            self.gy_ierror = 0
            self.gz_ierror = 0
            self.delta_time = 0.005
            self.quarter_time = 0.25 * self.delta_time
            self.param_kp = 50
            self.param_ki = 0.2
            self.attitude_init()

    # Read Sh3001 register
    def _read_byte(self, memaddr):
        self._i2c.readfrom_mem_into(self._addr, memaddr, self._buf1)
        return self._buf1[0]

    # Write SH3001 register
    def _write_byte(self, memaddr, data):
        self._buf1[0] = data
        self._i2c.writeto_mem(self._addr, memaddr, self._buf1)

    # Convert two bytes to signed integer
    def _bytes_to_int(self, msb, lsb):
        if not msb & 0x80:
            return msb << 8 | lsb
        return -(((msb ^ 255) << 8) | (lsb ^ 255) + 1)

    # Reset SH3001 internal modules
    def reset_internal_modules(self):
        # Get configuration based on chip version
        reg = [0xC0, 0xD3, 0xD3, 0xD5, 0xD4, 0xBB, 0xB9, 0xBA]
        ver = self._read_byte(0xDD)
        if ver == 0x08:
            dat1 = [0x38, 0xC6, 0xC1, 0x02, 0x0C, 0x18, 0x18, 0x18]
            dat2 = [0x3D, 0xC2, 0xC2, 0x00, 0x04, 0x00, 0x00, 0x00]
        elif ver == 0x10:
            dat1 = [0x38, 0xD6, 0xD1, 0x02, 0x08, 0x18, 0x18, 0x18]
            dat2 = [0x3D, 0xD2, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00]
        elif ver == 0x20:
            dat1 = [0x38, 0x16, 0x11, 0x02, 0x08, 0x18, 0x18, 0x18]
            dat2 = [0x3E, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00]
        else:
            dat1 = [0x38, 0xD6, 0xD1, 0x02, 0x08, 0x18, 0x18, 0x18]
            dat2 = [0x3D, 0xD2, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00]

        # Device start
        self._write_byte(reg[0], dat1[0])
        self._write_byte(reg[1], dat1[1])
        time.sleep_ms(100)
        self._write_byte(reg[0], dat2[0])
        self._write_byte(reg[1], dat2[1])
        time.sleep_ms(50)

        # ADC restart
        self._write_byte(reg[2], dat1[2])
        self._write_byte(reg[3], dat1[3])
        time.sleep_ms(1)
        self._write_byte(reg[2], dat2[2])
        time.sleep_ms(1)
        self._write_byte(reg[3], dat2[3])
        time.sleep_ms(50)

        # CVA reset
        self._write_byte(reg[4], dat1[4])
        time.sleep_ms(10)
        self._write_byte(reg[4], dat2[4])
        time.sleep_ms(1)
        self._write_byte(reg[5], dat1[5])
        time.sleep_ms(10)
        self._write_byte(reg[6], dat1[6])
        time.sleep_ms(10)
        self._write_byte(reg[7], dat1[7])
        time.sleep_ms(10)
        self._write_byte(reg[5], dat2[5])
        time.sleep_ms(10)
        self._write_byte(reg[6], dat2[6])
        time.sleep_ms(10)
        self._write_byte(reg[7], dat2[7])
        time.sleep_ms(10)

        # Configure INT and INT1 pins as open-drain outputs
        self._write_byte(0x44, 0x00)

    # Accelerometer configuration parameters
    ACC_ODR_1000HZ      = (0x00 << 0)
    ACC_ODR_500HZ       = (0x01 << 0)
    ACC_ODR_250HZ       = (0x02 << 0)
    ACC_ODR_125HZ       = (0x03 << 0)
    ACC_ODR_63HZ        = (0x04 << 0)
    ACC_ODR_31HZ        = (0x05 << 0)
    ACC_ODR_16HZ        = (0x06 << 0)
    ACC_ODR_2000HZ      = (0x08 << 0)
    ACC_ODR_4000HZ      = (0x09 << 0)
    ACC_ODR_8000HZ      = (0x0A << 0)
    ACC_RANGE_16G       = (0x02 << 0)
    ACC_RANGE_8G        = (0x03 << 0)
    ACC_RANGE_4G        = (0x04 << 0)
    ACC_RANGE_2G        = (0x05 << 0)
    ACC_FREQ_ODRX040    = (0x00 << 5)
    ACC_FREQ_ODRX025    = (0x01 << 5)
    ACC_FREQ_ODRX011    = (0x02 << 5)
    ACC_FREQ_ODRX004    = (0x03 << 5)
    ACC_FREQ_ODRX002    = (0x04 << 5)
    ACC_FILTER_DISABLE  = (0x00 << 3)
    ACC_FILTER_ENABLE   = (0x01 << 3)

    # Configure SH3001 accelerometer parameters
    def acc_config(self, odr, range, freq, filter):
        # Enable accelerometer digital filter
        data = self._read_byte(0x22)
        data = data | 0x01
        self._write_byte(0x22, data)
        # Configure accelerometer ODR
        self.acc_odr = odr
        self._write_byte(0x23, odr)
        # Configure accelerometer range
        self._write_byte(0x25, range)
        # Configure accelerometer low pass filter cut-off frequency and enable or not
        data = self._read_byte(0x26)
        data = data & 0x17
        data = data | (freq | filter)
        self._write_byte(0x26, data)

    # Gyroscope configuration parameters
    GYOR_ODR_1000HZ     = (0x00 << 0)
    GYOR_ODR_500HZ      = (0x01 << 0)
    GYOR_ODR_250HZ      = (0x02 << 0)
    GYOR_ODR_125HZ      = (0x03 << 0)
    GYOR_ODR_63HZ       = (0x04 << 0)
    GYOR_ODR_31HZ       = (0x05 << 0)
    GYOR_ODR_2000HZ     = (0x08 << 0)
    GYOR_ODR_4000HZ     = (0x09 << 0)
    GYOR_ODR_8000HZ     = (0x0A << 0)
    GYOR_ODR_16000HZ    = (0x0B << 0)
    GYOR_ODR_32000HZ    = (0x0C << 0)
    GYRO_RANGE_125DPS   = (0x02 << 0)
    GYRO_RANGE_250DPS   = (0x03 << 0)
    GYRO_RANGE_500DPS   = (0x04 << 0)
    GYRO_RANGE_1000DPS  = (0x05 << 0)
    GYRO_RANGE_2000DPS  = (0x06 << 0)
    GYRO_FREQ_00        = (0x00 << 2)
    GYRO_FREQ_01        = (0x01 << 2)
    GYRO_FREQ_02        = (0x02 << 2)
    GYRO_FREQ_03        = (0x03 << 2)
    GYRO_FILTER_DISABLE = (0x00 << 4)
    GYRO_FILTER_ENABLE  = (0x01 << 4)

    # Configure SH3001 gyroscope parameters
    def gyro_config(self, odr, rangex, rangey, rangez, freq, filter):
        # Enable gyroscope digital filter
        data = self._read_byte(0x28)
        data = data | 0x01
        self._write_byte(0x28, data)
        # Configure gyroscope ODR
        self._write_byte(0x29, odr)
        # Configure gyroscope X\Y\Z range
        self._write_byte(0x8F, rangex)
        self._write_byte(0x9F, rangey)
        self._write_byte(0xAF, rangez)
        # Configure gyroscope digital LPF cut-off frequency and enable or not
        data = self._read_byte(0x2B)
        data = data & 0xE3
        data = data | (freq | filter)
        self._write_byte(0x2B, data)

    # Thermometer configuration parameters
    TEMP_ODR_500HZ  = (0x00 << 4)
    TEMP_ODR_250HZ  = (0x01 << 4)
    TEMP_ODR_125HZ  = (0x02 << 4)
    TEMP_ODR_63HZ   = (0x03 << 4)
    TEMP_DISABLE    = (0x00 << 7)
    TEMP_ENABLE     = (0x01 << 7)

    # Configure SH3001 thermometer parameters
    def temp_config(self, odr, enable):
        # Configure thermometer enable or not and ODR
        data = self._read_byte(0x20)
        data = data & 0x4F
        data = data | (odr | enable)
        self._write_byte(0x20, data)
        # Read room temperature
        self.room_temp = self._bytes_to_int(self._read_byte(0x20) & 0x0F, self._read_byte(0x21))

    # Working mode definition
    MODE_NORMAL     = 0x00
    MODE_SLEEP      = 0x01
    MODE_POWERDOWN  = 0x02
    MODE_ACC_NORMAL = 0x03

    # Configure SH3001 working mode
    def mode_config(self, mode):
        reg = [0xCF, 0x22, 0x2F, 0xCB, 0xCE, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7]
        dat = []

        # Read register data
        for addr in reg:
            data = self._read_byte(addr)
            dat.append(data)

        # Configure working mode
        if mode == self.MODE_NORMAL:
            self._write_byte(0x23, self.acc_odr)
            dat[0] = (dat[0] & 0xF8)
            dat[1] = (dat[1] & 0x7F)
            dat[2] = (dat[2] & 0xF7)
            dat[3] = (dat[3] & 0xF7)
            dat[4] = (dat[4] & 0xFE)
            dat[5] = (dat[5] & 0xFC) | 0x02
            dat[6] = (dat[6] & 0x9F)
            dat[7] = (dat[7] & 0xF9)
            for index in range(0, 8):
                self._write_byte(reg[index], dat[index])
            dat[7] = (dat[7] & 0x87)
            dat[8] = (dat[8] & 0x1F)
            dat[9] = (dat[9] & 0x03)
            for index in range(7, 10):
                self._write_byte(reg[index], dat[index])
            self.reset_internal_modules()
        elif mode == self.MODE_SLEEP:
            self.acc_odr = self._read_byte(0x23)
            self._write_byte(0x23, self.ACC_ODR_1000HZ)
            dat[0] = (dat[0] & 0xF8) | 0x07
            dat[1] = (dat[1] & 0x7F) | 0x80
            dat[2] = (dat[2] & 0xF7) | 0x08
            dat[3] = (dat[3] & 0xF7) | 0x08
            dat[4] = (dat[4] & 0xFE)
            dat[5] = (dat[5] & 0xFC) | 0x01
            dat[6] = (dat[6] & 0x9F)
            dat[7] = (dat[7] & 0xF9) | 0x06
            for index in range(0, 8):
                self._write_byte(reg[index], dat[index])
            dat[7] = (dat[7] & 0x87)
            dat[8] = (dat[8] & 0x1F)
            dat[9] = (dat[9] & 0x03)
            for index in range(7, 10):
                self._write_byte(reg[index], dat[index])
        elif mode == self.MODE_POWERDOWN:
            dat[0] = (dat[0] & 0xF8)
            dat[1] = (dat[1] & 0x7F) | 0x80
            dat[2] = (dat[2] & 0xF7) | 0x08
            dat[3] = (dat[3] & 0xF7) | 0x08
            dat[4] = (dat[4] & 0xFE)
            dat[5] = (dat[5] & 0xFC) | 0x01
            dat[6] = (dat[6] & 0x9F) | 0x60
            dat[7] = (dat[7] & 0xF9) | 0x06
            for index in range(0, 8):
                self._write_byte(reg[index], dat[index])
            dat[7] = (dat[7] & 0x87)
            dat[8] = (dat[8] & 0x1F)
            dat[9] = (dat[9] & 0x03)
            for index in range(7, 10):
                self._write_byte(reg[index], dat[index])
        elif mode == self.MODE_ACC_NORMAL:
            dat[0] = (dat[0] & 0xF8)
            dat[1] = (dat[1] & 0x7F)
            dat[2] = (dat[2] & 0xF7)
            dat[3] = (dat[3] & 0xF7)
            dat[4] = (dat[4] | 0x01)
            dat[5] = (dat[5] & 0xFC) | 0x01
            dat[6] = (dat[6] & 0x9F)
            dat[7] = (dat[7] & 0xF9) | 0x06
            for index in range(0, 8):
                self._write_byte(reg[index], dat[index])
            dat[7] = (dat[7] & 0x87) | 0x78
            dat[8] = (dat[8] & 0x1F) | 0xE0
            dat[9] = (dat[9] & 0x03) | 0xFC
            for index in range(7, 10):
                self._write_byte(reg[index], dat[index])
        else:
            pass

    # Read SH3001 compcoef
    def read_compcoef(self):
        data = self._read_byte(0x81)
        if data < 128:
            self.cYX = data
        else:
            self.cYX = data - 256
        data = self._read_byte(0x82)
        if data < 128:
            self.cZX = data
        else:
            self.cZX = data - 256
        data = self._read_byte(0x91)
        if data < 128:
            self.cXY = data
        else:
            self.cXY = data - 256
        data = self._read_byte(0x92)
        if data < 128:
            self.cZY = data
        else:
            self.cZY = data - 256
        data = self._read_byte(0xA1)
        if data < 128:
            self.cXZ = data
        else:
            self.cXZ = data - 256
        data = self._read_byte(0xA2)
        if data < 128:
            self.cYZ = data
        else:
            self.cYZ = data - 256
        self.jX = self._read_byte(0x60)
        self.jY = self._read_byte(0x68)
        self.jZ = self._read_byte(0x70)
        data = self._read_byte(0x8F)
        data = data & 0x07
        if data < 2 or data >= 7:
            self.xMulti = 1
        else:
            self.xMulti = (1 << (6 - data))
        data = self._read_byte(0x9F)
        data = data & 0x07
        if data < 2 or data >= 7:
            self.yMulti = 1
        else:
            self.yMulti = (1 << (6 - data))
        data = self._read_byte(0xAF)
        data = data & 0x07
        if data < 2 or data >= 7:
            self.zMulti = 1
        else:
            self.zMulti = (1 << (6 - data))
        data = self._read_byte(0x2E)
        self.paramP0 = data & 0x1F

    # Get SH3001 accelerometer raw data
    def get_raw_acc(self):
        ax = self._bytes_to_int(self._read_byte(0x01), self._read_byte(0x00))
        ay = self._bytes_to_int(self._read_byte(0x03), self._read_byte(0x02))
        az = self._bytes_to_int(self._read_byte(0x05), self._read_byte(0x04))
        return ax, ay, az

    # Get SH3001 accelerometer data
    def get_acc(self):
        ax_raw, ay_raw, az_raw = self.get_raw_acc()
        ax = ax_raw + (ay_raw * (self.cXY / 1024)) + (az_raw * (self.cXZ / 1024))
        ay = (ax_raw * (self.cYX / 1024)) + ay_raw + (az_raw * (self.cYZ / 1024))
        az = (ax_raw * (self.cZX / 1024)) + (ay_raw * (self.cZY / 1024)) + az_raw
        if ax > 32767:
            ax = 32767
        elif ax < -32768:
            ax = -32768
        if ay > 32767:
            ay = 32767
        elif ay < -32768:
            ay = -32768
        if az > 32767:
            az = 32767
        elif az < -32768:
            az = -32768
        return ax, ay, az

    # Get SH3001 gyroscope raw data
    def get_raw_gyro(self):
        gx = self._bytes_to_int(self._read_byte(0x07), self._read_byte(0x06))
        gy = self._bytes_to_int(self._read_byte(0x09), self._read_byte(0x08))
        gz = self._bytes_to_int(self._read_byte(0x0B), self._read_byte(0x0A))
        return gx, gy, gz

    # Get SH3001 gyroscope data
    def get_gyro(self):
        gx_raw, gy_raw, gz_raw = self.get_raw_gyro()
        paramp = self._read_byte(0x0E) & 0x1F
        gx = gx_raw - (paramp - self.paramP0) * self.jX * self.xMulti
        gy = gy_raw - (paramp - self.paramP0) * self.jY * self.yMulti
        gz = gz_raw - (paramp - self.paramP0) * self.jZ * self.zMulti
        if gx > 32767:
            gx = 32767
        elif gx < -32768:
            gx = -32768
        if gy > 32767:
            gy = 32767
        elif gy < -32768:
            gy = -32768
        if gz > 32767:
            gz = 32767
        elif gz < -32768:
            gz = -32768
        return gx, gy, gz

    # Get SH3001 thermometer data
    def get_temp(self):
        temp = self._bytes_to_int(self._read_byte(0x0D) & 0x0F, self._read_byte(0x0C))
        return (((temp - self.room_temp) / 16) + 25)

    # Initialize attitude calculation
    def attitude_init(self):
        time.sleep_ms(100)
        for times in range(250):
            time.sleep_ms(5)
            ax, ay, az = self.get_acc()
            gx, gy, gz = self.get_gyro()
            self.ax_avg = self.ax_avg + ax
            self.ay_avg = self.ay_avg + ay
            self.az_avg = self.az_avg + az
            self.gx_avg = self.gx_avg + gx
            self.gy_avg = self.gy_avg + gy
            self.gz_avg = self.gz_avg + gz
        self.ax_avg = self.ax_avg / 250
        self.ay_avg = self.ay_avg / 250
        self.az_avg = self.az_avg / 250 + 2048
        self.gx_avg = self.gx_avg / 250
        self.gy_avg = self.gy_avg / 250
        self.gz_avg = self.gz_avg / 250

    # Calculation attitude
    def get_attitude(self):
        # Get SH3001 accelerometer and gyroscope data
        ax, ay, az = self.get_acc()
        gx, gy, gz = self.get_gyro()

        # Calibration accelerometer and gyroscope data
        ax = ax - self.ax_avg
        ay = -(ay - self.ay_avg)
        az = -(az - self.az_avg)
        gx = gx - self.gx_avg
        gy = -(gy - self.gy_avg)
        gz = -(gz - self.gz_avg)

        # Filtering accelerometer and gyroscope data
        ax = ax * self.acc_new_weight + self.ax_last * self.acc_old_weight
        ay = ay * self.acc_new_weight + self.ay_last * self.acc_old_weight
        az = az * self.acc_new_weight + self.az_last * self.acc_old_weight
        self.ax_last = ax
        self.ay_last = ay
        self.az_last = az
        gx = gx * math.pi / 180 / 16.4
        gy = gy * math.pi / 180 / 16.4
        gz = gz * math.pi / 180 / 16.4

        # Normalizing accelerometer data
        norm = 1 / math.sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2))
        ax = ax * norm
        ay = ay * norm
        az = az * norm

        # Normalizing gyroscope data
        vx = 2 * (self.q1 * self.q3 - self.q0 * self.q2)
        vy = 2 * (self.q0 * self.q1 + self.q2 * self.q3)
        vz = self.q0 * self.q0 - self.q1 * self.q1 - self.q2 * self.q2 + self.q3 * self.q3
        ex = ay * vz - az * vy
        ey = az * vx - ax * vz
        ez = ax * vy - ay * vx
        self.gx_ierror = self.gx_ierror + self.delta_time * ex
        self.gy_ierror = self.gy_ierror + self.delta_time * ey
        self.gz_ierror = self.gz_ierror + self.delta_time * ez
        gx = gx + self.param_kp * ex + self.param_ki * self.gx_ierror
        gy = gy + self.param_kp * ey + self.param_ki * self.gy_ierror
        gz = gz + self.param_kp * ez + self.param_ki * self.gz_ierror

        # Compute quaternions
        delta_2 = (2 * self.quarter_time * gx) * (2 * self.quarter_time * gx) + (2 * self.quarter_time * gy) * (2 * self.quarter_time * gy) + (2 * self.quarter_time * gz) * (2 * self.quarter_time * gz)
        self.q0 = (1 - delta_2 / 8) * self.q0 + (-self.q1 * gx - self.q2 * gy - self.q3 * gz) * self.quarter_time
        self.q1 = (1 - delta_2 / 8) * self.q1 + (self.q0 * gx + self.q2 * gz - self.q3 * gy) * self.quarter_time
        self.q2 = (1 - delta_2 / 8) * self.q2 + (self.q0 * gy - self.q1 * gz + self.q3 * gx) * self.quarter_time
        self.q3 = (1 - delta_2 / 8) * self.q3 + (self.q0 * gz + self.q1 * gy - self.q2 * gx) * self.quarter_time

        # Normalizing quaternion data
        norm = 1 / math.sqrt(pow(self.q0, 2) + pow(self.q1, 2) + pow(self.q2, 2) + pow(self.q3, 2))
        self.q0 = self.q0 * norm
        self.q1 = self.q1 * norm
        self.q2 = self.q2 * norm
        self.q3 = self.q3 * norm

        pitch = -math.asin(- 2 * self.q1 * self.q3 + 2 * self.q0 * self.q2) * 190 / math.pi
        roll = math.atan2(2 * self.q2 * self.q3 + 2 * self.q0 * self.q1, - 2 * self.q1 * self.q1 - 2 * self.q2 * self.q2 + 1) * 180 / math.pi
        yaw = math.atan2(2 * self.q1 * self.q2 + 2 * self.q0 * self.q3, -2 * self.q2 * self.q2 - 2 * self.q3 * self.q3 + 1) * 180 / math.pi

        if roll > 90 or roll < -90:
            if pitch > 0:
                pitch = 180 - pitch
            if pitch < 0:
                pitch = -(180 + pitch)
        if yaw > 180:
            yaw = yaw - 360
        elif yaw < -180:
            yaw = yaw + 360

        return pitch, roll, yaw
