__github__ = "https://github.com/AntonVanke/MicroPython-Drivers"
__update__ = "2023-06-17 10:47"
__license__ = "GNU AFFERO GENERAL PUBLIC LICENSE Version 3"

import machine
import time
from micropython import const

# IIC 设备地址
RTC_DEVICE_ADDR = const(0x32)

# 寄存器
RTC_SECOND_ADDRESS = const(0x00)
RTC_MINUTE_ADDRESS = const(0x01)
RTC_HOUR_ADDRESS = const(0x02)
RTC_WEEKDAY_ADDRESS = const(0x03)
RTC_DAY_ADDRESS = const(0x04)
RTC_MONTH_ADDRESS = const(0x05)
RTC_YEAR_ADDRESS = const(0x06)
# 定义时间报警寄存器
RTC_ALARM_SECOND_ADDRESS = const(0x07)
RTC_ALARM_MINUTE_ADDRESS = const(0x08)
RTC_ALARM_HOUR_ADDRESS = const(0x09)
RTC_ALARM_WEEKDAY_ADDRESS = const(0x0A)
RTC_ALARM_DAY_ADDRESS = const(0x0B)
RTC_ALARM_MONTH_ADDRESS = const(0x0C)
RTC_ALARM_YEAR_ADDRESS = const(0x0D)
RTC_ALARM_ALLOW_ADDRESS = const(0x0E)
# 定义控制寄存器
# 控制寄存器
RTC_CTR1_ADDRESS = const(0x0F)
RTC_CTR2_ADDRESS = const(0x10)
# 频率中断
RTC_CTR2_INTFE_MASK = const(0x01)
# 报警中断
RTC_CTR2_INTAE_MASK = const(0x02)
# 倒计时中断
RTC_CTR2_INTDE_MASK = const(0x04)
RTC_CTR3_ADDRESS = const(0x11)
# 25℃ TTF ? (ReadOnly)
RTC_TTF_ADDRESS = const(0x12)
# 倒计时定时器
RTC_COUNTDOWN_TIMER1_ADDRESS = const(0x13)
RTC_COUNTDOWN_TIMER2_ADDRESS = const(0x14)
RTC_COUNTDOWN_TIMER3_ADDRESS = const(0x15)
# 温度寄存器
RTC_TEMP_ADDRESS = const(0x16)
# IIC 控制寄存器
RTC_AGTC_ADDRESS = const(0x17)
# 充电寄存器
RTC_CHARGE_ADDRESS = const(0x18)
# 扩展控制寄存器
RTC_CTR4_ADDRESS = const(0x19)
RTC_CTR5_ADDRESS = const(0x1A)
# 电池电量
RTC_BAT_VAL_ADDRESS = const(0x1B)
# 低温报警值
RTC_TEMP_AL_ADDRESS = const(0x1C)
# 高温报警值
RTC_TEMP_AH_ADDRESS = const(0x1D)
# 历史最低温度
RTC_TEMP_HIS_L_ADDRESS = const(0x1E)
# 历史最高温度
RTC_TEMP_HIS_H_ADDRESS = const(0x1F)

# 8-byte chip identification code, including production date, internal batch number, internal serial number, etc.
RTC_CHIPID_ADDRESS = 0x72

# Frequency Interrupt NOTE: Except for 1 second, the frequency interrupts output by the INT pin are obtained by
# shaping and dividing the 32768HZ crystal oscillator circuit; the 1-second frequency interrupt refers to a square
# wave with a low level of 500ms and a high level of 500ms, and its falling edge of the low level is synchronized
# with the second carry; when the internal digital adjustment register works during temperature compensation,
# the duration of 1 second and 1Hz are different.
RTC_F0HZ = 0b0000
RTC_F4096HZ = 0b0010
RTC_F1024HZ = 0b0011
RTC_F64HZ = 0b0100
RTC_F32HZ = 0b0101
RTC_F16HZ = 0b0110
RTC_F8HZ = 0b0111
RTC_F4HZ = 0b1000
RTC_F2HZ = 0b1001
RTC_F1HZ = 0b1010
RTC_F1_2HZ = 0b1011
RTC_F1_4HZ = 0b1100
RTC_F1_8HZ = 0b1101
RTC_F1_16HZ = 0b1110
RTC_F1S = 0b1111


class SD3078:
    def __init__(self, scl, sda, irq, freq=2_000_000, addr=RTC_DEVICE_ADDR):
        self.addr = addr

        self.scl = machine.Pin(scl)
        self.sda = machine.Pin(sda)
        self.irq = machine.Pin(irq, machine.Pin.IN, machine.Pin.PULL_UP)
        self.i2c = machine.I2C(scl=self.scl, sda=self.sda, freq=freq)

    def register_lock(self):
        buf = bytearray([RTC_CTR1_ADDRESS, 0, 0])
        self.i2c.writeto(self.addr, buf)

    def register_unlock(self):
        buf = bytearray([RTC_CTR2_ADDRESS, 0x80])
        self.i2c.writeto(self.addr, buf)

        buf = bytearray([RTC_CTR1_ADDRESS, 0x84])
        self.i2c.writeto(self.addr, buf)

    def _write_datetime(self, tm_sec, tm_min, tm_hour, tm_wday, tm_mday, tm_mon, tm_year):
        self.register_unlock()

        buf = bytearray(
            [RTC_SECOND_ADDRESS, tm_sec, tm_min, tm_hour, tm_wday, tm_mday, tm_mon, tm_year])
        self.i2c.writeto(RTC_DEVICE_ADDR, buf)
        self.register_lock()

    def _read_datetime(self):
        self.i2c.writeto(RTC_DEVICE_ADDR, bytearray([RTC_SECOND_ADDRESS]))

        buf = self.i2c.readfrom(RTC_DEVICE_ADDR, 7)
        return buf

    def get_datetime(self):
        # time_bcd_array: (second, minute, hour, weekday, day, month, year)
        time_bcd_array = self._read_datetime()

        # time_data: (year, month, mday, hour, minute, second, weekday, yearday)
        time_data = [0 for _ in range(8)]
        time_data[0] = (time_bcd_array[6] >> 4) * 10 + (time_bcd_array[6] & 0x0F) + 2000
        time_data[1] = (time_bcd_array[5] >> 4) * 10 + (time_bcd_array[5] & 0x0F)
        time_data[2] = (time_bcd_array[4] >> 4) * 10 + (time_bcd_array[4] & 0x0F)
        time_data[3] = (time_bcd_array[2] >> 4) * 10 + (time_bcd_array[2] & 0x0F)
        time_data[4] = (time_bcd_array[1] >> 4) * 10 + (time_bcd_array[1] & 0x0F)
        time_data[5] = (time_bcd_array[0] >> 4) * 10 + (time_bcd_array[0] & 0x0F)
        time_data[6] = (time_bcd_array[3] & 0xFF)
        time_data[7] = 0  # 未知

        return time_data

    def set_datetime(self, time_data):
        # tm_sec, tm_min, tm_hour, tm_wday, tm_mday, tm_mon, tm_year
        tm_sec = ((time_data[5] // 10) << 4) + (time_data[5] % 10)
        tm_min = ((time_data[4] // 10) << 4) + (time_data[4] % 10)
        tm_hour = ((time_data[3] // 10) << 4) + (time_data[3] % 10)
        tm_wday = time_data[6] & 0x0F
        tm_mday = ((time_data[2] // 10) << 4) + (time_data[2] % 10)
        tm_mon = ((time_data[1] // 10) << 4) + (time_data[1] % 10)
        tm_year = (((time_data[0] - 2000) // 10) << 4) + ((time_data[0] - 2000) % 10)
        self._write_datetime(tm_sec, tm_min, tm_hour, tm_wday, tm_mday, tm_mon, tm_year)

    def write_bytes(self, _address: int, _data: bytearray):
        self.register_unlock()
        self.i2c.writeto_mem(RTC_DEVICE_ADDR, _address, _data)
        self.register_lock()

    def read_bytes(self, _address, length):
        return self.i2c.readfrom_mem(RTC_DEVICE_ADDR, _address, length)

    def clear_interrupt(self, interrupt_enable):
        _buffer = bytearray([0x80 & (~interrupt_enable)])
        self.write_bytes(RTC_CTR2_ADDRESS, _buffer)

    def set_interrupt(self, freq):
        _buffer = bytearray([0xA1, freq])
        self.write_bytes(RTC_CTR2_ADDRESS, _buffer)

    def get_temp(self):
        return self.read_bytes(RTC_TEMP_ADDRESS, 1)

    def get_chipID(self):
        return self.read_bytes(RTC_CHIPID_ADDRESS, 8)

    def interrupt_open(self, func):
        self.irq.irq(func, self.irq.IRQ_FALLING)

    def interrupt_close(self):
        self.irq.irq(None)
    # def sync_time(self):
    #     self.get_datetime()
    #     # （年、月、日、工作日、小时、分钟、秒、亚秒）
    #     machine.RTC().datetime()
