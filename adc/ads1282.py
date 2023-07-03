__github__ = "https://github.com/AntonVanke/MicroPython-Drivers"
__update__ = "2023-06-05 13:15"
__license__ = "GNU AFFERO GENERAL PUBLIC LICENSE Version 3"

import time
import machine
from micropython import const

HIGH = 1
LOW = 0

# 定义寄存器数量
NUM_REGISTERS = const(11)
# 必要延迟
DELAY_T_DLY = const(6)

"""
# 命令参数
"""
OPCODE_NOP = const(0x00)
OPCODE_WAKEUP = const(0x00)
OPCODE_STANDBY = const(0x03)
OPCODE_SYNC = const(0x05)
OPCODE_RESET = const(0x07)
OPCODE_RDATAC = const(0x10)
OPCODE_SDATAC = const(0x11)
OPCODE_RDATA = const(0x12)
OPCODE_RREG = const(0x20)
OPCODE_WREG = const(0x40)
OPCODE_REG_ADDR_MASK = const(0x1F)
OPCODE_REG_COUNT_MASK = const(0x1F)
OPCODE_OFSCAL = const(0x60)
OPCODE_GANCAL = const(0x61)

"""
# 寄存器定义
"""
# Register 0x00 (ID) definition
# ---------------------------------------------------------------------------------
# |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
# ---------------------------------------------------------------------------------
# |                ID[3:0]                |                   0                   |
# ---------------------------------------------------------------------------------
ID_ADDRESS = const(0x00)
ID_RESET_MASK = const(0x0F)
ID_DEFAULT = const(0x00)

ID_ID_MASK = const(0xF0)

# Register 0x01 (CONFIG0) definition
# ---------------------------------------------------------------------------------
# |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
# ---------------------------------------------------------------------------------
# |   SYNC  |    1    |            DR[2:0]          |  PHASE  |     FILTR[0:1]    |
# ---------------------------------------------------------------------------------
CONFIG0_ADDRESS = const(0x01)
CONFIG0_DEFAULT = const(0x52)

# Synchronization mode
CONFIG0_SYNC_MASK = const(0x80)
CONFIG0_SYNC_PULSE = const(0x00 << 7)  # default
CONFIG0_SYNC_CONTINUOUS = const(0x01 << 7)

# Data Rate Select
# NOTE: Specified data rates valid only in FIR filter mode with nominal 4.096 MHz FCLK
CONFIG0_DR_MASK = const(0x38)
CONFIG0_DR_250SPS = const(0x00 << 3)
CONFIG0_DR_500SPS = const(0x01 << 3)
CONFIG0_DR_1000SPS = const(0x02 << 3)  # default
CONFIG0_DR_2000SPS = const(0x03 << 3)
CONFIG0_DR_4000SPS = const(0x04 << 3)

# FIR Phase Response
CONFIG0_PHASE_MASK = const(0x04)
CONFIG0_PHASE_LINEAR = const(0x00 << 2)  # default
CONFIG0_PHASE_MINIMUM = const(0x01 << 2)

# Digital Filter Select
CONFIG0_FILTR_MASK = const(0x03)
CONFIG0_FILTR_MODMODE = const(0x00 << 0)
CONFIG0_FILTR_SINC = const(0x01 << 0)
CONFIG0_FILTR_SINC_LPF = const(0x02 << 0)  # default
CONFIG0_FILTR_SINC_LPF_HPF = const(0x03 << 0)

# Register 0x02 (CONFIG1) definition
# ---------------------------------------------------------------------------------
# |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
# ---------------------------------------------------------------------------------
# |    0    |           MUX[2:0]          |   CHOP  |           PGA[3:0]          |
# ---------------------------------------------------------------------------------
# CONFIG1 register
CONFIG1_ADDRESS = const(0x02)
CONFIG1_DEFAULT = const(0x08)
# MUX Select
CONFIG1_MUX_MASK = const(0x70)
CONFIG1_MUX_AINP1_AINN1 = const(0x00 << 4)  # default
CONFIG1_MUX_AINP2_AINN2 = const(0x01 << 4)
CONFIG1_MUX_INT_400OHM_SHORT = const(0x02 << 4)
CONFIG1_MUX_AINX1_AINX2 = const(0x03 << 4)
CONFIG1_MUX_EXT_SHORT_AINN2 = const(0x04 << 4)
# PGA Chopping Enable
CONFIG1_CHOP_MASK = const(0x08)
CONFIG1_CHOP_DISABLED = const(0x00 << 3)
CONFIG1_CHOP_ENDABLED = const(0x01 << 3)  # default
# PGA Gain Select
CONFIG1_PGA_MASK = const(0x07)
CONFIG1_PGA_1 = const(0x00 << 0)
CONFIG1_PGA_2 = const(0x01 << 0)
CONFIG1_PGA_4 = const(0x02 << 0)
CONFIG1_PGA_8 = const(0x03 << 0)
CONFIG1_PGA_16 = const(0x04 << 0)
CONFIG1_PGA_32 = const(0x05 << 0)
CONFIG1_PGA_64 = const(0x06 << 0)

# Register 0x03 (HPF0) definition
HPF0_ADDRESS = const(0x03)
HPF0_DEFAULT = const(0x32)
# Register 0x04 (HPF1) definition
HPF1_ADDRESS = const(0x04)
HPF1_DEFAULT = const(0x03)
# Register 0x05 (OFC0) definition
OFC0_ADDRESS = const(0x05)
OFC0_DEFAULT = const(0x00)
# Register 0x06 (OFC1) definition
OFC1_ADDRESS = const(0x06)
OFC1_DEFAULT = const(0x00)
# Register 0x07 (OFC2) definition
OFC2_ADDRESS = const(0x07)
OFC2_DEFAULT = const(0x00)
# Register 0x08 (FSC0) definition
FSC0_ADDRESS = const(0x08)
FSC0_DEFAULT = const(0x00)
# Register 0x09 (FSC1) definition
FSC1_ADDRESS = const(0x09)
FSC1_DEFAULT = const(0x00)
# Register 0x0A (FSC2) definition
FSC2_ADDRESS = const(0x0A)
FSC2_DEFAULT = const(0x40)


class ADS1282:
    def __init__(self, spi: machine.SPI, rst, drdy):
        self.spi = spi
        self.rst = machine.Pin(rst, machine.Pin.OUT)
        self.drdy = machine.Pin(drdy, machine.Pin.IN, machine.Pin.PULL_UP)

        # 寄存器映射
        self.register_map = [_ for _ in range(NUM_REGISTERS)]
        # 连续读模式
        self.read_continuous_mode = False
        # 是否有中断
        self.is_interrupt = False
        # ADC 采集数据
        self.adc_data = 0
        # 零点偏移数据
        self.zero_offset_value = 0

        # 重启
        self.set_reset(HIGH)
        self.toggle_reset()

        self.restore_register_defaults()

        # NOTE: 配置寄存器
        self.write_register_bits_value(CONFIG0_ADDRESS, CONFIG0_DR_MASK, CONFIG0_DR_4000SPS)

        # 自动校正
        self.auto_calibrate()

    @property
    def filter_setting(self):
        return self.get_register_value(CONFIG0_ADDRESS) & CONFIG0_FILTR_MASK

    @property
    def filter_bypassed(self):
        return self.filter_setting == CONFIG0_FILTR_MODMODE

    @property
    def filter_sinc_only(self):
        return self.filter_setting == CONFIG0_FILTR_SINC

    def restore_register_defaults(self):
        # 重置之后默认关闭连续读模式
        self.read_continuous_mode = False

        self.register_map[ID_ADDRESS] = ID_DEFAULT & ID_RESET_MASK
        self.register_map[CONFIG0_ADDRESS] = CONFIG0_DEFAULT
        self.register_map[CONFIG1_ADDRESS] = CONFIG1_DEFAULT
        self.register_map[HPF0_ADDRESS] = HPF0_DEFAULT
        self.register_map[HPF1_ADDRESS] = HPF1_DEFAULT
        self.register_map[OFC0_ADDRESS] = OFC0_DEFAULT
        self.register_map[OFC1_ADDRESS] = OFC1_DEFAULT
        self.register_map[OFC2_ADDRESS] = OFC2_DEFAULT
        self.register_map[FSC0_ADDRESS] = FSC0_DEFAULT
        self.register_map[FSC1_ADDRESS] = FSC1_DEFAULT
        self.register_map[FSC2_ADDRESS] = FSC2_DEFAULT

    def sign_extend_data(self, _data_array):
        # data_bytes is a list of uint8_t where the first element is the MSB.
        # Concatenate the four bytes into a 32-bit integer
        concatenated = (_data_array[0] << 24) | (_data_array[1] << 16) | (_data_array[2] << 8) | (_data_array[3] << 0)

        # Convert to a signed integer
        concatenated = concatenated - (1 << 32) if concatenated >= (1 << 31) else concatenated

        concatenated -= self.zero_offset_value

        if self.filter_sinc_only:
            return concatenated
        else:
            return concatenated >> 1

    def read_data(self):
        data_tx = bytearray(4)
        data_rx = bytearray(4)

        self.spi_send_receive(data_tx, data_rx)

        return self.sign_extend_data(data_rx)

    def get_register_value(self, _address):
        return self.register_map[_address]

    def set_reset(self, state):
        self.rst.value(state)

        if not state:
            self.restore_register_defaults()

    def toggle_reset(self):
        self.set_reset(LOW)
        time.sleep_us(20)
        self.set_reset(HIGH)

    def send_command(self, opcode):
        data_tx = bytearray([opcode])
        data_rx = bytearray(1)

        self.spi_send_receive(data_tx, data_rx)

        if opcode == OPCODE_RESET:
            self.restore_register_defaults()
        elif opcode == OPCODE_RDATAC:
            self.read_continuous_mode = True
        elif opcode == OPCODE_SDATAC:
            self.read_continuous_mode = False

    def write_single_register(self, _address, _data):
        data_tx = bytearray(2)
        data_rx = bytearray(2)

        data_tx[0] = OPCODE_WREG | (_address & OPCODE_REG_ADDR_MASK)

        self.spi_send_receive(data_tx, data_rx)

        data_tx = bytearray([_data])
        data_rx = bytearray([0])
        self.spi_send_receive(data_tx, data_rx)

        self.register_map[_address] = _data

    def read_single_register(self, _address):
        data_tx = bytearray(2)
        data_rx = bytearray(2)

        if self.read_continuous_mode:
            self.send_command(OPCODE_SDATAC)

        data_tx[0] = OPCODE_RREG | (_address | OPCODE_REG_ADDR_MASK)
        self.spi_send_receive(data_tx, data_rx)

        # 读数据
        data_tx = bytearray([OPCODE_NOP])
        data_rx = bytearray(1)
        self.spi_send_receive(data_tx, data_rx)
        time.sleep_us(DELAY_T_DLY)

        self.register_map[_address] = data_rx[0]

        # 返回数据
        return self.register_map[_address]

    def spi_send_receive(self, _tx, _rx):
        self.spi.write_readinto(_tx, _rx)

    def wait_for_drdy_interrupt(self, timeout_ms):
        if not self.read_continuous_mode:
            self.send_command(OPCODE_RDATAC)

        self.drdy.irq(self.interrupt_handler, self.drdy.IRQ_FALLING)

        start_time = time.ticks_ms()
        while (time.ticks_ms() - start_time < timeout_ms) and (not self.is_interrupt):
            pass

        self.drdy.irq(None)

        if self.is_interrupt:
            self.is_interrupt = False
            return True
        else:
            return False

    def interrupt_handler(self, _):
        self.is_interrupt = True
        self.adc_data = self.read_data()

    def interrupt_open(self, func):
        if not self.read_continuous_mode:
            self.send_command(OPCODE_RDATAC)
        self.drdy.irq(func, self.drdy.IRQ_FALLING)

    def interrupt_close(self):
        self.drdy.irq(None)
        self.send_command(OPCODE_SDATAC)

    # Helper
    def write_register_bits_value(self, _address, _mask, _config):
        value = self.get_register_value(_address)

        # 清除所选字节
        value &= ~_mask

        # 写入新字节
        value |= (_config & _mask)

        # 写入寄存器
        self.write_single_register(_address, value)

    def auto_calibrate(self):
        self.send_command(OPCODE_SDATAC)
        self.write_register_bits_value(CONFIG1_ADDRESS, CONFIG1_MUX_MASK, CONFIG1_MUX_INT_400OHM_SHORT)

        time.sleep_ms(DELAY_T_DLY)

        if self.wait_for_drdy_interrupt(100):
            self.zero_offset_value = self.adc_data

        self.send_command(OPCODE_SDATAC)
        time.sleep_ms(DELAY_T_DLY)
        self.write_register_bits_value(CONFIG1_ADDRESS, CONFIG1_MUX_MASK, CONFIG1_MUX_AINP1_AINN1)
        time.sleep_ms(DELAY_T_DLY)
