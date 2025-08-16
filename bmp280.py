# Micropython driver for the BMP280 temperature and pressure sensor
# This driver works with ESP8266 boards using the I2C interface.

import time

from machine import I2C, Pin

# BMP280 registers and constants
BMP280_I2C_ADDRESS = 0x76  # The datasheet says 0x76 or 0x77. The most common is 0x76.
BMP280_REG_RESET = 0xE0
BMP280_REG_CTRL_MEAS = 0xF4
BMP280_REG_CONFIG = 0xF5
BMP280_REG_CALIB00 = 0x88
BMP280_REG_PRESS_MSB = 0xF7
BMP280_REG_TEMP_MSB = 0xFA
BMP280_CHIP_ID = 0xD0
BMP280_REG_STATUS = 0xF3

# Oversampling settings for ctrl_meas register
# osrs_t: oversampling of temperature data
OSRS_T_X1 = 0b001
OSRS_T_X2 = 0b010
OSRS_T_X4 = 0b011
OSRS_T_X8 = 0b100
OSRS_T_X16 = 0b101
# osrs_p: oversampling of pressure data
OSRS_P_X1 = 0b001
OSRS_P_X2 = 0b010
OSRS_P_X4 = 0b011
OSRS_P_X8 = 0b100
OSRS_P_X16 = 0b101

# Power mode settings for ctrl_meas register
MODE_SLEEP = 0b00
MODE_FORCED = 0b01
MODE_NORMAL = 0b11

# IIR filter settings for config register
FILTER_OFF = 0b000
FILTER_X2 = 0b001
FILTER_X4 = 0b010
FILTER_X8 = 0b011
FILTER_X16 = 0b100

# Standby time settings for config register
T_SB_0_5 = 0b000
T_SB_62_5 = 0b001
T_SB_125 = 0b010
T_SB_250 = 0b011
T_SB_500 = 0b100
T_SB_1000 = 0b101
T_SB_2000 = 0b110
T_SB_4000 = 0b111


class BMP280:
    """
    MicroPython driver for the Bosch BMP280 temperature and pressure sensor.
    """

    def __init__(self, i2c, address=BMP280_I2C_ADDRESS):
        """
        Initializes the BMP280 sensor.
        :param i2c: An initialized I2C object (e.g., from machine.I2C).
        :param address: The I2C address of the BMP280 sensor.
        """
        self.i2c = i2c
        self.address = address
        self.dig_T1 = 0
        self.dig_T2 = 0
        self.dig_T3 = 0
        self.dig_P1 = 0
        self.dig_P2 = 0
        self.dig_P3 = 0
        self.dig_P4 = 0
        self.dig_P5 = 0
        self.dig_P6 = 0
        self.dig_P7 = 0
        self.dig_P8 = 0
        self.dig_P9 = 0
        self.t_fine = 0.0

        # Check if the sensor is present and has the correct chip ID
        chip_id = self._read_reg_u8(BMP280_CHIP_ID)
        if chip_id != 0x58:
            raise RuntimeError("BMP280 not found, check wiring or address.")

        self.read_calibration_data()

    def _write_reg(self, reg, value):
        """
        Write a single byte to a register.
        """
        self.i2c.writeto(self.address, bytes([reg, value]))

    def _read_reg_u8(self, reg):
        """
        Read a single unsigned byte from a register.
        """
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def _read_reg_s16(self, reg):
        """
        Read a signed 16-bit value from a register.
        """
        val = self.i2c.readfrom_mem(self.address, reg, 2)
        return int.from_bytes(val, "little", True)

    def _read_reg_u16(self, reg):
        """
        Read an unsigned 16-bit value from a register.
        """
        val = self.i2c.readfrom_mem(self.address, reg, 2)
        return int.from_bytes(val, "little", False)

    def read_calibration_data(self):
        """
        Reads the factory calibration data from the sensor's EEPROM.
        This data is used to compensate the raw sensor readings.
        """
        self.dig_T1 = self._read_reg_u16(0x88)
        self.dig_T2 = self._read_reg_s16(0x8A)
        self.dig_T3 = self._read_reg_s16(0x8C)
        self.dig_P1 = self._read_reg_u16(0x8E)
        self.dig_P2 = self._read_reg_s16(0x90)
        self.dig_P3 = self._read_reg_s16(0x92)
        self.dig_P4 = self._read_reg_s16(0x94)
        self.dig_P5 = self._read_reg_u16(0x96)
        self.dig_P6 = self._read_reg_s16(0x98)
        self.dig_P7 = self._read_reg_s16(0x9A)
        self.dig_P8 = self._read_reg_s16(0x9C)
        self.dig_P9 = self._read_reg_s16(0x9E)

    def set_config(self, t_sb, filter_coeff):
        """
        Sets the sensor configuration.
        :param t_sb: Standby time setting. Use constants like T_SB_1000.
        :param filter_coeff: IIR filter coefficient. Use constants like FILTER_X16.
        """
        config = (t_sb << 5) | (filter_coeff << 2)
        self._write_reg(BMP280_REG_CONFIG, config)

    def set_mode(self, osrs_t, osrs_p, mode):
        """
        Sets the measurement control register.
        :param osrs_t: Temperature oversampling setting. Use constants like OSRS_T_X16.
        :param osrs_p: Pressure oversampling setting. Use constants like OSRS_P_X16.
        :param mode: Power mode setting. Use constants like MODE_NORMAL.
        """
        ctrl_meas = (osrs_t << 5) | (osrs_p << 2) | mode
        self._write_reg(BMP280_REG_CTRL_MEAS, ctrl_meas)
        # Wait for the measurement to complete if in forced mode
        if mode == MODE_FORCED:
            while self._read_reg_u8(BMP280_REG_STATUS) & 0x08:
                time.sleep_ms(10)

    def get_raw_temp_and_pressure(self):
        """
        Reads the raw temperature and pressure data from the sensor.
        """
        # Read the raw temperature data (20-bit)
        temp_msb = self._read_reg_u8(BMP280_REG_TEMP_MSB)
        temp_lsb = self._read_reg_u8(BMP280_REG_TEMP_MSB + 1)
        temp_xlsb = self._read_reg_u8(BMP280_REG_TEMP_MSB + 2)
        raw_temp = ((temp_msb << 16) | (temp_lsb << 8) | temp_xlsb) >> 4

        # Read the raw pressure data (20-bit)
        press_msb = self._read_reg_u8(BMP280_REG_PRESS_MSB)
        press_lsb = self._read_reg_u8(BMP280_REG_PRESS_MSB + 1)
        press_xlsb = self._read_reg_u8(BMP280_REG_PRESS_MSB + 2)
        raw_press = ((press_msb << 16) | (press_lsb << 8) | press_xlsb) >> 4

        return raw_temp, raw_press

    def compensate_temperature(self, adc_T):
        """
        Compensates the raw temperature reading to a float value in degrees Celsius.
        Uses the compensation formula from the BMP280 datasheet.
        """
        var1 = ((adc_T / 16384.0) - (self.dig_T1 / 1024.0)) * self.dig_T2
        var2 = (
            ((adc_T / 131072.0) - (self.dig_T1 / 8192.0))
            * ((adc_T / 131072.0) - (self.dig_T1 / 8192.0))
        ) * self.dig_T3
        self.t_fine = var1 + var2
        temp = self.t_fine / 5120.0
        return temp

    def compensate_pressure(self, adc_P):
        """
        Compensates the raw pressure reading to a float value in Pascals.
        Uses the compensation formula from the BMP280 datasheet.
        """
        var1 = (self.t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = (var2 / 4.0) + (self.dig_P4 * 65536.0)
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        if var1 == 0:
            return 0  # Avoid division by zero
        pressure = 1048576.0 - adc_P
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1
        var1 = self.dig_P9 * pressure * pressure / 2147483648.0
        var2 = pressure * self.dig_P8 / 32768.0
        pressure = pressure + (var1 + var2 + self.dig_P7) / 16.0
        return pressure

    def get_values(self):
        """
        Reads and returns compensated temperature and pressure values.
        Returns a tuple: (temperature_celsius, pressure_hPa).
        """
        # Set to forced mode to take a measurement
        self.set_mode(OSRS_T_X16, OSRS_P_X16, MODE_FORCED)

        # Read raw data and compensate
        raw_temp, raw_press = self.get_raw_temp_and_pressure()

        # Compensate and convert to desired units
        temp_celsius = self.compensate_temperature(raw_temp)
        pressure_pa = self.compensate_pressure(raw_press)
        pressure_hPa = pressure_pa / 100

        return temp_celsius, pressure_hPa
