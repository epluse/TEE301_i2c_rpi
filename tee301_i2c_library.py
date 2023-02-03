# -*- coding: utf-8 -*-
"""
Read functions for measurement values of the TEE301 Sensor via I2c interface.

Copyright 2023 E+E Elektronik Ges.m.b.H.

DDisclaimer:
This application example is non-binding and does not claim to be complete with
regard to configuration and equipment as well as all eventualities. The
application example is intended to provide assistance with the TEE301 sensor
module design-in and is provided "as is".You yourself are responsible for the
proper operation of the products described. This application example does not
release you from the obligation to handle the product safely during
application, installation, operation and maintenance. By using this application
example, you acknowledge that we cannot be held liable for any damage beyond
the liability regulations described.

We reserve the right to make changes to this application example at any time
without notice. In case of discrepancies between the suggestions in this
application example and other E+E publications, such as catalogues, the content
of the other documentation takes precedence. We assume no liability for
the information contained in this document.
"""


# pylint: disable=E0401
from smbus2 import SMBus, i2c_msg
# pylint: enable=E0401
CRC8_ONEWIRE_POLY = 0x31
CRC8_ONEWIRE_START = 0xFF
TEE301_COMMAND_READ_SINGLE_SHOT_HIGH_EN = 0x2C06 #EN = clock stretching enabled
TEE301_COMMAND_READ_SINGLE_SHOT_MEDIUM_EN = 0x2C0D #EN = clock stretching enabled
TEE301_COMMAND_READ_SINGLE_SHOT_LOW_EN = 0x2C10 #EN = clock stretching enabled
TEE301_COMMAND_READ_SINGLE_SHOT_HIGH_DIS = 0x2C00 #DIS = clock stretching disabled
TEE301_COMMAND_READ_SINGLE_SHOT_MEDIUM_DIS = 0x2C0B #DIS = clock stretching disabled
TEE301_COMMAND_READ_SINGLE_SHOT_LOW_DIS = 0x2C16 #DIS = clock stretching disabled
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_0_5_HIGH = 0x2032
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MEDIUM = 0x2024
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_0_5_LOW = 0x202F
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_1_HIGH = 0x2130
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_1_MEDIUM = 0x2126
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_1_LOW = 0x212D
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_2_HIGH = 0x2236
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_2_MEDIUM = 0x2220
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_2_LOW = 0x222B
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_4_HIGH = 0x2334
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_4_MEDIUM = 0x2322
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_4_LOW = 0x2329
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_10_HIGH = 0x2737
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_10_MEDIUM = 0x2721
TEE301_COMMAND_START_PERIODIC_MEASUREMENT_10_LOW = 0x272A
TEE301_COMMAND_READ_PERIODIC_MEASUREMENT = 0xE000
TEE301_COMMAND_CLEAR_REGISTER = 0x3041
TEE301_COMMAND_READ_REGISTER = 0xF32D
TEE301_COMMAND_END_PERIODIC_MEASUREMENT = 0x3093
TEE301_COMMAND_SOFT_RESET = 0x30A2
TEE301_COMMAND_HEATER_ON = 0x306D
TEE301_COMMAND_HEATER_OFF = 0x3066
TEE301_COMMAND_READ_IDENTIFICATION = 0x7029


def get_status_string(status_code):
    """Return string from status_code."""
    status_string = {
        0: "Success",
        1: "Not acknowledge error",
        2: "Checksum error",
        3: "Measurement error",
    }

    if status_code < len(status_string):
        return status_string[status_code]
    return "Unknown error"


def calc_crc8(buf, start, end):
    ''' calculate crc8 checksum  '''
    crc_val = CRC8_ONEWIRE_START
    for j in range(start, end):
        cur_val = buf[j]
        for _ in range(8):
            if ((crc_val ^ cur_val) & 0x80) != 0:
                crc_val = (crc_val << 1) ^ CRC8_ONEWIRE_POLY
            else:
                crc_val = crc_val << 1
            cur_val = cur_val << 1
    crc_val &= 0xFF
    return crc_val


class TEE301():
    """Implements communication with TEE301 over i2c with a specific address."""

    def __init__(self, i2c_address):
        self.i2c_address = i2c_address

    def get_single_shot_temp(self, repeatability):  # repeatability: 0 = low, 1 = medium, 2 = high;
        """Let the sensor take a measurement and return the temperature values."""
        if repeatability == 0:
            i2c_response = self.wire_write_read(
                [(TEE301_COMMAND_READ_SINGLE_SHOT_LOW_EN >> 8),
                 (TEE301_COMMAND_READ_SINGLE_SHOT_LOW_EN & 0xFF)], 6)
        elif repeatability == 1:
            i2c_response = self.wire_write_read(
                [(TEE301_COMMAND_READ_SINGLE_SHOT_MEDIUM_EN >> 8),
                 (TEE301_COMMAND_READ_SINGLE_SHOT_MEDIUM_EN & 0xFF)], 6)
        else:
            i2c_response = self.wire_write_read(
                [(TEE301_COMMAND_READ_SINGLE_SHOT_HIGH_EN >> 8),
                 (TEE301_COMMAND_READ_SINGLE_SHOT_HIGH_EN & 0xFF)], 6)

        if i2c_response[2] == calc_crc8(i2c_response, 0, 2):
            temperature = -45 + 175 * ((float)(i2c_response[0]) * 256 + i2c_response[1]) / 65535
            return temperature
        else:
            raise Warning(get_status_string(2))

    def get_single_shot_temp_clock_stretching_disabled(self, repeatability):  # repeatability: 0 = low, 1 = medium, 2 = high;
        """Let the sensor take a measurement and return the temperature values."""
        if repeatability == 0:
            i2c_response = self.wire_write_read(
                [(TEE301_COMMAND_READ_SINGLE_SHOT_LOW_DIS >> 8),
                 (TEE301_COMMAND_READ_SINGLE_SHOT_LOW_DIS & 0xFF)], 6)
        elif repeatability == 1:
            i2c_response = self.wire_write_read(
                [(TEE301_COMMAND_READ_SINGLE_SHOT_MEDIUM_DIS >> 8),
                 (TEE301_COMMAND_READ_SINGLE_SHOT_MEDIUM_DIS & 0xFF)], 6)
        else:
            i2c_response = self.wire_write_read(
                [(TEE301_COMMAND_READ_SINGLE_SHOT_HIGH_DIS >> 8),
                 (TEE301_COMMAND_READ_SINGLE_SHOT_HIGH_DIS & 0xFF)], 6)

        if i2c_response[2] == calc_crc8(i2c_response, 0, 2):
            temperature = -45 + 175 * ((float)(i2c_response[0]) * 256 + i2c_response[1]) / 65535
            return temperature
        else:
            raise Warning(get_status_string(2))

    def get_periodic_measurement_temp(self):
        """Get the last measurement from the periodic measurement for temperature"""
        i2c_response = self.wire_write_read(
            [(TEE301_COMMAND_READ_PERIODIC_MEASUREMENT >> 8),
             (TEE301_COMMAND_READ_PERIODIC_MEASUREMENT & 0xFF)], 6)
        if i2c_response[2] == calc_crc8(i2c_response, 0, 2):
            temperature = -45 + 175 * ((float)(i2c_response[0]) * 256 + i2c_response[1]) / 65535
            return temperature
        else:
            raise Warning(get_status_string(2))



    def start_periodic_measurement(self, measurement_per_seconds, repeatability): # measurementPerSeconds: 0 = 0.5 mps, 1 = 1mps, 2 = 2mps, 3 = 4mps, 4 = 10mps;  repeatability: 0 = low, 1 = medium, 2 = high;
        """starts the periodic measurement"""
        if measurement_per_seconds == 0:
            if repeatability == 0:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_0_5_LOW >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_0_5_LOW & 0xFF)])
            elif repeatability == 1:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MEDIUM >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MEDIUM & 0xFF)])
            else:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_0_5_HIGH >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_0_5_HIGH & 0xFF)])
        if measurement_per_seconds == 1:
            if repeatability == 0:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_1_LOW >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_1_LOW & 0xFF)])
            elif repeatability == 1:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_1_MEDIUM >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_1_MEDIUM & 0xFF)])
            else:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_1_HIGH >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_1_HIGH & 0xFF)])
        if measurement_per_seconds == 2:
            if repeatability == 0:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_2_LOW >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_2_LOW & 0xFF)])
            elif repeatability == 1:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_2_MEDIUM >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_2_MEDIUM & 0xFF)])
            else:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_2_HIGH >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_2_HIGH & 0xFF)])
        if measurement_per_seconds == 3:
            if repeatability == 0:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_4_LOW >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_4_LOW & 0xFF)])
            elif repeatability == 1:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_4_MEDIUM >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_4_MEDIUM & 0xFF)])
            else:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_4_HIGH >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_4_HIGH & 0xFF)])
        if measurement_per_seconds == 4:
            if repeatability == 0:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_10_LOW >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_10_LOW & 0xFF)])
            elif repeatability == 1:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_10_MEDIUM >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_10_MEDIUM & 0xFF)])
            else:
                self.wire_write(
                    [(TEE301_COMMAND_START_PERIODIC_MEASUREMENT_10_HIGH >> 8),
                     (TEE301_COMMAND_START_PERIODIC_MEASUREMENT_10_HIGH & 0xFF)])

    def end_periodic_measurement(self):
        """ends the periodic measurement"""
        self.wire_write([(TEE301_COMMAND_END_PERIODIC_MEASUREMENT >> 8),
                         (TEE301_COMMAND_END_PERIODIC_MEASUREMENT & 0xFF)])

    def heater_on(self):
        """turns the heater on"""
        self.wire_write([(TEE301_COMMAND_HEATER_ON >> 8),
                         (TEE301_COMMAND_HEATER_ON & 0xFF)])

    def heater_off(self):
        """turns the heater off"""
        self.wire_write([(TEE301_COMMAND_HEATER_OFF >> 8),
                         (TEE301_COMMAND_HEATER_OFF & 0xFF)])

    def read_identification(self):
        """reads the identification number"""
        i2c_response = self.wire_write_read(
            [(TEE301_COMMAND_READ_IDENTIFICATION >> 8),
             (TEE301_COMMAND_READ_IDENTIFICATION & 0xFF)], 9)
        if i2c_response[8] == calc_crc8(i2c_response, 0, 8):
            return i2c_response
        else:
            raise Warning(get_status_string(2))

    def reset(self):
        """resets the sensor"""
        self.wire_write([(TEE301_COMMAND_SOFT_RESET >> 8),
                         (TEE301_COMMAND_SOFT_RESET & 0xFF)])

    def i2c_reset(self):
        """resets all the sensor"""
        write_command = i2c_msg.write(0x00, 0x06)
        with SMBus(1) as tee301_communication:
            tee301_communication.i2c_rdwr(write_command)

    def constant_heater_on_off(self):
        """get the informatio if the heater is on or off"""
        i2c_response = self.wire_write_read(
            [(TEE301_COMMAND_READ_REGISTER >> 8),
             (TEE301_COMMAND_READ_REGISTER & 0xFF)], 3)
        if i2c_response[2] == calc_crc8(i2c_response, 0, 2):
            i2c_response[0] = (i2c_response[0] << 2) & 255
            return i2c_response[0] >> 7
        else:
            raise Warning(get_status_string(2))

    def read_statusregister_1(self):
        """read statusregister 1"""
        i2c_response = self.wire_write_read(
            [(TEE301_COMMAND_READ_REGISTER >> 8),
             (TEE301_COMMAND_READ_REGISTER & 0xFF)], 3)
        if i2c_response[2] == calc_crc8(i2c_response, 0, 2):
            return i2c_response[0]
        else:
            raise Warning(get_status_string(2))

    def read_statusregister_2(self):
        """read statusregister 2"""
        i2c_response = self.wire_write_read(
            [(TEE301_COMMAND_READ_REGISTER >> 8),
             (TEE301_COMMAND_READ_REGISTER & 0xFF)], 3)
        if i2c_response[2] == calc_crc8(i2c_response, 0, 2):
            return i2c_response[1]
        else:
            raise Warning(get_status_string(2))

    def clear_statusregister_1(self):
        """clear the statusregister 1"""
        self.wire_write([(TEE301_COMMAND_CLEAR_REGISTER >> 8),
                         (TEE301_COMMAND_CLEAR_REGISTER & 0xFF)])

    def wire_write_read(self,  buf, receiving_bytes):
        """write a command to the sensor to get different answers like temperature values,..."""
        write_command = i2c_msg.write(self.i2c_address, buf)
        read_command = i2c_msg.read(self.i2c_address, receiving_bytes)
        with SMBus(1) as tee301_communication:
            tee301_communication.i2c_rdwr(write_command, read_command)
        return list(read_command)

    def wire_write(self, buf):
        """write to the sensor"""
        write_command = i2c_msg.write(self.i2c_address, buf)
        with SMBus(1) as tee301_communication:
            tee301_communication.i2c_rdwr(write_command)
