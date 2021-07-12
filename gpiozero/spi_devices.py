# vim: set fileencoding=utf-8:
#
# GPIO Zero: a library for controlling the Raspberry Pi's GPIO pins
#
# Copyright (c) 2016-2021 Dave Jones <dave@waveform.org.uk>
# Copyright (c) 2020 Grzegorz Szymaszek <gszymaszek@short.pl>
# Copyright (c) 2016-2019 Andrew Scheller <github@loowis.durge.org>
# Copyright (c) 2016-2018 Ben Nuttall <ben@bennuttall.com>
#
# SPDX-License-Identifier: BSD-3-Clause

from math import log, ceil
from operator import or_
import time
try:
    from functools import reduce
except ImportError:
    pass # py2's reduce is built-in

from .exc import DeviceClosed, SPIBadChannel, InputDeviceError
from .devices import Device
from .output_devices import OutputDevice


class SPIDevice(Device):
    """
    Extends :class:`Device`. Represents a device that communicates via the SPI
    protocol.

    See :ref:`spi_args` for information on the keyword arguments that can be
    specified with the constructor.
    """
    def __init__(self, **spi_args):
        self._spi = None
        super().__init__(pin_factory=spi_args.pop('pin_factory', None))
        self._spi = self.pin_factory.spi(**spi_args)

    def close(self):
        if getattr(self, '_spi', None):
            self._spi.close()
            self._spi = None
        super().close()

    @property
    def closed(self):
        return self._spi is None

    def _int_to_words(self, pattern):
        """
        Given a bit-pattern expressed an integer number, return a sequence of
        the individual words that make up the pattern. The number of bits per
        word will be obtained from the internal SPI interface.
        """
        try:
            bits_required = int(ceil(log(pattern, 2))) + 1
        except ValueError:
            # pattern == 0 (technically speaking, no bits are required to
            # transmit the value zero ;)
            bits_required = 1
        shifts = range(0, bits_required, self._spi.bits_per_word)[::-1]
        mask = 2 ** self._spi.bits_per_word - 1
        return [(pattern >> shift) & mask for shift in shifts]

    def _words_to_int(self, words, expected_bits=None):
        """
        Given a sequence of words which each fit in the internal SPI
        interface's number of bits per word, returns the value obtained by
        concatenating each word into a single bit-string.

        If *expected_bits* is specified, it limits the size of the output to
        the specified number of bits (by masking off bits above the expected
        number). If unspecified, no limit will be applied.
        """
        if expected_bits is None:
            expected_bits = len(words) * self._spi.bits_per_word
        shifts = range(0, expected_bits, self._spi.bits_per_word)[::-1]
        mask = 2 ** expected_bits - 1
        return reduce(or_, (word << shift for word, shift in zip(words, shifts))) & mask

    def __repr__(self):
        try:
            self._check_open()
            return (
                "<gpiozero.{self.__class__.__name__} object using "
                "{self._spi!r}>".format(self=self))
        except DeviceClosed:
            return (
                "<gpiozero.{self.__class__.__name__} object "
                "closed>".format(self=self))


class AnalogInputDevice(SPIDevice):
    """
    Represents an analog input device connected to SPI (serial interface).

    Typical analog input devices are `analog to digital converters`_ (ADCs).
    Several classes are provided for specific ADC chips, including
    :class:`MCP3004`, :class:`MCP3008`, :class:`MCP3204`, and :class:`MCP3208`.

    The following code demonstrates reading the first channel of an MCP3008
    chip attached to the Pi's SPI pins::

        from gpiozero import MCP3008

        pot = MCP3008(0)
        print(pot.value)

    The :attr:`value` attribute is normalized such that its value is always
    between 0.0 and 1.0 (or in special cases, such as differential sampling,
    -1 to +1). Hence, you can use an analog input to control the brightness of
    a :class:`PWMLED` like so::

        from gpiozero import MCP3008, PWMLED

        pot = MCP3008(0)
        led = PWMLED(17)
        led.source = pot

    The :attr:`voltage` attribute reports values between 0.0 and *max_voltage*
    (which defaults to 3.3, the logic level of the GPIO pins).

    .. _analog to digital converters: https://en.wikipedia.org/wiki/Analog-to-digital_converter
    """

    def __init__(self, bits, max_voltage=3.3, **spi_args):
        if bits is None:
            raise InputDeviceError(
                'you must specify the bit resolution of the device')
        self._bits = bits
        self._min_value = -(2 ** bits)
        self._range = 2 ** (bits + 1) - 1
        if max_voltage <= 0:
            raise InputDeviceError('max_voltage must be positive')
        self._max_voltage = float(max_voltage)
        super().__init__(shared=True, **spi_args)

    @property
    def bits(self):
        """
        The bit-resolution of the device/channel.
        """
        return self._bits

    def _read(self):
        raise NotImplementedError

    @property
    def value(self):
        """
        The current value read from the device, scaled to a value between 0 and
        1 (or -1 to +1 for certain devices operating in differential mode).
        """
        return (2 * (self._read() - self._min_value) / self._range) - 1

    @property
    def raw_value(self):
        """
        The raw value as read from the device.
        """
        return self._read()

    @property
    def max_voltage(self):
        """
        The voltage required to set the device's value to 1.
        """
        return self._max_voltage

    @property
    def voltage(self):
        """
        The current voltage read from the device. This will be a value between
        0 and the *max_voltage* parameter specified in the constructor.
        """
        return self.value * self._max_voltage


class MCP3xxx(AnalogInputDevice):
    """
    Extends :class:`AnalogInputDevice` to implement an interface for all ADC
    chips with a protocol similar to the Microchip MCP3xxx series of devices.
    """

    def __init__(self, channel=0, bits=10, differential=False, max_voltage=3.3,
                 **spi_args):
        self._channel = channel
        self._differential = bool(differential)
        super().__init__(bits, max_voltage, **spi_args)

    @property
    def channel(self):
        """
        The channel to read data from. The MCP3008/3208/3304 have 8 channels
        (0-7), while the MCP3004/3204/3302 have 4 channels (0-3), the
        MCP3002/3202 have 2 channels (0-1), and the MCP3001/3201/3301 only
        have 1 channel.
        """
        return self._channel

    @property
    def differential(self):
        """
        If ``True``, the device is operated in differential mode. In this mode
        one channel (specified by the channel attribute) is read relative to
        the value of a second channel (implied by the chip's design).

        Please refer to the device data-sheet to determine which channel is
        used as the relative base value (for example, when using an
        :class:`MCP3008` in differential mode, channel 0 is read relative to
        channel 1).
        """
        return self._differential

    def _read(self):
        return self._words_to_int(
            self._spi.transfer(self._send())[-2:], self.bits
            )

    def _send(self):
        # MCP3004/08 protocol looks like the following:
        #
        #     Byte        0        1        2
        #     ==== ======== ======== ========
        #     Tx   00000001 MCCCxxxx xxxxxxxx
        #     Rx   xxxxxxxx xxxxx0RR RRRRRRRR
        #
        # MCP3204/08 protocol looks like the following:
        #
        #     Byte        0        1        2
        #     ==== ======== ======== ========
        #     Tx   000001MC CCxxxxxx xxxxxxxx
        #     Rx   xxxxxxxx xxx0RRRR RRRRRRRR
        #
        # The transmit bits start with several preamble "0" bits, the number
        # of which is determined by the amount required to align the last byte
        # of the result with the final byte of output. A start "1" bit is then
        # transmitted, followed by the single/differential bit (M); 1 for
        # single-ended read, 0 for differential read. Next comes three bits for
        # channel (C).
        #
        # Read-out begins with a don't care bit (x), then a null bit (0)
        # followed by the result bits (R). All other bits are don't care (x).
        #
        # The 3x01 variant of the chips always operates in differential mode
        # and effectively only has one channel (composed of an IN+ and IN-). As
        # such it requires no input, just output.
        return self._int_to_words(
            (0b10000 | (not self.differential) << 3 | self.channel) << (self.bits + 2)
            )


class MCP3xx2(MCP3xxx):
    def _send(self):
        # MCP3002 protocol looks like the following:
        #
        #     Byte        0        1
        #     ==== ======== ========
        #     Tx   01MCLxxx xxxxxxxx
        #     Rx   xxxxx0RR RRRRRRRR for the 3002
        #
        # MCP3202 protocol looks like the following:
        #
        #     Byte        0        1        2
        #     ==== ======== ======== ========
        #     Tx   00000001 MCLxxxxx xxxxxxxx
        #     Rx   xxxxxxxx xxx0RRRR RRRRRRRR
        #
        # The transmit bits start with several preamble "0" bits, the number of
        # which is determined by the amount required to align the last byte of
        # the result with the final byte of output. A start "1" bit is then
        # transmitted, followed by the single/differential bit (M); 1 for
        # single-ended read, 0 for differential read. Next comes a single bit
        # for channel (C) then the MSBF bit (L) which selects whether the data
        # will be read out in MSB form only (1) or whether LSB read-out will
        # occur after MSB read-out (0).
        #
        # Read-out begins with a null bit (0) followed by the result bits (R).
        # All other bits are don't care (x).
        return self._int_to_words(
            (0b1001 | (not self.differential) << 2 | self.channel << 1) << (self.bits + 1)
            )


class MCP30xx(MCP3xxx):
    """
    Extends :class:`MCP3xxx` to implement an interface for all ADC
    chips with a protocol similar to the Microchip MCP30xx series of devices.
    """

    def __init__(self, channel=0, differential=False, max_voltage=3.3,
                 **spi_args):
        super().__init__(channel, 10, differential, max_voltage, **spi_args)


class MCP32xx(MCP3xxx):
    """
    Extends :class:`MCP3xxx` to implement an interface for all ADC
    chips with a protocol similar to the Microchip MCP32xx series of devices.
    """

    def __init__(self, channel=0, differential=False, max_voltage=3.3, **spi_args):
        super().__init__(channel, 12, differential, max_voltage, **spi_args)


class MCP33xx(MCP3xxx):
    """
    Extends :class:`MCP3xxx` with functionality specific to the MCP33xx family
    of ADCs; specifically this handles the full differential capability of
    these chips supporting the full 13-bit signed range of output values.
    """

    def __init__(self, channel=0, differential=False, max_voltage=3.3, **spi_args):
        super().__init__(channel, 12, differential, max_voltage, **spi_args)

    def _read(self):
        if self.differential:
            result = self._words_to_int(
                self._spi.transfer(self._send())[-2:], self.bits + 1)
            # Account for the sign bit
            if result > 4095:
                return -(8192 - result)
            else:
                return result
        else:
            return super()._read()

    def _send(self):
        # MCP3302/04 protocol looks like the following:
        #
        #     Byte        0        1        2
        #     ==== ======== ======== ========
        #     Tx   00001MCC Cxxxxxxx xxxxxxxx
        #     Rx   xxxxxxxx xx0SRRRR RRRRRRRR
        #
        # The transmit bits start with 4 preamble bits "0000", a start bit "1"
        # followed by the single/differential bit (M) which is 1 for
        # single-ended read, and 0 for differential read, followed by 3-bits
        # for the channel (C). The remainder of the transmission are "don't
        # care" bits (x).
        #
        # The first byte received and the top 2 bits of the second byte are
        # don't care bits (x). These are followed by a null bit (0), then the
        # sign bit (S), and then the 12 result bits (R).
        #
        # In single read mode (the default) the sign bit is always zero and the
        # result is effectively 12-bits. In differential mode, the sign bit is
        # significant and the result is a two's-complement 13-bit value.
        #
        # The MCP3301 variant operates similarly to the other MCP3x01 variants;
        # no input, just output and always differential.
        return self._int_to_words(
            (0b10000 | (not self.differential) << 3 | self.channel) << (self.bits + 3)
            )

    @property
    def differential(self):
        """
        If ``True``, the device is operated in differential mode. In this mode
        one channel (specified by the channel attribute) is read relative to
        the value of a second channel (implied by the chip's design).

        Please refer to the device data-sheet to determine which channel is
        used as the relative base value (for example, when using an
        :class:`MCP3304` in differential mode, channel 0 is read relative to
        channel 1).
        """
        return super().differential

    @property
    def value(self):
        """
        The current value read from the device, scaled to a value between 0 and
        1 (or -1 to +1 for devices operating in differential mode).
        """
        return super().value


class MCP3001(MCP30xx):
    """
    The `MCP3001`_ is a 10-bit analog to digital converter with 1 channel.
    Please note that the MCP3001 always operates in differential mode,
    measuring the value of IN+ relative to IN-.

    .. _MCP3001: http://www.farnell.com/datasheets/630400.pdf
    """
    def __init__(self, max_voltage=3.3, **spi_args):
        super().__init__(0, True, max_voltage, **spi_args)

    def _read(self):
        # MCP3001 protocol looks like the following:
        #
        #     Byte        0        1
        #     ==== ======== ========
        #     Rx   xx0RRRRR RRRRRxxx
        return self._words_to_int(self._spi.read(2), 13) >> 3


class MCP3002(MCP30xx, MCP3xx2):
    """
    The `MCP3002`_ is a 10-bit analog to digital converter with 2 channels
    (0-1).

    .. _MCP3002: http://www.farnell.com/datasheets/1599363.pdf
    """
    def __init__(self, channel=0, differential=False, max_voltage=3.3, **spi_args):
        if not 0 <= channel < 2:
            raise SPIBadChannel('channel must be 0 or 1')
        super().__init__(channel, differential, max_voltage, **spi_args)


class MCP3004(MCP30xx):
    """
    The `MCP3004`_ is a 10-bit analog to digital converter with 4 channels
    (0-3).

    .. _MCP3004: http://www.farnell.com/datasheets/808965.pdf
    """
    def __init__(self, channel=0, differential=False, max_voltage=3.3, **spi_args):
        if not 0 <= channel < 4:
            raise SPIBadChannel('channel must be between 0 and 3')
        super().__init__(channel, differential, max_voltage, **spi_args)


class MCP3008(MCP30xx):
    """
    The `MCP3008`_ is a 10-bit analog to digital converter with 8 channels
    (0-7).

    .. _MCP3008: http://www.farnell.com/datasheets/808965.pdf
    """
    def __init__(self, channel=0, differential=False, max_voltage=3.3, **spi_args):
        if not 0 <= channel < 8:
            raise SPIBadChannel('channel must be between 0 and 7')
        super().__init__(channel, differential, max_voltage, **spi_args)


class MCP3201(MCP32xx):
    """
    The `MCP3201`_ is a 12-bit analog to digital converter with 1 channel.
    Please note that the MCP3201 always operates in differential mode,
    measuring the value of IN+ relative to IN-.

    .. _MCP3201: http://www.farnell.com/datasheets/1669366.pdf
    """
    def __init__(self, max_voltage=3.3, **spi_args):
        super().__init__(0, True, max_voltage, **spi_args)

    def _read(self):
        # MCP3201 protocol looks like the following:
        #
        #     Byte        0        1
        #     ==== ======== ========
        #     Rx   xx0RRRRR RRRRRRRx
        return self._words_to_int(self._spi.read(2), 13) >> 1


class MCP3202(MCP32xx, MCP3xx2):
    """
    The `MCP3202`_ is a 12-bit analog to digital converter with 2 channels
    (0-1).

    .. _MCP3202: http://www.farnell.com/datasheets/1669376.pdf
    """
    def __init__(self, channel=0, differential=False, max_voltage=3.3, **spi_args):
        if not 0 <= channel < 2:
            raise SPIBadChannel('channel must be 0 or 1')
        super().__init__(channel, differential, max_voltage, **spi_args)


class MCP3204(MCP32xx):
    """
    The `MCP3204`_ is a 12-bit analog to digital converter with 4 channels
    (0-3).

    .. _MCP3204: http://www.farnell.com/datasheets/808967.pdf
    """
    def __init__(self, channel=0, differential=False, max_voltage=3.3, **spi_args):
        if not 0 <= channel < 4:
            raise SPIBadChannel('channel must be between 0 and 3')
        super().__init__(channel, differential, max_voltage, **spi_args)


class MCP3208(MCP32xx):
    """
    The `MCP3208`_ is a 12-bit analog to digital converter with 8 channels
    (0-7).

    .. _MCP3208: http://www.farnell.com/datasheets/808967.pdf
    """
    def __init__(self, channel=0, differential=False, max_voltage=3.3, **spi_args):
        if not 0 <= channel < 8:
            raise SPIBadChannel('channel must be between 0 and 7')
        super().__init__(channel, differential, max_voltage, **spi_args)


class MCP3301(MCP33xx):
    """
    The `MCP3301`_ is a signed 13-bit analog to digital converter.  Please note
    that the MCP3301 always operates in differential mode measuring the
    difference between IN+ and IN-. Its output value is scaled from -1 to +1.

    .. _MCP3301: http://www.farnell.com/datasheets/1669397.pdf
    """
    def __init__(self, max_voltage=3.3, **spi_args):
        super().__init__(0, True, max_voltage, **spi_args)

    def _read(self):
        # MCP3301 protocol looks like the following:
        #
        #     Byte        0        1
        #     ==== ======== ========
        #     Rx   xx0SRRRR RRRRRRRR
        result = self._words_to_int(self._spi.read(2), 13)
        # Account for the sign bit
        if result > 4095:
            return -(8192 - result)
        else:
            return result


class MCP3302(MCP33xx):
    """
    The `MCP3302`_ is a 12/13-bit analog to digital converter with 4 channels
    (0-3). When operated in differential mode, the device outputs a signed
    13-bit value which is scaled from -1 to +1. When operated in single-ended
    mode (the default), the device outputs an unsigned 12-bit value scaled from
    0 to 1.

    .. _MCP3302: http://www.farnell.com/datasheets/1486116.pdf
    """
    def __init__(self, channel=0, differential=False, max_voltage=3.3, **spi_args):
        if not 0 <= channel < 4:
            raise SPIBadChannel('channel must be between 0 and 4')
        super().__init__(channel, differential, max_voltage, **spi_args)


class MCP3304(MCP33xx):
    """
    The `MCP3304`_ is a 12/13-bit analog to digital converter with 8 channels
    (0-7). When operated in differential mode, the device outputs a signed
    13-bit value which is scaled from -1 to +1. When operated in single-ended
    mode (the default), the device outputs an unsigned 12-bit value scaled from
    0 to 1.

    .. _MCP3304: http://www.farnell.com/datasheets/1486116.pdf
    """
    def __init__(self, channel=0, differential=False, max_voltage=3.3, **spi_args):
        if not 0 <= channel < 8:
            raise SPIBadChannel('channel must be between 0 and 7')
        super(MCP3304, self).__init__(channel, differential, max_voltage, **spi_args)


class NRF24L01_REGISTERS:
    """ nRF24L01 registers """

    CONFIG     = 0x00 # register for configuring IRQ, CRC, PWR & RX/TX roles
    EN_AA      = 0x01 # register for auto-ACK feature. Each bit represents this feature per pipe
    EN_RX      = 0x02 # register to open/close pipes. Each bit represents this feature per pipe
    SETUP_AW   = 0x03 # address width register
    SETUP_RETR = 0x04 # auto-retry count and auto-retry delay register
    RF_CH      = 0x05 # channel register
    RF_SETUP   = 0x06 # RF Power Amplifier & Data Rate
    RX_ADDR    = 0x0a # RX pipe addresses == [0,5]:[0x0A:0x0F]
    RX_PW      = 0x11 # RX payload widths on pipes == [0,5]:[0x11,0x16]
    FIFO       = 0x17 # register containing info on both RX/TX FIFOs + re-use payload flag
    DYNPD      = 0x1c # dynamic payloads feature. Each bit represents this feature per pipe
    FEATURE    = 0x1d # global flags for dynamic payloads, custom ACK payloads, & Ask no ACK
    TX_ADDR    = 0x10 # Address that is used for TX transmissions


class NRF24L01(SPIDevice):
    """A driver class for the nRF24L01(+) transceiver radios."""
    def __init__(self, ce_pin, **spi_args):
        self._status = 0
        # init shadow copy of RX addresses for all pipes for context manager
        self._pipes = [
            bytearray([0xE7] * 5),
            bytearray([0xC2] * 5),
            0xC3,
            0xC4,
            0xC5,
            0xC6,
        ]
        # self._status = status byte returned on all SPI transactions
        # pre-configure the CONFIGURE register:
        #   0x0E = all IRQs enabled, CRC is 2 bytes, and power up in TX mode
        self._status, self._config, self._spi = (0, 0x0E, None)

        # init the SPI bus and pins
        super(NRF24L01, self).__init__(shared=True, **spi_args)
        # check for device presence by verifying nRF24L01 is in TX + standby-I mode
        self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)
        hw_check = self._reg_read(NRF24L01_REGISTERS.CONFIG)
        if hw_check != self._config:
            raise RuntimeError(
                "nRF24L01 Hardware not responding; expected {}, got "
                "{}".format(hex(self._config), hex(hw_check))
            )

        # capture RX addresses from registers
        for i in range(6):
            if i < 2:
                self._pipes[i] = self._reg_read_bytes(NRF24L01_REGISTERS.RX_ADDR + i)
            else:
                self._pipes[i] = self._reg_read(NRF24L01_REGISTERS.RX_ADDR + i)

        #: This attribute controls the nRF24L01's the CE pin (for advanced users).
        self.ce_pin = OutputDevice(pin=ce_pin, pin_factory=self.pin_factory)
        # reset ce_pin.value & disable the chip comms
        self.ce_pin.value = False
        # if radio is powered up and CE is LOW: standby-I mode
        # if radio is powered up and CE is HIGH: standby-II mode

        # test is nRF24L01 is a plus variant using a command specific to
        # non-plus variants
        self._open_pipes, self._is_plus_variant = (0, False)  # close all RX pipes
        self._features = self._reg_read(NRF24L01_REGISTERS.FEATURE)
        self._reg_write(0x50, 0x73)  # derelict command toggles FEATURE register
        after_toggle = self._reg_read(NRF24L01_REGISTERS.FEATURE)
        if self._features == after_toggle:
            self._is_plus_variant = True
        elif not after_toggle:  # if features are disabled
            self._reg_write(0x50, 0x73)  # ensure they're enabled

        # pre-configure features for TX operations:
        #   5 = enable dynamic_payloads, disable custom ack payloads, &
        #       allow ask_no_ack command
        self._features = 5
        # init shadow copy of last RX_ADDR_P0 written to pipe 0 needed as
        # open_tx_pipe() appropriates pipe 0 for ACK packet
        self._pipe0_read_addr = None
        # shadow copy of the TX_ADDRESS
        self._tx_address = self._reg_read_bytes(NRF24L01_REGISTERS.TX_ADDR)
        # pre-configure the SETUP_RETR register
        self._retry_setup = 0x5F  # ard = 1500; arc = 15
        # pre-configure the RF_SETUP register
        self._rf_setup = 0x07  # 1 Mbps data_rate, and 0 dbm pa_level
        # pre-configure dynamic_payloads & auto_ack
        self._dyn_pl, self._aa = (0x3F,) * 2  # 0x3F = enable feature on all pipes
        self._channel = 76  # 2.476 GHz
        self._addr_len = 5  # 5-byte long addresses
        self._pl_len = [32] * 6  # 32-byte static payloads for all pipes

        self.ce_pin.value = 0  # ensure standby-I mode to write to CONFIG register
        self._config |= 2
        self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)
        self._reg_write(NRF24L01_REGISTERS.RF_SETUP, self._rf_setup)
        self._reg_write(NRF24L01_REGISTERS.EN_RX, self._open_pipes)
        self._reg_write(NRF24L01_REGISTERS.DYNPD, self._dyn_pl)
        self._reg_write(NRF24L01_REGISTERS.EN_AA, self._aa)
        self._reg_write(NRF24L01_REGISTERS.FEATURE, self._features)
        self._reg_write(NRF24L01_REGISTERS.SETUP_RETR, self._retry_setup)
        for i, addr in enumerate(self._pipes):
            if i < 2:
                self._reg_write_bytes(NRF24L01_REGISTERS.RX_ADDR + i, addr)
            else:
                self._reg_write(NRF24L01_REGISTERS.RX_ADDR + i, addr)
            self.set_payload_length(self._pl_len[i], i)
        self._reg_write_bytes(NRF24L01_REGISTERS.TX_ADDR, self._tx_address)
        self._reg_write(NRF24L01_REGISTERS.RF_CH, self._channel)
        self._reg_write(NRF24L01_REGISTERS.SETUP_AW, self._addr_len - 2)
        self.flush_rx()
        self.flush_tx()
        self.clear_status_flags()

    def __exit__(self, *exc):
        self.ce_pin.value = 0  # ensure standby-I mode to write to CONFIG register
        self._config &= 0x7D  # power off radio
        self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)
        return super().__exit__(*exc)

    def _reg_read(self, reg):
        reg = [reg, 0]  # 1 status byte + 1 byte of returned content
        # time.sleep(0.000005)  # time for CSN to settle
        buf = self._spi.transfer(reg)
        self._status = buf[0]  # save status byte
        return buf[1]  # drop status byte and return the rest

    def _reg_read_bytes(self, reg, buf_len=5):
        reg = list(bytes([reg]) + bytes(buf_len))
        # time.sleep(0.000005)  # time for CSN to settle
        buf = self._spi.transfer(reg)
        self._status = buf[0]  # save status byte
        return bytearray(buf[1:])  # drop status byte and return the rest

    def _reg_write_bytes(self, reg, out_buf):
        if isinstance(out_buf, (bytes, bytearray)):
            out_buf = list(bytes([0x20 | reg]) + out_buf)
        elif isinstance(out_buf, list):
            out_buf.insert(0, 0x20 | reg)
        # time.sleep(0.000005)  # time for CSN to settle
        buf = self._spi.transfer(out_buf)
        self._status = buf[0]  # save status byte

    def _reg_write(self, reg, value=None):
        if value is None: # only when writing non-operation command
            value = [reg] # non-operation command is 0x00, so don't (0x20 | reg) here
        elif isinstance(value, int):
            value = [(0x20 if reg != 0x50 else 0) | reg, value]
        # time.sleep(0.000005)  # time for CSN to settle
        buf = self._spi.transfer(value)
        self._status = buf[0]  # save status byte

    @property
    def address_length(self):
        """This `int` attribute specifies the length (in bytes) of addresses."""
        self._addr_len = self._reg_read(NRF24L01_REGISTERS.SETUP_AW) + 2
        return self._addr_len

    @address_length.setter
    def address_length(self, length):
        if not 3 <= length <= 5:
            raise ValueError("address_length can only be set in range [3, 5] bytes")
        self._addr_len = int(length)
        self._reg_write(NRF24L01_REGISTERS.SETUP_AW, length - 2)

    def open_tx_pipe(self, address):
        """This function is used to open a data pipe for OTA (over the air) TX
        transmissions."""
        if self._aa & 1:
            for i, val in enumerate(address):
                self._pipes[0][i] = val
            self._reg_write_bytes(NRF24L01_REGISTERS.RX_ADDR, address)
        for i, val in enumerate(address):
            self._tx_address[i] = val
        self._reg_write_bytes(NRF24L01_REGISTERS.TX_ADDR, address)

    def close_rx_pipe(self, pipe_number):
        """This function is used to close a specific data pipe from OTA (over the air) RX
        transmissions."""
        if pipe_number < 0 or pipe_number > 5:
            raise IndexError("pipe number must be in range [0, 5]")
        self._open_pipes = self._reg_read(NRF24L01_REGISTERS.EN_RX)
        if not pipe_number:
            self._pipe0_read_addr = None
        if self._open_pipes & (1 << pipe_number):
            self._open_pipes = self._open_pipes & ~(1 << pipe_number)
            self._reg_write(NRF24L01_REGISTERS.EN_RX, self._open_pipes)

    def open_rx_pipe(self, pipe_number, address):
        """This function is used to open a specific data pipe for OTA (over the air) RX
        transmissions."""
        if not 0 <= pipe_number <= 5:
            raise IndexError("pipe number must be in range [0, 5]")
        if not address:
            raise ValueError("address length cannot be 0")
        if pipe_number < 2:
            if not pipe_number:
                self._pipe0_read_addr = address
            for i, val in enumerate(address):
                self._pipes[pipe_number][i] = val
            self._reg_write_bytes(NRF24L01_REGISTERS.RX_ADDR + pipe_number, address)
        else:
            self._pipes[pipe_number] = address[0]
            self._reg_write(NRF24L01_REGISTERS.RX_ADDR + pipe_number, address[0])
        self._open_pipes = self._reg_read(NRF24L01_REGISTERS.EN_RX) | (1 << pipe_number)
        self._reg_write(NRF24L01_REGISTERS.EN_RX, self._open_pipes)

    @property
    def listen(self):
        """An attribute to represent the nRF24L01 primary role (RX or TX) as a radio."""
        return self.power and bool(self._config & 1)

    @listen.setter
    def listen(self, is_rx):
        self.ce_pin.value = 0
        if is_rx:
            if self._pipe0_read_addr is not None and self._aa & 1:
                for i, val in enumerate(self._pipe0_read_addr):
                    self._pipes[0][i] = val
                self._reg_write_bytes(NRF24L01_REGISTERS.RX_ADDR, self._pipe0_read_addr)
            elif self._pipe0_read_addr is None and self._open_pipes & 1:
                self._open_pipes &= 0x3E  # close_rx_pipe(0) is slower
                self._reg_write(NRF24L01_REGISTERS.EN_RX, self._open_pipes)
            is_pwr_up = self._config & 2
            self._config = (self._config & 0xFC) | 3
            self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)
            if not is_pwr_up:
                time.sleep(0.00015)  # mandatory wait to power up radio
            if self._status & 0x70:
                self.clear_status_flags()
            self.ce_pin.value = 1  # mandatory pulse is > 130 µs
            time.sleep(0.00013)
        else:
            if self._features & 6 == 6 and ((self._aa & self._dyn_pl) & 1):
                self.flush_tx()
            if self._aa & 1 and not self._open_pipes & 1:
                self._open_pipes |= 1
                self._reg_write(NRF24L01_REGISTERS.EN_RX, self._open_pipes)
            self._config = self._config & 0xFE | 2
            self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)
            time.sleep(0.00016)

    def available(self):
        """A `bool` describing if there is a payload in the RX FIFO."""
        return self.update() and self._status >> 1 & 7 < 6

    def any(self):
        """This function returns the length of the next available payload from the RX
        FIFO (if any at at all)."""
        if self.available():
            if self._features & 4:
                return self._reg_read(0x60)
            return self._pl_len[(self._status >> 1) & 7]
        return 0

    def read(self, length=None):
        """This function is used to retrieve the next available payload in the RX FIFO buffer, then
        clears the :attr:`irq_dr` status flag."""
        return_size = length if length is not None else self.any()
        if not return_size:
            return None
        result = self._reg_read_bytes(0x61, return_size)
        self.clear_status_flags(True, False, False)
        return result

    def send(self, buf, ask_no_ack=False, force_retry=0, send_only=False):
        """This blocking function is used to transmit payload(s)."""
        self.ce_pin.value = 0
        if isinstance(buf, (list, tuple)):
            result = []
            for b in buf:
                result.append(self.send(b, ask_no_ack, force_retry, send_only))
            return result
        if self._status & 0x11:
            self.flush_tx()
        if not send_only and self._status >> 1 & 7 < 6:
            self.flush_rx()
        self.write(buf, ask_no_ack)
        up_cnt = 0
        while not self._status & 0x30:
            up_cnt += self.update()
        self.ce_pin.value = 0
        result = bool(self._status & 0x20)
        # print(
        #     "send() waited {} updates DS: {} DR: {} DF: {}".format(
        #         up_cnt, self.irq_ds, self.irq_dr, self.irq_df
        #     )
        # )
        while force_retry and not result:
            result = self.resend(send_only)
            force_retry -= 1
        if self._status & 0x60 == 0x60 and not send_only:
            result = self.read()
        return result

    @property
    def tx_full(self):
        """An attribute to represent the nRF24L01's status flag signaling that the TX FIFO buffer
        is full. (read-only)"""
        return bool(self._status & 1)

    def update(self):
        """This function is only used to get an updated status byte over SPI from the nRF24L01."""
        # perform non-operation to get status byte
        # should be faster than reading the STATUS register
        self._reg_write(0xFF)
        return True

    @property
    def pipe(self):
        """This function returns information about the data pipe that received the next available
        payload in the RX FIFO buffer."""
        result = (self._status & 0x0E) >> 1
        if result <= 5:
            return result
        return None

    @property
    def irq_dr(self):
        """A `bool` that represents the "Data Ready" interrupted flag. (read-only)"""
        return bool(self._status & 0x40)

    @property
    def irq_ds(self):
        """A `bool` that represents the "Data Sent" interrupted flag. (read-only)"""
        return bool(self._status & 0x20)

    @property
    def irq_df(self):
        """A `bool` that represents the "Data Failed" interrupted flag. (read-only)"""
        return bool(self._status & 0x10)

    def clear_status_flags(self, data_recv=True, data_sent=True, data_fail=True):
        """This clears the interrupt flags in the status register."""
        # 0x07 = STATUS register; only bits 6 through 4 are write-able
        config = bool(data_recv) << 6 | bool(data_sent) << 5
        self._reg_write(7, config | bool(data_fail) << 4)

    def interrupt_config(self, data_recv=True, data_sent=True, data_fail=True):
        """Sets the configuration of the nRF24L01's IRQ (interrupt) pin (write-only).
        The signal from the nRF24L01's IRQ pin is active LOW."""
        self._config = self._reg_read(NRF24L01_REGISTERS.CONFIG)  # refresh data
        # save to register and update local copy of pwr & RX/TX modes' flags
        self._config = (self._config & 0x0F) | (not data_fail << 4) | (not data_sent << 5) | \
            (not data_recv << 6)
        self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)

    def print_details(self, dump_pipes=True):
        """This debuggung function aggregates and outputs all status/condition related information
        from the nRF24L01."""
        observer = self._reg_read(8)
        _fifo = self._reg_read(0x17)
        self._config = self._reg_read(NRF24L01_REGISTERS.CONFIG)
        self._rf_setup = self._reg_read(NRF24L01_REGISTERS.RF_SETUP)
        self._retry_setup = self._reg_read(NRF24L01_REGISTERS.SETUP_RETR)
        self._channel = self.channel
        self._addr_len = self._reg_read(NRF24L01_REGISTERS.SETUP_AW) + 2
        self._features = self._reg_read(NRF24L01_REGISTERS.FEATURE)
        self._aa = self._reg_read(NRF24L01_REGISTERS.EN_AA)
        self._dyn_pl = self._reg_read(NRF24L01_REGISTERS.DYNPD)
        _crc = (
            (2 if self._config & 4 else 1)
            if self._aa
            else max(0, ((self._config & 0x0C) >> 2) - 1)
        )
        d_rate = self._rf_setup & 0x28
        d_rate = (2 if d_rate == 8 else 250) if d_rate else 1
        _pa_level = (3 - ((self._rf_setup & 6) >> 1)) * -6

        print("Is a plus variant_________{}".format(self.is_plus_variant))
        print(
            "Channel___________________{} ~ {} GHz".format(
                self._channel, (self._channel + 2400) / 1000
            )
        )
        print(
            "RF Data Rate______________{}".format(d_rate),
            "Mbps" if d_rate != 250 else "Kbps",
        )
        print("RF Power Amplifier________{} dbm".format(_pa_level))
        print(
            "RF Low Noise Amplifier____{}".format(
                "Enabled" if bool(self._rf_setup & 1) else "Disabled"
            )
        )
        print("CRC bytes_________________{}".format(_crc))
        print("Address length____________{} bytes".format(self._addr_len))
        print("TX Payload lengths________{} bytes".format(self._pl_len[0]))
        print(
            "Auto retry delay__________{} microseconds".format(
                ((self._rf_setup & 0xF0) >> 4) * 250 + 250
            )
        )
        print("Auto retry attempts_______{} maximum".format(self._rf_setup & 0x0F))
        print("Re-use TX FIFO____________{}".format(bool(_fifo & 64)))
        print(
            "Packets lost on current channel_____________________{}".format(
                observer >> 4
            )
        )
        print(
            "Retry attempts made for last transmission___________{}".format(
                observer & 0xF
            )
        )
        print(
            "IRQ on Data Ready__{}    Data Ready___________{}".format(
                "_Enabled" if not self._config & 0x40 else "Disabled", self.irq_dr
            )
        )
        print(
            "IRQ on Data Fail___{}    Data Failed__________{}".format(
                "_Enabled" if not self._config & 0x10 else "Disabled", self.irq_df
            )
        )
        print(
            "IRQ on Data Sent___{}    Data Sent____________{}".format(
                "_Enabled" if not self._config & 0x20 else "Disabled", self.irq_ds
            )
        )
        print(
            "TX FIFO full__________{}    TX FIFO empty________{}".format(
                "_True" if _fifo & 0x20 else "False",
                "True" if _fifo & 0x10 else "False",
            )
        )
        print(
            "RX FIFO full__________{}    RX FIFO empty________{}".format(
                "_True" if _fifo & 2 else "False",
                "True" if _fifo & 1 else "False",
            )
        )
        print(
            "Ask no ACK_________{}    Custom ACK Payload___{}".format(
                "_Allowed" if self._features & 1 else "Disabled",
                "Enabled" if self._features & 2 else "Disabled",
            )
        )
        print(
            "Dynamic Payloads___{}    Auto Acknowledgment__{}".format(
                "_Enabled"
                if self._dyn_pl == 0x3F
                else (
                    bin(self._dyn_pl).replace(
                        "b", "b" + "0" * (8 - len(bin(self._dyn_pl)))
                    )
                    if self._dyn_pl
                    else "Disabled"
                ),
                "Enabled"
                if self._aa == 0x3F
                else (
                    bin(self._aa).replace("b", "b" + "0" * (8 - len(bin(self._aa))))
                    if self._aa
                    else "Disabled"
                ),
            )
        )
        print(
            "Primary Mode_____________{}X    Power Mode___________{}".format(
                "R" if self._config & 1 else "T",
                ("Standby-II" if self.ce_pin.value else "Standby-I")
                if self._config & 2
                else "Off",
            )
        )
        if dump_pipes:
            self._dump_pipes()

    def _dump_pipes(self):
        self._open_pipes = self._reg_read(NRF24L01_REGISTERS.EN_RX)
        self._tx_address = self._reg_read_bytes(NRF24L01_REGISTERS.TX_ADDR)
        for i in range(6):
            if i < 2:
                self._pipes[i] = self._reg_read_bytes(NRF24L01_REGISTERS.RX_ADDR + i)
            else:
                self._pipes[i] = self._reg_read(NRF24L01_REGISTERS.RX_ADDR + i)
            self._pl_len[i] = self._reg_read(NRF24L01_REGISTERS.RX_PW + i)
        print("TX address____________", bytes(self.address()))
        for i in range(6):
            is_open = self._open_pipes & (1 << i)
            print(
                "Pipe {} ({}) bound: {}".format(
                    i,
                    " open " if is_open else "closed",
                    bytes(self.address(i)),
                )
            )
            if is_open and not self._dyn_pl & (1 << i):
                print("\t\texpecting {} byte static payloads".format(self._pl_len[i]))

    @property
    def is_plus_variant(self):
        """A `bool` descibing if the nRF24L01 is a plus variant or not (read-only)."""
        return self._is_plus_variant

    @property
    def dynamic_payloads(self):
        """This `bool` attribute controls the nRF24L01's dynamic payload length
        feature."""
        self._dyn_pl = self._reg_read(NRF24L01_REGISTERS.DYNPD)
        return self._dyn_pl

    @dynamic_payloads.setter
    def dynamic_payloads(self, enable):
        self._features = self._reg_read(NRF24L01_REGISTERS.FEATURE)
        if isinstance(enable, bool):
            self._dyn_pl = 0x3F if enable else 0
        elif isinstance(enable, int):
            self._dyn_pl = 0x3F & enable
        elif isinstance(enable, (list, tuple)):
            self._dyn_pl = self._reg_read(NRF24L01_REGISTERS.DYNPD)
            for i, val in enumerate(enable):
                if i < 6 and val >= 0:  # skip pipe if val is negative
                    self._dyn_pl = (self._dyn_pl & ~(1 << i)) | (bool(val) << i)
        else:
            raise ValueError("dynamic_payloads: {} is an invalid input" % enable)
        self._features = (self._features & 3) | (bool(self._dyn_pl) << 2)
        self._reg_write(NRF24L01_REGISTERS.FEATURE, self._features)
        self._reg_write(NRF24L01_REGISTERS.DYNPD, self._dyn_pl)

    def set_dynamic_payloads(self, enable, pipe_number=None):
        """Control the dynamic payload feature for a specific data pipe."""
        if pipe_number is None:
            self.dynamic_payloads = bool(enable)
        elif 0 <= pipe_number <= 5:
            self._dyn_pl = self._reg_read(NRF24L01_REGISTERS.DYNPD) & ~(1 << pipe_number)
            self.dynamic_payloads = self._dyn_pl | (bool(enable) << pipe_number)
        else:
            raise IndexError("pipe_number must be in range [0, 5]")

    def get_dynamic_payloads(self, pipe_number=0):
        """Returns a `bool` describing the dynamic payload feature about a pipe."""
        if 0 <= pipe_number <= 5:
            return bool(self.dynamic_payloads & (1 << pipe_number))
        raise IndexError("pipe_number must be in range [0, 5]")

    @property
    def payload_length(self):
        """This `int` attribute specifies the length (in bytes) of statically sized
        payload"""
        return self._pl_len[0]

    @payload_length.setter
    def payload_length(self, length):
        if isinstance(length, int):
            length = [max(1, length)] * 6
        elif not isinstance(length, (list, tuple)):
            raise ValueError("length {} is not a valid input".format(length))
        for i, val in enumerate(length):
            if i < 6 and val > 0:  # don't throw exception, just skip pipe
                self._pl_len[i] = min(32, val)
                self._reg_write(NRF24L01_REGISTERS.RX_PW + i, self._pl_len[i])

    def set_payload_length(self, length, pipe_number=None):
        """Sets the static payload length feature for each/all data pipes."""
        if pipe_number is None:
            self.payload_length = length
        else:
            self._pl_len[pipe_number] = max(1, min(32, length))
            self._reg_write(NRF24L01_REGISTERS.RX_PW + pipe_number, length)

    def get_payload_length(self, pipe_number=0):
        """Returns an `int` describing the specified data pipe's static
        payload length."""
        self._pl_len[pipe_number] = self._reg_read(NRF24L01_REGISTERS.RX_PW + pipe_number)
        return self._pl_len[pipe_number]

    @property
    def arc(self):
        """This `int` attribute specifies the nRF24L01's number of attempts to re-transmit TX
        payload when acknowledgment packet is not received."""
        self._retry_setup = self._reg_read(NRF24L01_REGISTERS.SETUP_RETR)
        return self._retry_setup & 0x0F

    @arc.setter
    def arc(self, count):
        count = max(0, min(int(count), 15))
        self._retry_setup = (self._retry_setup & 0xF0) | count
        self._reg_write(NRF24L01_REGISTERS.SETUP_RETR, self._retry_setup)

    @property
    def ard(self):
        """This `int` attribute specifies the nRF24L01's delay (in microseconds) between attempts to
        automatically re-transmit the TX payload when an expected acknowledgement (ACK) packet is
        not received."""
        self._retry_setup = self._reg_read(NRF24L01_REGISTERS.SETUP_RETR)
        return ((self._retry_setup & 0xF0) >> 4) * 250 + 250

    @ard.setter
    def ard(self, delta):
        delta = max(250, min(delta, 4000))
        self._retry_setup = (self._retry_setup & 15) | int((delta - 250) / 250) << 4
        self._reg_write(NRF24L01_REGISTERS.SETUP_RETR, self._retry_setup)

    def set_auto_retries(self, delay, count):
        """Set the :attr:`ard` & :attr:`arc` attributes with 1 function."""
        delay = int((max(250, min(delay, 4000)) - 250) / 250) << 4
        self._retry_setup = delay | max(0, min(int(count), 15))
        self._reg_write(NRF24L01_REGISTERS.SETUP_RETR, self._retry_setup)

    def get_auto_retries(self):
        """Get the :attr:`ard` & :attr:`arc` attributes with 1 function."""
        return (self.ard, self._retry_setup & 0x0F)

    @property
    def last_tx_arc(self):
        """Return the number of attempts made for last transission (read-only)."""
        return self._reg_read(8) & 0x0F

    @property
    def auto_ack(self):
        """This `bool` attribute controls the nRF24L01's automatic acknowledgment
        feature."""
        self._aa = self._reg_read(NRF24L01_REGISTERS.EN_AA)
        return self._aa

    @auto_ack.setter
    def auto_ack(self, enable):
        if isinstance(enable, bool):
            self._aa = 0x3F if enable else 0
        elif isinstance(enable, int):
            self._aa = 0x3F & enable
        elif isinstance(enable, (list, tuple)):
            self._aa = self._reg_read(NRF24L01_REGISTERS.EN_AA)
            for i, val in enumerate(enable):
                if i < 6 and val >= 0:  # skip pipe if val is negative
                    self._aa = (self._aa & ~(1 << i)) | (bool(val) << i)
        else:
            raise ValueError("auto_ack: {} is not a valid input".format(enable))
        self._reg_write(NRF24L01_REGISTERS.EN_AA, self._aa)

    def set_auto_ack(self, enable, pipe_number):
        """Control the :attr:`auto_ack` feature for a specific data pipe."""
        if pipe_number is None:
            self.auto_ack = bool(enable)
        elif 0 <= pipe_number <= 5:
            self._aa = self._reg_read(NRF24L01_REGISTERS.EN_AA) & ~(1 << pipe_number)
            self.auto_ack = self._aa | (bool(enable) << pipe_number)
        else:
            raise IndexError("pipe_number must be in range [0, 5]")

    def get_auto_ack(self, pipe_number):
        """Returns a `bool` describing the :attr:`auto_ack` feature about a data pipe."""
        if 0 <= pipe_number <= 5:
            self._aa = self._reg_read(NRF24L01_REGISTERS.EN_AA)
            return bool(self._aa & (1 << pipe_number))
        raise IndexError("pipe_number must be in range [0, 5]")

    @property
    def ack(self):
        """This `bool` attribute represents the status of the nRF24L01's capability to use custom
        payloads as part of the automatic acknowledgment (ACK) packet."""
        self._aa = self._reg_read(NRF24L01_REGISTERS.EN_AA)
        self._dyn_pl = self._reg_read(NRF24L01_REGISTERS.DYNPD)
        self._features = self._reg_read(NRF24L01_REGISTERS.FEATURE)
        return bool((self._features & 6) == 6 and ((self._aa & self._dyn_pl) & 1))

    @ack.setter
    def ack(self, enable):
        if bool(enable):
            self.set_auto_ack(True, 0)
            self._dyn_pl = self._dyn_pl & 0x3E | 1
            self._reg_write(NRF24L01_REGISTERS.DYNPD, self._dyn_pl)
            self._features = self._features | 4
        self._features = self._features & 5 | bool(enable) << 1
        self._reg_write(NRF24L01_REGISTERS.FEATURE, self._features)

    def load_ack(self, buf, pipe_number):
        """This allows the MCU to specify an ACK payload to be allocated into the TX FIFO buffer for use
        on a specific data pipe."""
        if pipe_number < 0 or pipe_number > 5:
            raise IndexError("pipe_number must be in range [0, 5]")
        if not buf or len(buf) > 32:
            raise ValueError("payload must have a byte length in range [1, 32]")
        if not bool((self._features & 6) == 6 and ((self._aa & self._dyn_pl) & 1)):
            self.ack = True
        if not self.tx_full:
            self._reg_write_bytes(0xA8 | pipe_number, buf)
            return True
        return False

    @property
    def allow_ask_no_ack(self):
        """Allow or disable ``ask_no_ack`` parameter to :meth:`send()` & :meth:`write()`."""
        self._features = self._reg_read(NRF24L01_REGISTERS.FEATURE)
        return bool(self._features & 1)

    @allow_ask_no_ack.setter
    def allow_ask_no_ack(self, enable):
        self._features = self._reg_read(NRF24L01_REGISTERS.FEATURE) & 6 | bool(enable)
        self._reg_write(NRF24L01_REGISTERS.FEATURE, self._features)

    @property
    def data_rate(self):
        """This `int` attribute specifies the nRF24L01's frequency data rate for OTA (over the air)
        transmissions."""
        self._rf_setup = self._reg_read(NRF24L01_REGISTERS.RF_SETUP)
        rf_setup = self._rf_setup & 0x28
        return (2 if rf_setup == 8 else 250) if rf_setup else 1

    @data_rate.setter
    def data_rate(self, speed):
        if not speed in (1, 2, 250):
            raise ValueError("data_rate must be 1 (Mbps), 2 (Mbps), or 250 (kbps)")
        if not self._is_plus_variant and speed == 250:
            raise NotImplementedError(
                "250 kbps data rate is not available for the non-plus "
                "variants of the nRF24L01 transceivers."
            )
        speed = 0 if speed == 1 else (0x20 if speed != 2 else 8)
        self._rf_setup = self._reg_read(NRF24L01_REGISTERS.SETUP_RETR) & 0xD7 | speed
        self._reg_write(NRF24L01_REGISTERS.SETUP_RETR, self._rf_setup)

    @property
    def channel(self):
        """This `int` attribute specifies the nRF24L01's frequency (in 2400 +
        `channel` MHz)."""
        return self._reg_read(NRF24L01_REGISTERS.RF_CH)

    @channel.setter
    def channel(self, channel):
        if not 0 <= int(channel) <= 125:
            raise ValueError("channel can only be set in range [0, 125]")
        self._channel = channel
        self._reg_write(NRF24L01_REGISTERS.RF_CH, channel)  # always writes to reg

    @property
    def crc(self):
        """This `int` attribute specifies the nRF24L01's CRC (cyclic redundancy checking) encoding
        scheme in terms of byte length."""
        self._config = self._reg_read(NRF24L01_REGISTERS.CONFIG)
        self._aa = self._reg_read(NRF24L01_REGISTERS.EN_AA)
        if self._aa:
            return 2 if self._config & 4 else 1
        return max(0, ((self._config & 0x0C) >> 2) - 1)

    @crc.setter
    def crc(self, length):
        length = min(2, abs(int(length)))
        length = (length + 1) << 2 if length else 0
        self._config = self._config & 0x73 | length
        self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)

    @property
    def power(self):
        """This `bool` attribute controls the power state of the nRF24L01."""
        self._config = self._reg_read(NRF24L01_REGISTERS.CONFIG)
        return bool(self._config & 2)

    @power.setter
    def power(self, is_on):
        self._config = self._reg_read(NRF24L01_REGISTERS.CONFIG) & 0x7D | bool(is_on) << 1
        self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)
        time.sleep(0.00016)

    @property
    def pa_level(self):
        """This `int` attribute specifies the nRF24L01's power amplifier level (in
        dBm)."""
        self._rf_setup = self._reg_read(NRF24L01_REGISTERS.RF_SETUP)
        return (3 - ((self._rf_setup & 0x06) >> 1)) * -6

    @pa_level.setter
    def pa_level(self, power):
        lna_bit = True
        if isinstance(power, (list, tuple)) and len(power) > 1:
            lna_bit, power = bool(power[1]), int(power[0])
        if power not in (-18, -12, -6, 0):
            raise ValueError("pa_level must be -18, -12, -6, or 0 (in dBm)")
        power = (3 - int(power / -6)) * 2
        self._rf_setup = (self._rf_setup & 0xF8) | power | lna_bit
        self._reg_write(NRF24L01_REGISTERS.RF_SETUP, self._rf_setup)

    @property
    def is_lna_enabled(self):
        """A read-only `bool` attribute about the LNA gain feature."""
        self._rf_setup = self._reg_read(NRF24L01_REGISTERS.RF_SETUP)
        return bool(self._rf_setup & 1)

    def resend(self, send_only=False):
        """Use this function to maunally re-send the previously failed-to-transmit payload in the
        top level (first out) of the TX FIFO buffer."""
        result = False
        if self.fifo(True, True):
            return result
        self.ce_pin.value = 0
        if not send_only and (self._status >> 1) < 6:
            self.flush_rx()
        self.clear_status_flags()
        # self._reg_write(0xE3)
        self.ce_pin.value = 1
        up_cnt = 0
        while self._spi is not None and not self._status & 0x30:
            up_cnt += self.update()
        self.ce_pin.value = 0
        result = bool(self._status & 0x20)
        # print("resend() waited {} updates DS: {} DR: {} DF: {}".format(
        #     up_cnt, self.irq_ds, self.irq_dr, self.irq_df
        # ))
        if result and self._status & 0x40 and not send_only:
            return self.read()
        return result

    def write(self, buf=None, ask_no_ack=False, write_only=False):
        """This non-blocking function (when used as alternative to :meth:`send()`) is meant for
        asynchronous applications and can only handle one payload at a time as it is a helper
        function to :meth:`send()`."""
        if not buf or len(buf) > 32:
            raise ValueError("buffer must have a length in range [1, 32]")
        self.clear_status_flags()
        if self._status & 1:
            return False
        is_power_up = self._config & 2
        if self._config & 3 != 2:  # is radio powered up in TX mode?
            self._config = (self._config & 0x7C) | 2
            self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)
            if not is_power_up:
                time.sleep(0.00016)
        if not bool((self._dyn_pl & 1) and (self._features & 4)):
            if len(buf) < self._pl_len[0]:
                buf += b"\0" * (self._pl_len[0] - len(buf))
            elif len(buf) > self._pl_len[0]:
                buf = buf[: self._pl_len[0]]
        if ask_no_ack and self._features & 1 == 0:
            self._features = self._features & 0xFE | 1
            self._reg_write(NRF24L01_REGISTERS.FEATURE, self._features)
        self._reg_write_bytes(0xA0 | (bool(ask_no_ack) << 4), buf)
        if not write_only:
            self.ce_pin.value = 1
        return True

    def flush_rx(self):
        """A helper function to flush the nRF24L01's internal RX FIFO buffer
        (write-only)."""
        self._reg_write(0xE2)

    def flush_tx(self):
        """A helper function to flush the nRF24L01's internal TX FIFO buffer
        (write-only)."""
        self._reg_write(0xE1)

    def fifo(self, about_tx=False, check_empty=None):
        """This provides some precision determining the status of the TX/RX FIFO buffers
        (read-only)."""
        _fifo, about_tx = (self._reg_read(0x17), bool(about_tx))
        if check_empty is None:
            return (_fifo & (0x30 if about_tx else 0x03)) >> (4 * about_tx)
        return bool(_fifo & ((2 - bool(check_empty)) << (4 * about_tx)))

    def address(self, index=-1):
        """Returns the current TX address or optionally RX address. (read-only)"""
        if index > 5:
            raise IndexError("index {} is out of bounds [0,5]".format(index))
        if index < 0:
            return self._tx_address
        if index <= 1:
            return self._pipes[index]
        return bytes([self._pipes[index]]) + self._pipes[1][1:]

    @property
    def rpd(self):
        """Returns `True` if signal was detected or `False` if not. (read-only)"""
        return bool(self._reg_read(0x09))

    def start_carrier_wave(self):
        """Starts a continuous carrier wave test."""
        self.power = 0
        self.ce_pin.value = 0
        self.power = 1
        self.listen = 0
        self._rf_setup |= 0x90
        self._reg_write(NRF24L01_REGISTERS.RF_SETUP, self._rf_setup)
        if not self.is_plus_variant:
            self.auto_ack = False
            self._retry_setup = 0
            self._reg_write(NRF24L01_REGISTERS.SETUP_RETR, self._retry_setup)
            self._tx_address = bytearray([0xFF] * 5)
            self._reg_write_bytes(NRF24L01_REGISTERS.TX_ADDR, self._tx_address)
            self._reg_write_bytes(0xA0, b"\xFF" * 32)
            self.crc = 0
            self.ce_pin.value = 1
            time.sleep(0.001)
            self.ce_pin.value = 0
            self.clear_status_flags()
            self._reg_write(0x17, 0x40)
        self.ce_pin.value = 1

    def stop_carrier_wave(self):
        """Stops a continuous carrier wave test."""
        self.ce_pin.value = 0
        self.power = 0
        self._rf_setup &= ~0x90
        self._reg_write(NRF24L01_REGISTERS.RF_SETUP, self._rf_setup)
