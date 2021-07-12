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
    """A driver class for the nRF24L01(+) transceiver radios. This class aims to be compatible with
    other devices in the nRF24xxx product line that implement the Nordic proprietary Enhanced
    ShockBurst Protocol (and/or the legacy ShockBurst Protocol), but officially only supports
    (through testing) the nRF24L01 and nRF24L01+ devices.

    See also the section on :ref:`spi_args`. Note that the *select_pin* argument is the pin on the
    Raspberry Pi connected to the *CSN* pin on the nRF24L01 transceiver (Default is GPIO8).

    :type ce_pin: int or str
    :param ce_pin: The digital output pin that is connected to the nRF24L01's CE (Chip Enable) pin.
        This is required.
    """
    def __init__(self, ce_pin,
                 **spi_args):
        self._payload_length = 32  # inits internal attribute
        self.payload_length = self._payload_length
        # last address assigned to pipe0 for reading. init to None
        self._fifo = 0
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
        self._config = self._reg_read(NRF24L01_REGISTERS.CONFIG)
        if self._config & 3 == 2: # if in TX + standby-I mode
            self.power = False  # power down
        else: # hardware presence check NOT passed
            print(bin(self._config))
            raise RuntimeError("nRF24L01 Hardware not responding")

        for i in range(6):  # capture RX addresses from registers
            if i < 2:
                self._pipes[i] = self._reg_read_bytes(NRF24L01_REGISTERS.RX_ADDR + i)
            else:
                self._pipes[i] = self._reg_read(NRF24L01_REGISTERS.RX_ADDR + i)

        # store the ce pin
        self.ce_pin = OutputDevice(pin=ce_pin, pin_factory=self.pin_factory)
        # reset ce.value & disable the chip comms
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

        with self:  # write to registers & power up
            # using __enter__() configures all virtual features and settings to the hardware
            # registers
            self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config | 1)  # enable RX mode
            time.sleep(0.000015)  # wait time for transitioning modes RX/TX
            self.flush_rx()  # spec sheet say "used in RX mode"
            self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config & 0xC)  # power down + TX mode
            time.sleep(0.000015)  # wait time for transitioning modes RX/TX
            self.flush_tx()  # spec sheet say "used in TX mode"
            self.clear_status_flags()  # writes directly to STATUS register

    def __enter__(self):
        self.ce_pin.value = 0  # ensure standby-I mode to write to CONFIG register
        self._config |= 2
        self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)
        # time.sleep(0.00015)  # let the rest of this function be the delay
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
        return self

    def __exit__(self, *exc):
        self.ce_pin.value = 0  # ensure standby-I mode to write to CONFIG register
        self._config &= 0x7D  # power off radio
        self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)
        time.sleep(0.00016)
        return False

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
        if isinstance(out_buf, bytearray):
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
        """This `int` attribute specifies the length (in bytes) of addresses to be used for RX/TX
        pipes. The addresses assigned to the data pipes must have byte length equal to the value
        set for this attribute.

        A valid input value must be an `int` in range [3,5]. Otherwise a `ValueError` exception is
        thrown. Default is set to the nRF24L01's maximum of 5.
        """
        self._addr_len = self._reg_read(NRF24L01_REGISTERS.SETUP_AW) + 2
        return self._addr_len

    @address_length.setter
    def address_length(self, length):
        if not 3 <= length <= 5:
            raise ValueError("address_length can only be set in range [3, 5] bytes")
        self._addr_len = int(length)
        self._reg_write(NRF24L01_REGISTERS.SETUP_AW, length - 2)

    def open_tx_pipe(self, address):
        """This function is used to open a data pipe for OTA (over the air) TX transmissions.

        :param bytearray address: The virtual address of the receiving nRF24L01. This must have a
            length equal to the `address_length` attribute (see `address_length` attribute).
            Otherwise a `ValueError` exception is thrown. The address specified here must match the
            address set to one of the RX data pipes of the receiving nRF24L01.

        .. note:: There is no option to specify which data pipe to use because the nRF24L01 only
            uses data pipe 0 in TX mode. Additionally, the nRF24L01 uses the same data pipe (pipe
            0) for receiving acknowledgement (ACK) packets in TX mode when the `auto_ack` attribute
            is enabled. Thus, RX pipe 0 is appropriated with the TX address (specified here) when
            `auto_ack` is set to `True`.
        """
        if self._aa & 1:
            for i, val in enumerate(address):
                self._pipes[0][i] = val
            self._reg_write_bytes(NRF24L01_REGISTERS.RX_ADDR, address)
        for i, val in enumerate(address):
            self._tx_address[i] = val
        self._reg_write_bytes(NRF24L01_REGISTERS.TX_ADDR, address)

    def close_rx_pipe(self, pipe_number, reset=True):
        """This function is used to close a specific data pipe from OTA (over the air) RX
        transmissions.

        :param int pipe_number: The data pipe to use for RX transactions. This must be in range
            [0,5]. Otherwise a `ValueError` exception is thrown.
        :param bool reset: `True` resets the address for the specified ``pipe_number`` to the
            factory address (different for each pipe). `False` leaves the address on the specified
            ``pipe_number`` alone. Be aware that the addresses will remain despite loss of power.
        """
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
        transmissions. If `dynamic_payloads` attribute is `False`, then the `payload_length`
        attribute is used to specify the expected length of the RX payload on the specified data
        pipe.

        :param int pipe_number: The data pipe to use for RX transactions. This must be in range
            [0,5]. Otherwise a `ValueError` exception is thrown.
        :param bytearray address: The virtual address to the receiving nRF24L01. This must have a
            byte length equal to the `address_length` attribute. Otherwise a `ValueError`
            exception is thrown. If using a ``pipe_number`` greater than 1, then only the MSByte
            of the address is written, so make sure MSByte (first character) is unique among other
            simultaneously receiving addresses).

        .. note:: The nRF24L01 shares the addresses' LSBytes (address[1:5]) on data pipes 2 through
            5. These shared LSBytes are determined by the address set to pipe 1.
        """
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
        """An attribute to represent the nRF24L01 primary role as a radio.

        Setting this attribute incorporates the proper transitioning to/from RX mode as it involves
        playing with the `power` attribute and the nRF24L01's CE pin. This attribute does not power
        down the nRF24L01, but will power it up when needed; use `power` attribute set to `False`
        to put the nRF24L01 to sleep.

        A valid input value is a `bool` in which:

            `True` enables RX mode. Additionally, per `Appendix B of the nRF24L01+ Specifications
            Sheet
            <https://www.sparkfun.com/datasheets/Components/SMD/
            nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf#G1091756>`_, this attribute
            flushes the RX FIFO, clears the `irq_dr` status flag, and puts nRF24L01 in power up
            mode. Notice the CE pin is be held HIGH during RX mode.

            `False` disables RX mode. As mentioned in above link, this puts nRF24L01's power in
            Standby-I (CE pin is LOW meaning low current & no transmissions) mode which is ideal
            for post-reception work. Disabing RX mode doesn't flush the RX/TX FIFO buffers, so
            remember to flush your 3-level FIFO buffers when appropriate using `flush_tx()` or
            `flush_rx()` (see also the `recv()` function).
        """
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
        """This function checks if the nRF24L01 has received any data at all. Internally, this
        function uses `pipe()` then reports the next available payload's length (in bytes) -- if
        there is any.

        :returns:
            - `int` of the size (in bytes) of an available RX payload (if any).
            - ``0`` if there is no payload in the RX FIFO buffer.
        """
        if self.available():
            if self._features & 4:
                return self._reg_read(0x60)
            return self._pl_len[(self._status >> 1) & 7]
        return 0

    def read(self, length=None):
        """This function is used to retrieve the next available payload in the RX FIFO buffer, then
        clears the `irq_dr` status flag. This function also serves as a helper function to
        `read_ack()` in TX mode to aquire any custom payload in the automatic acknowledgement (ACK)
        packet -- only when the `ack` attribute is enabled.

        :returns: A `bytearray` of the RX payload data

            - If the `dynamic_payloads` attribute is disabled, then the returned bytearray's length
              is equal to the user defined `payload_length` attribute (which defaults to 32).
            - If the `dynamic_payloads` attribute is enabled, then the returned bytearray's length
              is equal to the payload's length

        .. tip:: Call the `any()` function before calling `recv()` to verify that there is data to
            fetch. If there's no data to fetch, then the nRF24L01 returns bogus data and should not
            regarded as a valid payload.
        """
        return_size = length if length is not None else self.any()
        if not return_size:
            return None
        result = self._reg_read_bytes(0x61, return_size)
        self.clear_status_flags(True, False, False)
        return result

    def send(self, buf, ask_no_ack=False, force_retry=0, send_only=False):
        """This blocking function is used to transmit payload(s).

        :returns:
            * `list` if a list or tuple of payloads was passed as the ``buf`` parameter. Each item
              in the returned list will contain the returned status for each corresponding payload
              in the list/tuple that was passed. The return statuses will be in one of the
              following forms:
            * `False` if transmission fails.
            * `True` if transmission succeeds.
            * `bytearray` when the `ack` attribute is `True`, the payload expects a responding
              custom ACK payload; the response is returned (upon successful transmission) as a
              `bytearray`. Empty ACK payloads (upon successful transmission) when the `ack`
              attribute is set `True` are replaced with an error message ``b'NO ACK RETURNED'``.
            * `None` if transmission times out meaning nRF24L01 has malfunctioned. This condition
              is very rare. The allowed time for transmission is calculated using `table 18 in the
              nRF24L01 specification sheet <https://www.sparkfun.com/datasheets/Components/SMD/
              nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf#G1123001>`_

        :param bytearray,list,tuple buf: The payload to transmit. This bytearray must have a length
            greater than 0 and less than 32, otherwise a `ValueError` exception is thrown. This can
            also be a list or tuple of payloads (`bytearray`); in which case, all items in the
            list/tuple are processed for consecutive transmissions.

            - If the `dynamic_payloads` attribute is disabled and this bytearray's length is less
              than the `payload_length` attribute, then this bytearray is padded with zeros until
              its length is equal to the `payload_length` attribute.
            - If the `dynamic_payloads` attribute is disabled and this bytearray's length is
              greater than `payload_length` attribute, then this bytearray's length is truncated to
              equal the `payload_length` attribute.
        :param bool ask_no_ack: Pass this parameter as `True` to tell the nRF24L01 not to wait for
            an acknowledgment from the receiving nRF24L01. This parameter directly controls a
            ``NO_ACK`` flag in the transmission's Packet Control Field (9 bits of information about
            the payload). Therefore, it takes advantage of an nRF24L01 feature specific to
            individual payloads, and its value is not saved anywhere. You do not need to specify
            this for every payload if the `auto_ack` attribute is disabled, however this parameter
            should work despite the `auto_ack` attribute's setting.

            .. note:: Each transmission is in the form of a packet. This packet contains sections
                of data around and including the payload. `See Chapter 7.3 in the nRF24L01
                Specifications Sheet <https://www.sparkfun.com/datasheets/Components/SMD/
                nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf#G1136318>`_ for more
                details.

        .. tip:: It is highly recommended that `auto_ack` attribute is enabled when sending
            multiple payloads. Test results with the `auto_ack` attribute disabled were very poor
            (much < 50% received). This same advice applies to the ``ask_no_ack`` parameter (leave
            it as `False` for multiple payloads).

        .. warning::  The nRF24L01 will block usage of the TX FIFO buffer upon failed
            transmissions. Failed transmission's payloads stay in TX FIFO buffer until the MCU
            calls `flush_tx()` and `clear_status_flags()`. Therefore, this function will discard
            failed transmissions' payloads when sending a list or tuple of payloads, so it can
            continue to process through the list/tuple even if any payload fails to be
            acknowledged.

        .. note:: We've tried very hard to keep nRF24L01s driven by CircuitPython devices compliant
            with nRF24L01s driven by the Raspberry Pi. But due to the Raspberry Pi's seemingly
            slower SPI speeds, we've had to resort to internally deploying `resend()` twice (at
            most when needed) for payloads that failed during multi-payload processing. This tactic
            is meant to slow down CircuitPython devices just enough for the Raspberry Pi to catch
            up. Transmission failures are less possible this way.
        """
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
        is full. (read-only)

        Calling this does not execute an SPI transaction. It only exposes that latest data
        contained in the STATUS byte that's always returned from any SPI transactions with the
        nRF24L01. Use the `update()` function to manually refresh this data when needed.

        :returns:
            * `True` for TX FIFO buffer is full
            * `False` for TX FIFO buffer is not full. This doesn't mean the TX FIFO buffer is
              empty.
        """
        return bool(self._status & 1)

    def update(self):
        """This function is only used to get an updated status byte over SPI from the nRF24L01 and
        is exposed to the MCU for asynchronous applications. Refreshing the status byte is vital to
        checking status of the interrupts, RX pipe number related to current RX payload, and if the
        TX FIFO buffer is full. This function returns nothing, but internally updates the `irq_dr`,
        `irq_ds`, `irq_df`, and `tx_full` attributes. Internally this is a helper function to
        `pipe()`, `send()`, and `resend()` functions"""
        # perform non-operation to get status byte
        # should be faster than reading the STATUS register
        self._reg_write(0xFF)

    @property
    def pipe(self):
        """This function returns information about the data pipe that received the next available
        payload in the RX FIFO buffer.

        :returns:
            - `None` if there is no payload in RX FIFO.
            - The `int` identifying pipe number [0,5] that received the next available payload in
              the RX FIFO buffer.
        """
        result = (self._status & 0x0E) >> 1
        if result <= 5:
            return result
        return None

    @property
    def irq_dr(self):
        """A `bool` that represents the "Data Ready" interrupted flag. (read-only)

        * `True` represents Data is in the RX FIFO buffer
        * `False` represents anything depending on context (state/condition of FIFO buffers) --
          usually this means the flag's been reset.

        Pass ``dataReady`` parameter as `True` to `clear_status_flags()` and reset this. As this is
        a virtual representation of the interrupt event, this attribute will always be updated
        despite what the actual IRQ pin is configured to do about this event.

        Calling this does not execute an SPI transaction. It only exposes that latest data
        contained in the STATUS byte that's always returned from any other SPI transactions. Use
        the `update()` function to manually refresh this data when needed.
        """
        return bool(self._status & 0x40)

    @property
    def irq_ds(self):
        """A `bool` that represents the "Data Sent" interrupted flag. (read-only)

        * `True` represents a successful transmission
        * `False` represents anything depending on context (state/condition of FIFO buffers) --
          usually this means the flag's been reset.

        Pass ``dataSent`` parameter as `True` to `clear_status_flags()` to reset this. As this is a
        virtual representation of the interrupt event, this attribute will always be updated
        despite what the actual IRQ pin is configured to do about this event.

        Calling this does not execute an SPI transaction. It only exposes that latest data
        contained in the STATUS byte that's always returned from any other SPI transactions. Use
        the `update()` function to manually refresh this data when needed.
        """
        return bool(self._status & 0x20)

    @property
    def irq_df(self):
        """A `bool` that represents the "Data Failed" interrupted flag. (read-only)

        * `True` signifies the nRF24L01 attemped all configured retries
        * `False` represents anything depending on context (state/condition) -- usually this means
          the flag's been reset.

        Pass ``dataFail`` parameter as `True` to `clear_status_flags()` to reset this. As this is a
        virtual representation of the interrupt event, this attribute will always be updated
        despite what the actual IRQ pin is configured to do about this event.see also the `arc` and
        `ard` attributes.

        Calling this does not execute an SPI transaction. It only exposes that latest data
        contained in the STATUS byte that's always returned from any other SPI transactions. Use
        the `update()` function to manually refresh this data when needed.
        """
        return bool(self._status & 0x10)

    def clear_status_flags(self, data_recv=True, data_sent=True, data_fail=True):
        """This clears the interrupt flags in the status register. Internally, this is
        automatically called by `send()`, `write()`, `recv()`, and when `listen` changes from
        `False` to `True`.

        :param bool data_recv: specifies wheather to clear the "RX Data Ready" flag.
        :param bool data_sent: specifies wheather to clear the "TX Data Sent" flag.
        :param bool data_fail: specifies wheather to clear the "Max Re-transmit reached" flag.

        .. note:: Clearing the ``data_fail`` flag is necessary for continued transmissions from the
            nRF24L01 (locks the TX FIFO buffer when `irq_df` is `True`) despite wheather or not the
            MCU is taking advantage of the interrupt (IRQ) pin. Call this function only when there
            is an antiquated status flag (after you've dealt with the specific payload related to
            the staus flags that were set), otherwise it can cause payloads to be ignored and
            occupy the RX/TX FIFO buffers. See `Appendix A of the nRF24L01+ Specifications Sheet
            <https://www.sparkfun.com/datasheets/Components/SMD/
            nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf#G1047965>`_ for an outline of
            proper behavior.
        """
        # 0x07 = STATUS register; only bits 6 through 4 are write-able
        config = bool(data_recv) << 6 | bool(data_sent) << 5
        self._reg_write(7, config | bool(data_fail) << 4)

    def interrupt_config(self, data_recv=True, data_sent=True, data_fail=True):
        """Sets the configuration of the nRF24L01's IRQ (interrupt) pin. The signal from the
        nRF24L01's IRQ pin is active LOW. (write-only)

        :param bool data_recv: If this is `True`, then IRQ pin goes active when there is new data
            to read in the RX FIFO buffer.
        :param bool data_sent: If this is `True`, then IRQ pin goes active when a payload from TX
            buffer is successfully transmit.
        :param bool data_fail: If this is `True`, then IRQ pin goes active when maximum number of
            attempts to re-transmit the packet have been reached. If `auto_ack` attribute is
            disabled, then this IRQ event is not used.

        .. note:: To fetch the status (not configuration) of these IRQ flags, use the `irq_df`,
            `irq_ds`, `irq_dr` attributes respectively.

        .. tip:: Paraphrased from nRF24L01+ Specification Sheet:

            The procedure for handling ``data_recv`` IRQ should be:

            1. read payload through `recv()`
            2. clear ``dataReady`` status flag (taken care of by using `recv()` in previous step)
            3. read FIFO_STATUS register to check if there are more payloads available in RX FIFO
               buffer. (a call to `pipe()`, `any()` or even ``(False,True)`` as parameters to
               `fifo()` will get this result)
            4. if there is more data in RX FIFO, repeat from step 1
        """
        self._config = self._reg_read(NRF24L01_REGISTERS.CONFIG)  # refresh data
        # save to register and update local copy of pwr & RX/TX modes' flags
        self._config = (self._config & 0x0F) | (not data_fail << 4) | (not data_sent << 5) | \
            (not data_recv << 6)
        self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)

    def print_details(self, dump_pipes=True):
        """This debuggung function aggregates and outputs all status/condition related information
        from the nRF24L01. Some information may be irrelevant depending on nRF24L01's
        state/condition.

        :prints:

            - ``Channel`` The current setting of the `channel` attribute
            - ``RF Data Rate`` The current setting of the RF `data_rate` attribute.
            - ``RF Power Amplifier`` The current setting of the `pa_level` attribute.
            - ``CRC bytes`` The current setting of the `crc` attribute
            - ``Address length`` The current setting of the `address_length` attribute
            - ``Payload lengths`` The current setting of the `payload_length` attribute
            - ``Auto retry delay`` The current setting of the `ard` attribute
            - ``Auto retry attempts`` The current setting of the `arc` attribute
            - ``Packets Lost`` Total amount of packets lost (transmission failures)
            - ``Retry Attempts Made`` Maximum amount of attempts to re-transmit during last
              transmission (resets per payload)
            - ``IRQ - Data Ready`` The current setting of the IRQ pin on "Data Ready" event
            - ``IRQ - Data Sent`` The current setting of the IRQ pin on "Data Sent" event
            - ``IRQ - Data Fail`` The current setting of the IRQ pin on "Data Fail" event
            - ``Data Ready`` Is there RX data ready to be read?
              (state of the `irq_dr` flag)
            - ``Data Sent`` Has the TX data been sent? (state of the `irq_ds` flag)
            - ``Data Failed`` Has the maximum attempts to re-transmit been reached?
              (state of the `irq_df` flag)
            - ``TX FIFO full`` Is the TX FIFO buffer full? (state of the `tx_full` flag)
            - ``TX FIFO empty`` Is the TX FIFO buffer empty?
            - ``RX FIFO full`` Is the RX FIFO buffer full?
            - ``RX FIFO empty`` Is the RX FIFO buffer empty?
            - ``Custom ACK payload`` Is the nRF24L01 setup to use an extra (user defined) payload
              attached to the acknowledgment packet? (state of the `ack` attribute)
            - ``Ask no ACK`` Is the nRF24L01 setup to transmit individual packets that don't
              require acknowledgment?
            - ``Automatic Acknowledgment`` Is the `auto_ack` attribute enabled?
            - ``Dynamic Payloads`` Is the `dynamic_payloads` attribute enabled?
            - ``Primary Mode`` The current mode (RX or TX) of communication of the nRF24L01 device.
            - ``Power Mode`` The power state can be Off, Standby-I, Standby-II, or On.

        :param bool dump_pipes: `True` appends the output and prints:

            * the current address used for TX transmissions
            * ``Pipe [#] ([open/closed]) bound: [address]`` where ``#`` represent the pipe number,
              the ``open/closed`` status is relative to the pipe's RX status, and ``address`` is
              read directly from the nRF24L01 registers.
            * if the pipe is open, then the output also prints ``expecting [X] byte static
              payloads`` where ``X`` is the `payload_length` (in bytes) the pipe is setup to
              receive when `dynamic_payloads` is disabled.

            Default is `False` and skips this extra information.
        """
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
                ("Standby-II" if self.ce_pin else "Standby-I")
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
        print("TX address____________ 0x" + self.address())
        for i in range(6):
            is_open = self._open_pipes & (1 << i)
            print(
                "Pipe {} ({}) bound: {}".format(
                    i,
                    " open " if is_open else "closed",
                    "0x" + self.address(i),
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
        """This `bool` attribute controls the nRF24L01's dynamic payload length feature.

        - `True` enables nRF24L01's dynamic payload length feature. The `payload_length`
          attribute is ignored when this feature is enabled.
        - `False` disables nRF24L01's dynamic payload length feature. Be sure to adjust
          the `payload_length` attribute accordingly when `dynamic_payloads` feature is disabled.
        """
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
        """This `int` attribute specifies the length (in bytes) of payload that is regarded,
        meaning "how big of a payload should the radio care about?" If the `dynamic_payloads`
        attribute is enabled, this attribute has no affect. When `dynamic_payloads` is disabled,
        this attribute is used to specify the payload length when entering RX mode.

        A valid input value must be an `int` in range [1,32]. Otherwise a `ValueError` exception is
        thrown. Default is set to the nRF24L01's maximum of 32.

        .. note:: When `dynamic_payloads` is disabled during transmissions:

            - Payloads' size of greater than this attribute's value will be truncated to match.
            - Payloads' size of less than this attribute's value will be padded with zeros to
              match.
        """
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
        """"This `int` attribute specifies the nRF24L01's number of attempts to re-transmit TX
        payload when acknowledgment packet is not received. The nRF24L01 does not attempt to
        re-transmit if `auto_ack` attribute is disabled.

        A valid input value must be in range [0,15]. Otherwise a `ValueError` exception is thrown.
        Default is set to 3.
        """
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
        not received. During this time, the nRF24L01 is listening for the ACK packet. If the
        `auto_ack` attribute is disabled, this attribute is not applied.

        A valid input value must be a multiple of 250 in range [250,4000]. Otherwise a `ValueError`
        exception is thrown. Default is 1500 for reliability.

        .. note:: Paraphrased from nRF24L01 specifications sheet:

            Please take care when setting this parameter. If the custom ACK payload is more than 15
            bytes in 2 Mbps data rate, the `ard` must be 500 microseconds or more. If the custom ACK payload
            is more than 5 bytes in 1 Mbps data rate, the `ard` must be 500 microseconds or more. In 250kbps
            data rate (even when there is no custom ACK payload) the `ard` must be 500 microseconds or more.

            See `data_rate` attribute on how to set the data rate of the nRF24L01's transmissions.
        """
        self._retry_setup = self._reg_read(NRF24L01_REGISTERS.SETUP_RETR)
        return ((self._retry_setup & 0xF0) >> 4) * 250 + 250

    @ard.setter
    def ard(self, delta):
        delta = max(250, min(delta, 4000))
        self._retry_setup = (self._retry_setup & 15) | int((delta - 250) / 250) << 4
        self._reg_write(NRF24L01_REGISTERS.SETUP_RETR, self._retry_setup)

    def set_auto_retries(self, delay, count):
        """set the `ard` & `arc` attributes with 1 function."""
        delay = int((max(250, min(delay, 4000)) - 250) / 250) << 4
        self._retry_setup = delay | max(0, min(int(count), 15))
        self._reg_write(NRF24L01_REGISTERS.SETUP_RETR, self._retry_setup)

    def get_auto_retries(self):
        """get the `ard` & `arc` attributes with 1 function."""
        return (self.ard, self._retry_setup & 0x0F)

    @property
    def last_tx_arc(self):
        """Return the number of attempts made for last transission (read-only)."""
        return self._reg_read(8) & 0x0F

    @property
    def auto_ack(self):
        """This `bool` attribute controls the nRF24L01's automatic acknowledgment feature.

        - `True` enables automatic acknowledgment packets. The CRC (cyclic redundancy checking)
          is enabled automatically by the nRF24L01 if the `auto_ack` attribute is enabled (see also
          `crc` attribute).
        - `False` disables automatic acknowledgment packets. The `crc` attribute will
          remain unaffected (remains enabled) when disabling the `auto_ack` attribute.
        """
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
        """Control the `auto_ack` feature for a specific data pipe."""
        if pipe_number is None:
            self.auto_ack = bool(enable)
        elif 0 <= pipe_number <= 5:
            self._aa = self._reg_read(NRF24L01_REGISTERS.EN_AA) & ~(1 << pipe_number)
            self.auto_ack = self._aa | (bool(enable) << pipe_number)
        else:
            raise IndexError("pipe_number must be in range [0, 5]")

    def get_auto_ack(self, pipe_number):
        """Returns a `bool` describing the `auto_ack` feature about a data pipe."""
        if 0 <= pipe_number <= 5:
            self._aa = self._reg_read(NRF24L01_REGISTERS.EN_AA)
            return bool(self._aa & (1 << pipe_number))
        raise IndexError("pipe_number must be in range [0, 5]")

    @property
    def ack(self):
        """This `bool` attribute represents the status of the nRF24L01's capability to use custom
        payloads as part of the automatic acknowledgment (ACK) packet. Use this attribute to
        set/check if the custom ACK payloads feature is enabled.

        - `True` enables the use of custom ACK payloads in the ACK packet when responding to
          receiving transmissions. As `dynamic_payloads` and `auto_ack` attributes are required for
          this feature to work, they are automatically enabled as needed.
        - `False` disables the use of custom ACK payloads. Disabling this feature does not disable
          the `auto_ack` and `dynamic_payloads` attributes (they work just fine without this
          feature).
        """
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
        """This allows the MCU to specify a payload to be allocated into the TX FIFO buffer for use
        on a specific data pipe. This payload will then be appended to the automatic acknowledgment
        (ACK) packet that is sent when fresh data is received on the specified pipe. See
        `read_ack()` on how to fetch a received custom ACK payloads.

        :param bytearray buf: This will be the data attached to an automatic ACK packet on the
            incoming transmission about the specified ``pipe_number`` parameter. This must have a
            length in range [1,32] bytes, otherwise a `ValueError` exception is thrown. Any ACK
            payloads will remain in the TX FIFO buffer until transmitted successfully or
            `flush_tx()` is called.
        :param int pipe_number: This will be the pipe number to use for deciding which
            transmissions get a response with the specified ``buf`` parameter's data. This number
            must be in range [0,5], otherwise a `ValueError` exception is thrown.

        :returns: `True` if payload was successfully loaded onto the TX FIFO buffer. `False` if it
            wasn't because TX FIFO buffer is full.

        .. note:: this function takes advantage of a special feature on the nRF24L01 and needs to
            be called for every time a customized ACK payload is to be used (not for every
            automatic ACK packet -- this just appends a payload to the ACK packet). The `ack`,
            `auto_ack`, and `dynamic_payloads` attributes are also automatically enabled by this
            function when necessary.

        .. tip:: The ACK payload must be set prior to receiving a transmission. It is also worth
            noting that the nRF24L01 can hold up to 3 ACK payloads pending transmission. Using this
            function does not over-write existing ACK payloads pending; it only adds to the queue
            (TX FIFO buffer) if it can. Use `flush_tx()` to discard unused ACK payloads when done
            listening.
        """
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
        """Allow or disable ``ask_no_ack`` parameter to `send()` & `write()`."""
        self._features = self._reg_read(NRF24L01_REGISTERS.FEATURE)
        return bool(self._features & 1)

    @allow_ask_no_ack.setter
    def allow_ask_no_ack(self, enable):
        self._features = self._reg_read(NRF24L01_REGISTERS.FEATURE) & 6 | bool(enable)
        self._reg_write(NRF24L01_REGISTERS.FEATURE, self._features)

    @property
    def data_rate(self):
        """This `int` attribute specifies the nRF24L01's frequency data rate for OTA (over the air)
        transmissions.

        A valid input value is:

        - ``1`` sets the frequency data rate to 1 Mbps
        - ``2`` sets the frequency data rate to 2 Mbps
        - ``250`` sets the frequency data rate to 250 Kbps

        Any invalid input throws a `ValueError` exception. Default is 1 Mbps.

        .. warning:: 250 Kbps is be buggy on the non-plus models of the nRF24L01 product line. If
            you use 250 Kbps data rate, and some transmissions report failed by the transmitting
            nRF24L01, even though the same packet in question actually reports received by the
            receiving nRF24L01, then try a higher data rate. CAUTION: Higher data rates mean less
            maximum distance between nRF24L01 transceivers (and vise versa).
        """
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
        """This `int` attribute specifies the nRF24L01's frequency (in 2400 + `channel` MHz).

        A valid input value must be in range [0, 125] (that means [2.4, 2.525] GHz). Otherwise a
        `ValueError` exception is thrown. Default is 76.
        """
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
        scheme in terms of byte length.

        A valid input value is in range [0,2]:

        - ``0`` disables CRC
        - ``1`` enables CRC encoding scheme using 1 byte
        - ``2`` enables CRC encoding scheme using 2 bytes

        Any invalid input throws a `ValueError` exception. Default is enabled using 2 bytes.

        .. note:: The nRF24L01 automatically enables CRC if automatic acknowledgment feature is
            enabled (see `auto_ack` attribute).
        """
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
        """This `bool` attribute controls the power state of the nRF24L01. This is exposed for
        asynchronous applications and user preference.

        - `False` basically puts the nRF24L01 to sleep (AKA power down mode) with ultra-low
          current consumption. No transmissions are executed when sleeping, but the nRF24L01 can
          still be accessed through SPI. Upon instantiation, this driver class puts the nRF24L01
          to sleep until the MCU invokes RX/TX transmissions. This driver class doesn't power down
          the nRF24L01 after RX/TX transmissions are complete (avoiding the required power up/down
          130 microseconds wait time), that preference is left to the user.
        - `True` powers up the nRF24L01. This is the first step towards entering RX/TX modes (see
          also `listen` attribute). Powering up is automatically handled by the `listen` attribute
          as well as the `send()` and `write()` functions.

        .. note:: This attribute needs to be `True` if you want to put radio on Standby-II (highest
            current consumption) or Standby-I (moderate current consumption) modes. TX
            transmissions are only executed during Standby-II by calling `send()` or `write()`. RX
            transmissions are received during Standby-II by setting `listen` attribute to `True`
            (see `Chapter 6.1.2-7 of the nRF24L01+ Specifications Sheet <https://www.sparkfun.com/
            datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0
            .pdf#G1132980>`_). After using `send()` or setting `listen` to `False`, the nRF24L01
            is left in Standby-I mode (see also notes on the `write()` function).
        """
        self._config = self._reg_read(NRF24L01_REGISTERS.CONFIG)
        return bool(self._config & 2)

    @power.setter
    def power(self, is_on):
        self._config = self._reg_read(NRF24L01_REGISTERS.CONFIG) & 0x7D | bool(is_on) << 1
        self._reg_write(NRF24L01_REGISTERS.CONFIG, self._config)
        time.sleep(0.00016)

    @property
    def pa_level(self):
        """This `int` attribute specifies the nRF24L01's power amplifier level (in dBm). Higher
        levels mean the transmission will cover a longer distance. Use this attribute to tweak the
        nRF24L01 current consumption on projects that don't span large areas.

        A valid input value is:

        - ``-18`` sets the nRF24L01's power amplifier to -18 dBm (lowest)
        - ``-12`` sets the nRF24L01's power amplifier to -12 dBm
        - ``-6`` sets the nRF24L01's power amplifier to -6 dBm
        - ``0`` sets the nRF24L01's power amplifier to 0 dBm (highest)

        Any invalid input throws a `ValueError` exception. Default is 0 dBm.
        """
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
        top level (first out) of the TX FIFO buffer.

        .. note:: The nRF24L01 normally removes a payload from the TX FIFO buffer after successful
            transmission, but not when this function is called. The payload (successfully
            transmitted or not) will remain in the TX FIFO buffer until `flush_tx()` is called to
            remove them. Alternatively, using this function also allows the failed payload to be
            over-written by using `send()` or `write()` to load a new payload.
        """
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
        """This non-blocking function (when used as alternative to `send()`) is meant for
        asynchronous applications and can only handle one payload at a time as it is a helper
        function to `send()`.

        :param bytearray buf: The payload to transmit. This bytearray must have a length greater
            than 0 and less than 32 bytes, otherwise a `ValueError` exception is thrown.

            - If the `dynamic_payloads` attribute is disabled and this bytearray's length is less
              than the `payload_length` attribute, then this bytearray is padded with zeros until
              its length is equal to the `payload_length` attribute.
            - If the `dynamic_payloads` attribute is disabled and this bytearray's length is
              greater than `payload_length` attribute, then this bytearray's length is truncated to
              equal the `payload_length` attribute.
        :param bool ask_no_ack: Pass this parameter as `True` to tell the nRF24L01 not to wait for
            an acknowledgment from the receiving nRF24L01. This parameter directly controls a
            ``NO_ACK`` flag in the transmission's Packet Control Field (9 bits of information about
            the payload). Therefore, it takes advantage of an nRF24L01 feature specific to
            individual payloads, and its value is not saved anywhere. You do not need to specify
            this for every payload if the `auto_ack` attribute is disabled, however this parameter
            should work despite the `auto_ack` attribute's setting.

            .. note:: Each transmission is in the form of a packet. This packet contains sections
                of data around and including the payload. `See Chapter 7.3 in the nRF24L01
                Specifications Sheet <https://www.sparkfun.com/datasheets/Components/SMD/
                nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf#G1136318>`_ for more
                details.

        This function isn't completely non-blocking as we still need to wait just under 5 ms for
        the CSN pin to settle (allowing a clean SPI transaction).

        .. note:: The nRF24L01 doesn't initiate sending until a mandatory minimum 10 microseconds pulse on
            the CE pin is acheived. That pulse is initiated before this function exits. However, we
            have left that 10 microseconds wait time to be managed by the MCU in cases of asychronous
            application, or it is managed by using `send()` instead of this function. If the CE pin
            remains HIGH for longer than 10 microseconds, then the nRF24L01 will continue to transmit all
            payloads found in the TX FIFO buffer.

        .. warning:: A note paraphrased from the `nRF24L01+ Specifications Sheet <https://www.
            sparkfun.com/datasheets/Components/SMD/
            nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf#G1121422>`_:

            It is important to NEVER to keep the nRF24L01+ in TX mode for more than 4 ms at a time.
            If the [`auto_ack` and `dynamic_payloads`] features are enabled, nRF24L01+ is never in
            TX mode longer than 4 ms.

        .. tip:: Use this function at your own risk. Because of the underlying `"Enhanced
            ShockBurst Protocol" <https://www.sparkfun.com/datasheets/Components/SMD/
            nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf#G1132607>`_, disobeying the 4
            ms rule is easily avoided if you enable the `dynamic_payloads` and `auto_ack`
            attributes. Alternatively, you MUST use interrupt flags or IRQ pin with user defined
            timer(s) to AVOID breaking the 4 ms rule. If the `nRF24L01+ Specifications Sheet
            explicitly states this <https://www.sparkfun.com/datasheets/Components/SMD/
            nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf#G1121422>`_, we have to assume
            radio damage or misbehavior as a result of disobeying the 4 ms rule. See also `table 18
            in the nRF24L01 specification sheet <https://www.sparkfun.com/datasheets/Components/SMD/
            nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf#G1123001>`_ for calculating
            necessary transmission time (these calculations are used in the `send()` function).
        """
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
        """A helper function to flush the nRF24L01's internal RX FIFO buffer. (write-only)

        .. note:: The nRF24L01 RX FIFO is 3 level stack that holds payload data. This means that
            there can be up to 3 received payloads (each of a maximum length equal to 32 bytes)
            waiting to be read (and popped from the stack) by `recv()` or `read_ack()`. This
            function clears all 3 levels.
        """
        self._reg_write(0xE2)

    def flush_tx(self):
        """A helper function to flush the nRF24L01's internal TX FIFO buffer. (write-only)

        .. note:: The nRF24L01 TX FIFO is 3 level stack that holds payload data. This means that
            there can be up to 3 payloads (each of a maximum length equal to 32 bytes) waiting to
            be transmit by `send()`, `resend()` or `write()`. This function clears all 3 levels. It
            is worth noting that the payload data is only popped from the TX FIFO stack upon
            successful transmission (see also `resend()` as the handling of failed transmissions
            can be altered).
        """
        self._reg_write(0xE1)

    def fifo(self, about_tx=False, check_empty=None):
        """This provides some precision determining the status of the TX/RX FIFO buffers.
        (read-only)

        :param bool about_tx:
            * `True` means information returned is about the TX FIFO buffer.
            * `False` means information returned is about the RX FIFO buffer. This parameter
              defaults to `False` when not specified.
        :param bool check_empty:
            * `True` tests if the specified FIFO buffer is empty.
            * `False` tests if the specified FIFO buffer is full.
            * `None` (when not specified) returns a 2 bit number representing both empty (bit 1) &
              full (bit 0) tests related to the FIFO buffer specified using the ``tx`` parameter.
        :returns:
            * A `bool` answer to the question:
              "Is the [TX/RX]:[`True`/`False`] FIFO buffer [empty/full]:[`True`/`False`]?
            * If the ``check_empty`` parameter is not specified: an `int` in range [0,2] for which:

              - ``1`` means the specified FIFO buffer is full
              - ``2`` means the specified FIFO buffer is empty
              - ``0`` means the specified FIFO buffer is neither full nor empty
        """
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
            self.ce_pin = 1
            time.sleep(0.001)
            self.ce_pin = 0
            self.clear_status_flags()
            self._reg_write(0x17, 0x40)
        self.ce_pin = 1

    def stop_carrier_wave(self):
        """Stops a continuous carrier wave test."""
        self.ce_pin = 0
        self.power = 0
        self._rf_setup &= ~0x90
        self._reg_write(NRF24L01_REGISTERS.RF_SETUP, self._rf_setup)
