#!/usr/bin/env python3
"""
CrowPi 2 / Raspberry Pi 4 - turn OFF the 8x8 LED matrix (MAX7219).

The CrowPi 2 LED matrix is a MAX7219 driven over SPI0 with chip-enable CE1,
which is physical header pin 26 (= BCM GPIO7 = /dev/spidev0.1). In luma that
is port=0, device=1.

The MAX7219 latches its register state, so once we blank it the matrix stays
dark after this script exits.

Setup:
    Enable SPI:   sudo raspi-config -> Interface Options -> SPI -> Yes -> reboot
    Install lib:  pip3 install luma.led_matrix
"""

from luma.core.interface.serial import spi, noop
from luma.led_matrix.device import max7219


def main():
    # port=0 -> SPI0 ; device=1 -> CE1 -> physical pin 26 (BCM GPIO7)
    serial = spi(port=0, device=1, gpio=noop())
    device = max7219(serial, cascaded=1, block_orientation=0, rotate=0)

    device.clear()       # write 0 to every pixel -> all LEDs off
    device.contrast(0)   # intensity to minimum as well

    print("LED matrix cleared (all LEDs off).")


if __name__ == "__main__":
    main()
