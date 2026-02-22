#!/usr/bin/env python3
'''
setup board.h for ESP32
AP_FLAKE8_CLEAN
'''

import os
import sys
import re
import shlex

# Module level import not at top of file fix: move sys.path adjustment
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef  # noqa: E402

class ESP32HWDef(hwdef.HWDef):
    # Hardware capability database for ESP32 variants
    CHIP_DATA = {
        'ESP32': {
            'reserved': {6, 7, 8, 9, 10, 11},
            'adc': {0, 2, 4, 12, 13, 14, 15, 25, 26, 27, 32, 33, 34, 35, 36, 37, 38, 39},
            'input_only': {34, 35, 36, 37, 38, 39},
            'strapping': {0, 2, 12, 15},
            'features': ['HAL_ESP32_HAS_MCPWM', 'HAL_ESP32_HAS_DAC']
        },
        'ESP32S2': {
            'reserved': {26, 27, 28, 29, 30, 31, 32},
            'adc': set(range(1, 21)),
            'input_only': {46},
            'strapping': {0, 45, 46},
            'features': ['HAL_ESP32_HAS_USB_OTG']
        },
        'ESP32S3': {
            'reserved': {26, 27, 28, 29, 30, 31, 32},
            'adc': set(range(1, 21)),
            'input_only': set(),
            'strapping': {0, 3, 45, 46},
            'features': ['HAL_ESP32_HAS_MCPWM', 'HAL_ESP32_HAS_USB_SERIAL_JTAG',
                         'HAL_ESP32_LARGE_BUFFERS']
        },
        'ESP32C3': {
            'reserved': {12, 13, 14, 15, 16, 17},
            'adc': {0, 1, 2, 3, 4, 5},
            'input_only': set(),
            'strapping': {2, 8, 9},
            'features': ['HAL_ESP32_HAS_USB_SERIAL_JTAG']
        },
        'ESP32C6': {
            'reserved': set(),
            'adc': set(),
            'input_only': set(),
            'strapping': set(),
            'features': ['HAL_ESP32_HAS_USB_SERIAL_JTAG', 'HAL_ESP32_HAS_MCPWM']
        },
        'ESP32P4': {
            'reserved': set(),
            'adc': set(),
            'input_only': set(),
            'strapping': set(),
            'features': ['HAL_ESP32_HAS_MCPWM', 'HAL_ESP32_HAS_MIPI',
                         'HAL_ESP32_HAS_USB_SERIAL_JTAG', 'HAL_ESP32_LARGE_BUFFERS']
        }
    }

    def __init__(self, outdir, hwdef, quiet=False, **kwargs):
        super(ESP32HWDef, self).__init__(outdir=outdir, hwdef=hwdef, quiet=quiet, **kwargs)
        self.generate_defines = True
        self.board = os.path.basename(os.path.dirname(hwdef[0]))
        self.mcu = 'esp32'
        self.psram_config = {}  # Store PSRAM configuration
        self.pin_config = {}    # Store pin assignments
        self.serial_pins = {}   # Store SERIAL pin assignments
        self.can_pins = {}      # Store CAN pin assignments
        self.led_pins = {}      # Store LED pin assignments
        self.rcout_pins = []    # Store RCOUT pin assignments
        self.spi_buses = []     # Store SPI bus configurations
        self.spi_devices = []   # Store SPI device configurations
        self.i2c_buses = []     # Store I2C bus configurations
        self.uart_devices = []  # Store UART device configurations
        self.adc_pins = []      # Store ADC pin configurations
        self.rmt_rx_pin = None  # Store RMT RX pin
        self.rcin_pin = None    # Store RCIN pin
        self.pin_assignments = {}  # Track all pin assignments for conflict detection
        self.reserved_pins = set()  # Track reserved/system pins
        self.input_only_pins = set()  # Track input-only pins
        self.strapping_pins = set()  # Track strapping pins
        self.partition_table_filename = None  # Store custom partition table filename
        self.flash_size_mb = None  # Store custom flash size in MB
        self.psram_size = None  # Store PSRAM size
        # Note: init_hardware_constraints() called after MCU is determined

    def init_hardware_constraints(self):
        '''Initialize chip-specific pin constraints from CHIP_DATA'''
        chip = self.CHIP_DATA.get(self.mcu.upper())
        if chip:
            self.reserved_pins.update(chip.get('reserved', set()))
            self.adc_capable_pins = chip.get('adc', set())
            self.input_only_pins = chip.get('input_only', set())
            self.strapping_pins = chip.get('strapping', set())
            self.progress(f"{self.mcu.upper()} constraints loaded")
        else:
            self.progress(f"Unknown MCU '{self.mcu}' - using minimal constraints")
            self.adc_capable_pins = set()

    def validate_pin_assignment(self, pin_num, function, pin_name):
        '''Validate a pin assignment using chip-specific capabilities'''
        try:
            pin_num = int(pin_num)
        except (ValueError, TypeError):
            self.error(f"Invalid pin number '{pin_num}' for {function} {pin_name}")
            return False

        # Check if pin is reserved
        if pin_num in self.reserved_pins:
            self.error(f"Pin {pin_num} is reserved for system use "
                       f"(Flash/PSRAM/USB) - cannot assign to "
                       f"{function} {pin_name}")
            return False

        # Check if pin is input-only but used for output
        if pin_num in self.input_only_pins and function in \
           ['CAN_TX', 'UART_TX', 'SCK', 'MOSI', 'SDA_OUT',
            'LED_DATA', 'LED_CLOCK']:
            self.error(f"Pin {pin_num} is input-only - cannot use for "
                       f"output function {function} {pin_name}")
            return False

        # Check peripheral-specific capabilities
        if (function in ['ADC'] and hasattr(self, 'adc_capable_pins')
                and pin_num not in self.adc_capable_pins):
            self.error(f"Pin {pin_num} does not support ADC - "
                       f"cannot assign to {function} {pin_name}")
            return False

        # Warn about strapping pins (don't block, just warn)
        if hasattr(self, 'strapping_pins') and pin_num in self.strapping_pins:
            self.progress(f"WARNING: GPIO{pin_num} is a strapping pin - "
                          f"may affect boot behavior ({function} {pin_name})")

        # Check for pin conflicts
        if pin_num in self.pin_assignments:
            existing = self.pin_assignments[pin_num]
            self.error(f"Pin conflict: GPIO{pin_num} assigned to both "
                       f"'{existing}' and '{function} {pin_name}'")
            return False

        # Record pin assignment
        self.pin_assignments[pin_num] = f"{function} {pin_name}"
        return True

    def _is_pin_assignment_line(self, line):
        '''Check if line is a pin assignment using space-separated syntax'''
        line = line.strip()
        if not line or line.startswith('#') or line.startswith('define'):
            return False

        parts = line.split()
        if len(parts) >= 2:
            first_part = parts[0].upper()

            # Explicitly exclude non-pin assignments
            if ('_BAUD' in first_part or
                '_PROTOCOL' in first_part or
                '_SPEED' in first_part or
                    first_part.endswith('_ENABLED')):
                return False

            # Exclude ESP32-specific directives (handled by base class)
            if first_part.startswith('ESP32_'):
                return False

            # Check if it looks like a pin assignment
            if ('_PIN' in first_part or
                first_part.startswith('CAN_') or
                first_part.startswith('SERIAL') and '_PIN' in first_part or
                first_part in ['KEY_BOOT'] or
                    first_part.startswith('RMT_')):
                # Exclude BAUD settings which are not pin assignments
                if '_BAUD' not in first_part:
                    return True
        return False

    def error(self, msg):
        '''Print error message and raise exception'''
        print(f"ERROR: {msg}")
        raise ValueError(msg)

    def validate_configuration(self):
        '''Perform final validation of the complete pin configuration'''
        self.progress("Validating pin configuration...")

        # Check for required interfaces
        warnings = []

        # Check CAN configuration
        if self.can_pins:
            for can_id in ['1']:  # Check CAN1
                tx_key = f'tx_{can_id}'
                rx_key = f'rx_{can_id}'
                if tx_key in self.can_pins and rx_key not in self.can_pins:
                    warnings.append(f"CAN{can_id} has TX pin but no RX pin")
                elif rx_key in self.can_pins and tx_key not in self.can_pins:
                    warnings.append(f"CAN{can_id} has RX pin but no TX pin")

        # Check SERIAL pairs
        if self.serial_pins:
            for serial_id in ['1', '2', '3']:
                tx_key = f'tx_{serial_id}'
                rx_key = f'rx_{serial_id}'
                if tx_key in self.serial_pins and rx_key not in self.serial_pins:
                    warnings.append(f"SERIAL{serial_id} has TX but no RX pin")
                elif rx_key in self.serial_pins and tx_key not in self.serial_pins:
                    warnings.append(f"SERIAL{serial_id} has RX but no TX pin")

        # Report summary
        total_pins = len(self.pin_assignments)
        reserved_used = len([p for p in self.pin_assignments.keys()
                            if p in self.reserved_pins])

        self.progress(f"Pin validation complete: {total_pins} pins assigned, "
                      f"{reserved_used} reserved pins used")

        for warning in warnings:
            self.progress(f"WARNING: {warning}")

        if warnings:
            self.progress(f"Found {len(warnings)} configuration warnings")

    def run(self):
        self.process_hwdefs()
        # Validate pin configuration after processing
        self.validate_configuration()
        if self.generate_defines:
            self.generate_hwdef_h()
        # Generate ESP-IDF config if PSRAM is configured
        self.write_esp_idf_config()
        # Copy board-specific partition table if specified
        self.copy_partition_table()
        return 0

    def process_line(self, line, depth=0):
        super(ESP32HWDef, self).process_line(line, depth)
        if line.startswith("MCU"):
            self.mcu = line.split()[1]
            self.env_vars['MCU'] = self.mcu
            # Initialize hardware constraints now that we know the MCU
            self.init_hardware_constraints()
        elif line.startswith("APJ_BOARD_ID"):
            parts = line.split()
            if len(parts) >= 2:
                # Store the board ID as an integer define
                self.intdefines['APJ_BOARD_ID'] = int(parts[1])
                self.progress("Found APJ_BOARD_ID: %s" % parts[1])
        elif line.startswith("PSRAM_SIZE"):
            parts = line.split()
            if len(parts) >= 2:
                self.psram_config['size'] = parts[1]
                self.psram_size = parts[1]
                self.progress("Found PSRAM_SIZE: %s" % parts[1])
        elif line.startswith("PSRAM_MODE"):
            parts = line.split()
            if len(parts) >= 2:
                self.psram_config['mode'] = parts[1]
                self.progress("Found PSRAM_MODE: %s" % parts[1])
        elif line.startswith("PSRAM_MALLOC_THRESHOLD"):
            parts = line.split()
            if len(parts) >= 2:
                self.psram_config['malloc_threshold'] = parts[1]
                self.progress("Found PSRAM_MALLOC_THRESHOLD: %s" % parts[1])
        elif line.startswith("PSRAM_RESERVE_INTERNAL"):
            parts = line.split()
            if len(parts) >= 2:
                self.psram_config['reserve_internal'] = parts[1]
                self.progress("Found PSRAM_RESERVE_INTERNAL: %s" % parts[1])
        elif line.startswith("PARTITION_TABLE_CUSTOM_FILENAME"):
            parts = line.split()
            if len(parts) >= 2:
                self.partition_table_filename = parts[1]
                self.progress("Found PARTITION_TABLE_CUSTOM_FILENAME: "
                              "%s" % parts[1])
        elif line.startswith("FLASH_SIZE_MB"):
            parts = line.split()
            if len(parts) >= 2:
                self.flash_size_mb = int(parts[1])
                self.progress("Found FLASH_SIZE_MB: %sMB" % parts[1])
        # Parse pin assignments
        elif (('=' in line and not line.startswith('define'))
              or self._is_pin_assignment_line(line)):
            if '=' in line:
                parts = line.split('=', 1)
                pin_name = parts[0].strip()
                pin_value = parts[1].strip()
            else:
                # Handle space-separated syntax like "SERIAL1_TX_PIN 1"
                parts = line.strip().split()
                if len(parts) >= 2:
                    pin_name = parts[0]
                    pin_value = parts[1]
                else:
                    self.progress(f"Skipping invalid pin assignment: "
                                  f"{line.strip()}")
                    return

            # CAN pins
            if pin_name.startswith('CAN_TX_'):
                can_num = pin_name.split('_')[2]
                if self.validate_pin_assignment(pin_value, 'CAN_TX',
                                               f'CAN{can_num}'):
                    self.can_pins[f'tx_{can_num}'] = pin_value
            elif pin_name.startswith('CAN_RX_'):
                can_num = pin_name.split('_')[2]
                if self.validate_pin_assignment(pin_value, 'CAN_RX',
                                               f'CAN{can_num}'):
                    self.can_pins[f'rx_{can_num}'] = pin_value

            # SERIAL pins
            elif pin_name.startswith('SERIAL') and '_TX_PIN' in pin_name:
                serial_num = pin_name.split('SERIAL')[1].split('_')[0]
                if self.validate_pin_assignment(pin_value, 'UART_TX',
                                               f'SERIAL{serial_num}'):
                    self.serial_pins[f'tx_{serial_num}'] = pin_value
            elif pin_name.startswith('SERIAL') and '_RX_PIN' in pin_name:
                serial_num = pin_name.split('SERIAL')[1].split('_')[0]
                if self.validate_pin_assignment(pin_value, 'UART_RX',
                                               f'SERIAL{serial_num}'):
                    self.serial_pins[f'rx_{serial_num}'] = pin_value

            # LED pins
            elif pin_name == 'APA102_DATA_PIN':
                if self.validate_pin_assignment(pin_value, 'LED_DATA',
                                               'APA102'):
                    self.led_pins['apa102_data'] = pin_value
            elif pin_name == 'APA102_CLOCK_PIN':
                if self.validate_pin_assignment(pin_value, 'LED_CLOCK',
                                               'APA102'):
                    self.led_pins['apa102_clock'] = pin_value
            elif pin_name == 'APA102_NUM_LEDS':
                try:
                    num_leds = int(pin_value)
                    if 1 <= num_leds <= 16:
                        self.led_pins['apa102_num_leds'] = num_leds
                    else:
                        print(f"Warning: APA102_NUM_LEDS value "
                              f"{num_leds} out of range (1-16)")
                except ValueError:
                    print(f"Warning: Invalid APA102_NUM_LEDS value "
                          f"'{pin_value}'")

            # Boot/system pins
            elif pin_name == 'KEY_BOOT':
                if self.validate_pin_assignment(pin_value, 'BOOT_BUTTON',
                                               'SYSTEM'):
                    pass  # Just validate

            # RMT (Remote Control) pin handling
            elif pin_name.startswith('RMT_RX'):
                if self.validate_pin_assignment(pin_value, 'RMT_RX',
                                               pin_name):
                    self.rmt_rx_pin = pin_value

            # RCOUT pin handling
            elif pin_name.startswith('RCOUT') and pin_name.endswith('_PIN'):
                if self.validate_pin_assignment(pin_value, 'RCOUT',
                                               pin_name):
                    try:
                        channel_str = pin_name.replace('RCOUT', '') \
                            .replace('_PIN', '')
                        channel_num = int(channel_str)
                        while len(self.rcout_pins) < channel_num:
                            self.rcout_pins.append(None)
                        self.rcout_pins[channel_num - 1] = pin_value
                    except ValueError:
                        self.progress(f"Invalid RCOUT channel number "
                                      f"in {pin_name}")

            # I2C pin handling
            elif (pin_name.startswith('I2C') and
                  ('_SDA_PIN' in pin_name or '_SCL_PIN' in pin_name)):
                if self.validate_pin_assignment(pin_value, 'I2C', pin_name):
                    try:
                        bus_str = pin_name.split('_')[0].replace('I2C', '')
                        bus_num = int(bus_str) if bus_str else 0

                        bus_entry = None
                        for bus in self.i2c_buses:
                            if bus['num'] == bus_num:
                                bus_entry = bus
                                break

                        if bus_entry is None:
                            bus_entry = {'num': bus_num,
                                         'sda': None, 'scl': None}
                            self.i2c_buses.append(bus_entry)

                        if '_SDA_PIN' in pin_name:
                            bus_entry['sda'] = pin_value
                        elif '_SCL_PIN' in pin_name:
                            bus_entry['scl'] = pin_value

                    except ValueError:
                        self.progress(f"Invalid I2C bus number "
                                      f"in {pin_name}")

            # SPI pin handling
            elif (pin_name.startswith('SPI') and
                  ('_SCK_PIN' in pin_name or '_MISO_PIN' in pin_name
                   or '_MOSI_PIN' in pin_name)):
                if self.validate_pin_assignment(pin_value, 'SPI', pin_name):
                    try:
                        bus_str = pin_name.split('_')[0].replace('SPI', '')
                        bus_num = int(bus_str) if bus_str else 0

                        bus_entry = None
                        for bus in self.spi_buses:
                            if bus['num'] == bus_num:
                                bus_entry = bus
                                break

                        if bus_entry is None:
                            bus_entry = {'num': bus_num, 'sck': None,
                                         'miso': None, 'mosi': None}
                            self.spi_buses.append(bus_entry)

                        if '_SCK_PIN' in pin_name:
                            bus_entry['sck'] = pin_value
                            self.progress(f"Added SPI{bus_num} SCK: "
                                          f"GPIO {pin_value}")
                        elif '_MISO_PIN' in pin_name:
                            bus_entry['miso'] = pin_value
                            self.progress(f"Added SPI{bus_num} MISO: "
                                          f"GPIO {pin_value}")
                        elif '_MOSI_PIN' in pin_name:
                            bus_entry['mosi'] = pin_value
                            self.progress(f"Added SPI{bus_num} MOSI: "
                                          f"GPIO {pin_value}")

                    except ValueError:
                        self.progress(f"Invalid SPI bus number "
                                      f"in {pin_name}")

            # SPI device chip select pins
            elif pin_name.endswith('_CS_PIN'):
                if self.validate_pin_assignment(pin_value, 'SPI_CS',
                                               pin_name):
                    device_name = pin_name.replace('_CS_PIN', '').lower()
                    spi_device = {'name': device_name, 'cs': pin_value}
                    self.spi_devices.append(spi_device)
                    self.progress(f"Added SPI device {device_name}: "
                                  f"CS GPIO {pin_value}")

            # ADC pin handling
            elif pin_name.startswith('ADC') and pin_name.endswith('_PIN'):
                if self.validate_pin_assignment(pin_value, 'ADC', pin_name):
                    try:
                        channel_str = pin_name.replace('ADC', '') \
                            .replace('_PIN', '')
                        channel_num = int(channel_str)
                        adc_entry = {'channel': channel_num,
                                     'pin': pin_value}
                        self.adc_pins.append(adc_entry)
                        self.progress(f"Added ADC channel {channel_num}: "
                                      f"GPIO {pin_value}")
                    except ValueError:
                        self.progress(f"Invalid ADC channel number "
                                      f"in {pin_name}")

            # RCInput pin handling
            elif pin_name == 'RCIN_PIN':
                if self.validate_pin_assignment(pin_value, 'RCIN', pin_name):
                    self.rcin_pin = pin_value
                    self.progress(f"Added RCIN: GPIO {pin_value}")

            # Generic GPIO validation for any other pins
            else:
                if self.validate_pin_assignment(pin_value, 'GPIO', pin_name):
                    pass  # Validated successfully

            # Store all pin assignments
            self.pin_config[pin_name] = pin_value

    def generate_hwdef_h(self):
        '''generate a hwdef.h file'''
        self.progress("Generating hwdef.h")
        fname = os.path.join(self.outdir, "hwdef.h")
        f = open(fname, "w")
        f.write("/* Auto-generated hwdef.h for %s */\n\n" % self.board)
        f.write("#pragma once\n\n")

        # Include board-specific HAL configuration
        f.write("#include <AP_HAL/board/esp32.h>\n\n")

        # Write numerical defines (hwdef.h should override defaults)
        for d in sorted(self.intdefines.keys()):
            f.write("#undef %s\n" % d)
            f.write("#define %s %s\n" % (d, self.intdefines[d]))

        # Write string defines (hwdef.h should override defaults)
        if hasattr(self, 'strdefines'):
            for name in sorted(self.strdefines.keys()):
                undef_name = name.split('(')[0]
                f.write("#undef %s\n" % undef_name)
                f.write("#define %s %s\n" % (name, self.strdefines[name]))

        # Automatically handle ADC configuration if not explicitly defined
        if (not self.adc_pins
                and 'HAL_USE_ADC' not in self.intdefines
                and 'HAL_DISABLE_ADC_DRIVER' not in self.intdefines):
            f.write("#undef HAL_DISABLE_ADC_DRIVER\n")
            f.write("#define HAL_DISABLE_ADC_DRIVER 1\n")
            f.write("#undef HAL_USE_ADC\n")
            f.write("#define HAL_USE_ADC 0\n")

        # Auto-disable GYROFFT when DSP is disabled
        if ('HAL_WITH_DSP' in self.intdefines
                and self.intdefines['HAL_WITH_DSP'] == 0):
            if 'HAL_GYROFFT_ENABLED' not in self.intdefines:
                self.progress("HAL_WITH_DSP is 0, automatically "
                              "disabling HAL_GYROFFT_ENABLED")
                f.write("#undef HAL_GYROFFT_ENABLED\n")
                f.write("#define HAL_GYROFFT_ENABLED 0\n")

        # Ensure GCS support is enabled by default for ESP32
        if 'HAL_GCS_ENABLED' not in self.intdefines:
            f.write("#undef HAL_GCS_ENABLED\n")
            f.write("#define HAL_GCS_ENABLED 1\n")

        # Add standard TRUE/FALSE constants for compatibility
        f.write("#undef TRUE\n")
        f.write("#define TRUE 1\n")
        f.write("#undef FALSE\n")
        f.write("#define FALSE 0\n")
        f.write("\n")

        # Emit chip feature defines from CHIP_DATA
        chip = self.CHIP_DATA.get(self.mcu.upper())
        if chip:
            for feat in chip.get('features', []):
                if feat not in self.intdefines:
                    f.write("#ifndef %s\n#define %s 1\n#endif\n"
                            % (feat, feat))

        # Write IMU probe list from IMU directives
        self.write_IMU_config(f)

        # Write barometer probe list from BARO directives
        self.write_BARO_config(f)

        # Write ESP32-specific board definitions
        self.write_esp32_board_config(f)
        f.close()

    def write_esp32_board_config(self, f):
        '''Write ESP32-specific board configuration defines'''
        f.write("\n/* ESP32 Board Configuration */\n")

        # Only generate UART devices if SERIAL pins are defined
        if self.serial_pins:
            f.write("\n/* UART Configuration - only defined pins */\n")
            f.write("#define HAL_ESP32_UART_DEVICES \\\n")

            uart_entries = []
            for key in sorted(self.serial_pins.keys()):
                if key.startswith('tx_'):
                    serial_num = key.split('_')[1]
                    tx_key = f'tx_{serial_num}'
                    rx_key = f'rx_{serial_num}'
                    if (tx_key in self.serial_pins
                            and rx_key in self.serial_pins):
                        tx_pin = self.serial_pins[tx_key]
                        rx_pin = self.serial_pins[rx_key]
                        uart_entries.append(
                            f'    {{ .rx = GPIO_NUM_{rx_pin}, '
                            f'.tx = GPIO_NUM_{tx_pin} }}'
                        )

            if uart_entries:
                f.write(',\\\n'.join(uart_entries))
                f.write('\n\n')
            else:
                f.write('    /* No UART devices defined */\n\n')
        else:
            f.write("\n/* No UART Configuration */\n")
            f.write("#define HAL_ESP32_UART_DEVICES {}\n\n")

        # Only generate CAN pins if defined
        if self.can_pins:
            f.write("/* CAN Configuration */\n")
            if 'tx_1' in self.can_pins and 'rx_1' in self.can_pins:
                f.write(f"#define HAL_CAN1_TX_PIN "
                        f"{self.can_pins['tx_1']}\n")
                f.write(f"#define HAL_CAN1_RX_PIN "
                        f"{self.can_pins['rx_1']}\n")
            f.write("\n")

        # Only generate LED pins if defined
        if self.led_pins:
            f.write("/* LED Configuration */\n")
            if 'apa102_data' in self.led_pins:
                f.write(f"#define HAL_APA102_DATA_PIN "
                        f"{self.led_pins['apa102_data']}\n")
            if 'apa102_clock' in self.led_pins:
                f.write(f"#define HAL_APA102_CLOCK_PIN "
                        f"{self.led_pins['apa102_clock']}\n")
            if 'apa102_num_leds' in self.led_pins:
                f.write(f"#define HAL_APA102_NUM_LEDS "
                        f"{self.led_pins['apa102_num_leds']}\n")
            f.write("\n")

        # Only generate RCOUT if pins are defined
        if self.rcout_pins and any(p is not None for p in self.rcout_pins):
            f.write("/* RCOUT Configuration */\n")
            valid_pins = [pin for pin in self.rcout_pins if pin is not None]
            f.write(f"#define HAL_ESP32_RCOUT_MAX {len(valid_pins)}\n")
            pin_list = ','.join([f'GPIO_NUM_{pin}' for pin in valid_pins])
            f.write(f"#define HAL_ESP32_RCOUT {{{pin_list}}}\n\n")
        else:
            f.write("/* No RCOUT pins defined */\n")
            f.write("#define HAL_ESP32_RCOUT_MAX 0\n")
            f.write("#define HAL_ESP32_RCOUT {}\n\n")

        # Generate SPI bus and device configuration
        if self.spi_buses:
            f.write("/* SPI Bus Configuration */\n")
            f.write("#define HAL_ESP32_SPI_BUSES \\\n")

            spi_bus_entries = []
            for bus in sorted(self.spi_buses, key=lambda x: x['num']):
                if (bus['sck'] is not None and bus['miso'] is not None
                        and bus['mosi'] is not None):
                    mcu_lower = self.mcu.lower()

                    if bus['num'] == 1:
                        host = "SPI2_HOST"
                    elif bus['num'] == 2:
                        host = "SPI3_HOST"
                    elif bus['num'] == 3:
                        if mcu_lower in ['esp32c3', 'esp32c6', 'esp32h2']:
                            host = "SPI1_HOST"
                        elif (mcu_lower in ['esp32s2', 'esp32s3']
                              and not self.psram_config):
                            host = "SPI1_HOST"
                        else:
                            self.error(
                                f"SPI3 (SPI1_HOST) not available on "
                                f"{self.mcu} with PSRAM/OPI flash")
                            continue
                    else:
                        self.error(
                            f"ESP32 only supports SPI1-3, "
                            f"not SPI{bus['num']}")
                        continue
                    entry = (f"    {{.host={host}, .dma_ch=1, "
                             f".mosi=GPIO_NUM_{bus['mosi']}, "
                             f".miso=GPIO_NUM_{bus['miso']}, "
                             f".sclk=GPIO_NUM_{bus['sck']}}}")
                    spi_bus_entries.append(entry)

            if spi_bus_entries:
                f.write(',\\\n'.join(spi_bus_entries))
                f.write('\n\n')
            else:
                f.write("    /* No valid SPI buses defined */\n\n")
        else:
            f.write("/* SPI Configuration */\n")
            f.write("#define HAL_ESP32_SPI_BUSES {}\n\n")

        # Generate SPI device configuration
        if self.spi_devices:
            f.write("/* SPI Device Configuration */\n")
            f.write("#define HAL_ESP32_SPI_DEVICES \\\n")

            spi_device_entries = []
            for i, device in enumerate(self.spi_devices):
                if 'mpu9250' in device['name']:
                    mode, lspeed, hspeed = 0, "2*MHZ", "8*MHZ"
                elif 'bmp280' in device['name']:
                    mode, lspeed, hspeed = 3, "1*MHZ", "1*MHZ"
                else:
                    mode, lspeed, hspeed = 0, "1*MHZ", "10*MHZ"

                entry = (f'    {{.name="{device["name"]}", .bus=0, '
                         f'.device={i}, .cs=GPIO_NUM_{device["cs"]}, '
                         f'.mode={mode}, .lspeed={lspeed}, '
                         f'.hspeed={hspeed}}}')
                spi_device_entries.append(entry)

            f.write(',\\\n'.join(spi_device_entries))
            f.write('\n\n')
        else:
            f.write("/* SPI Device Configuration */\n")
            f.write("#define HAL_ESP32_SPI_DEVICES {}\n\n")

        # Generate I2C bus configuration
        if self.i2c_buses:
            f.write("/* I2C Configuration - board-specific buses */\n")
            f.write("#define HAL_ESP32_I2C_BUSES \\\n")

            i2c_entries = []
            for bus in sorted(self.i2c_buses, key=lambda x: x['num']):
                if bus['sda'] is not None and bus['scl'] is not None:
                    port_num = bus['num'] if bus['num'] <= 1 else 0
                    entry = (f"    {{.port=I2C_NUM_{port_num}, "
                             f".sda=GPIO_NUM_{bus['sda']}, "
                             f".scl=GPIO_NUM_{bus['scl']}, "
                             f".speed=400*KHZ, .internal=true}}")
                    i2c_entries.append(entry)

            if i2c_entries:
                f.write(',\\\n'.join(i2c_entries))
                f.write('\n\n')
            else:
                f.write("    /* No valid I2C buses defined */\n\n")
        else:
            # MCU-appropriate default I2C pins
            if self.mcu.upper() in ('ESP32S3', 'ESP32S2', 'ESP32P4'):
                default_sda, default_scl = 41, 42
            elif self.mcu.upper() == 'ESP32':
                default_sda, default_scl = 21, 22
            else:
                # C3, C6 etc
                default_sda, default_scl = 5, 6
            f.write("/* I2C Configuration - default internal bus */\n")
            f.write(f"#define HAL_ESP32_I2C_BUSES "
                    f"{{.port=I2C_NUM_0, .sda=GPIO_NUM_{default_sda}, "
                    f".scl=GPIO_NUM_{default_scl}, .speed=400*KHZ, "
                    f".internal=true, .soft=true}}\n\n")

        # Generate ADC pin configuration
        if self.adc_pins:
            f.write("/* ADC Configuration - board-specific pins */\n")
            f.write("#define HAL_ESP32_ADC_PINS \\\n")

            adc_entries = []
            for adc in sorted(self.adc_pins, key=lambda x: x['channel']):
                gpio_pin = int(adc['pin'])
                adc_channel_map = {
                    36: "ADC1_GPIO36_CHANNEL",
                    37: "ADC1_GPIO37_CHANNEL",
                    38: "ADC1_GPIO38_CHANNEL",
                    39: "ADC1_GPIO39_CHANNEL",
                    32: "ADC1_GPIO32_CHANNEL",
                    33: "ADC1_GPIO33_CHANNEL",
                    34: "ADC1_GPIO34_CHANNEL",
                    35: "ADC1_GPIO35_CHANNEL",
                }
                if gpio_pin in adc_channel_map:
                    adc_channel = adc_channel_map[gpio_pin]
                    entry = f"    {{{adc_channel}, 11, {adc['pin']}}}"
                    adc_entries.append(entry)

            if adc_entries:
                f.write(',\\\n'.join(adc_entries))
                f.write('\n\n')
            else:
                f.write("    /* No valid ADC pins defined */\n\n")
        else:
            f.write("/* ADC Configuration */\n")
            f.write("#define HAL_ESP32_ADC_PINS {}\n\n")

        # Generate RCIN pin configuration
        if self.rcin_pin is not None:
            f.write("/* RCIN Configuration */\n")
            f.write(f"#define HAL_ESP32_RCIN "
                    f"GPIO_NUM_{self.rcin_pin}\n\n")

        # RMT Configuration
        if (hasattr(self, 'strdefines')
                and 'HAL_ESP32_RMT_RX_PIN_NUMBER' in self.strdefines):
            f.write("/* RMT Configuration */\n")
            f.write(f"#define HAL_ESP32_RMT_RX_PIN_NUMBER "
                    f"{self.strdefines['HAL_ESP32_RMT_RX_PIN_NUMBER']}\n\n")
        elif self.rmt_rx_pin is not None:
            f.write("/* RMT Configuration */\n")
            f.write(f"#define HAL_ESP32_RMT_RX_PIN_NUMBER "
                    f"{self.rmt_rx_pin}\n\n")

    def generate_board_header(self):
        '''Generate complete ESP32 board header file'''
        self.progress("Generating ESP32 board header")
        fname = os.path.join(self.outdir, f"boards/{self.board}.h")

        # Create boards directory if it doesn't exist
        boards_dir = os.path.join(self.outdir, "boards")
        os.makedirs(boards_dir, exist_ok=True)

        f = open(fname, "w")
        f.write("/*\n")
        f.write(" * This file is free software: you can redistribute "
                "it and/or modify it\n")
        f.write(" * under the terms of the GNU General Public License "
                "as published by the\n")
        f.write(" * Free Software Foundation, either version 3 of the "
                "License, or\n")
        f.write(" * (at your option) any later version.\n")
        f.write(" *\n")
        f.write(" * This file is distributed in the hope that it will "
                "be useful, but\n")
        f.write(" * WITHOUT ANY WARRANTY; without even the implied "
                "warranty of\n")
        f.write(" * MERCHANTABILITY or FITNESS FOR A PARTICULAR "
                "PURPOSE.\n")
        f.write(" * See the GNU General Public License for more "
                "details.\n")
        f.write(" *\n")
        f.write(" * You should have received a copy of the GNU General "
                "Public License along\n")
        f.write(" * with this program.  If not, see "
                "<http://www.gnu.org/licenses/>.\n")
        f.write(" */\n")
        f.write(f"/* Auto-generated board header for {self.board} */\n")
        f.write("#pragma once\n\n")

        # Board identification
        f.write("#define CONFIG_HAL_BOARD_SUBTYPE "
                "HAL_BOARD_SUBTYPE_ESP32_LILYGO_TCONNECT\n")
        f.write(f'#define HAL_ESP32_BOARD_NAME "{self.board}"\n\n')

        # Write ESP32-specific board definitions using hwdef data
        self.write_esp32_board_config(f)

        f.close()

    def get_env_vars(self):
        return self.env_vars

    def get_psram_config(self):
        '''Return PSRAM configuration dictionary'''
        return self.psram_config

    def has_psram(self):
        '''Check if board has PSRAM configured'''
        return ('size' in self.psram_config
                and self.psram_config['size'] != '0MB')

    def has_wifi(self):
        '''Check if board has WiFi enabled'''
        hal_with_wifi = self.intdefines.get('HAL_WITH_WIFI', '1')
        return str(hal_with_wifi) != '0'

    def generate_esp_idf_config(self):
        '''Generate ESP-IDF configuration settings based on hwdef'''
        config_lines = []

        # Flash size configuration
        if self.flash_size_mb:
            self.progress(f"Configuring flash size: "
                          f"{self.flash_size_mb}MB")
            for size in [1, 2, 4, 8, 16, 32, 64, 128]:
                if size == self.flash_size_mb:
                    config_lines.append(
                        f"CONFIG_ESPTOOLPY_FLASHSIZE_{size}MB=y")
                else:
                    config_lines.append(
                        f"# CONFIG_ESPTOOLPY_FLASHSIZE_{size}MB "
                        f"is not set")

        # Partition table configuration
        if self.partition_table_filename:
            self.progress(f"Using board-specific partition table: "
                          f"{self.partition_table_filename}")
            config_lines.append("CONFIG_PARTITION_TABLE_CUSTOM=y")
            config_lines.append(
                f'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME='
                f'"{self.partition_table_filename}"')
            config_lines.append("CONFIG_PARTITION_TABLE_OFFSET=0x8000")
        else:
            self.progress("Using default partition table for chip type")

        # WiFi configuration
        if not self.has_wifi():
            self.progress("WiFi disabled - configuring ESP-IDF "
                          "for optimal dual-core usage")
            config_lines.append("# CONFIG_ESP_WIFI_ENABLED is not set")
            config_lines.append(
                "# CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_1 is not set")
            config_lines.append(
                "# CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE "
                "is not set")
            config_lines.append(
                "# CONFIG_ESP_WIFI_GMAC_SUPPORT is not set")
            config_lines.append(
                "CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0=y")
            config_lines.append(
                "# CONFIG_ESP_MAIN_TASK_AFFINITY_CPU1 is not set")
            config_lines.append(
                "# CONFIG_ESP_MAIN_TASK_AFFINITY_NO_AFFINITY "
                "is not set")
        else:
            self.progress("WiFi enabled - configuring ESP-IDF WiFi "
                          "with SoftAP support")
            config_lines.append("CONFIG_ESP_WIFI_ENABLED=y")
            config_lines.append("CONFIG_ESP_WIFI_SOFTAP_SUPPORT=y")

        if self.has_psram():
            self.progress("Generating ESP-IDF PSRAM configuration")
            config_lines.append(
                "# SPIRAM configuration handled automatically "
                "by ESP-IDF 5.5+")
        else:
            self.progress("No PSRAM configured - using ESP-IDF 5.5+ "
                          "defaults")

        # ESP32 Memory Protection and Debugging Configuration
        debug_mode = '1'  # Default to debug mode for safety
        for line in self.hwdef:
            with open(line, 'r') as f:
                for content_line in f:
                    if content_line.strip().startswith(
                            'define ESP32_DEBUG_MODE'):
                        debug_mode = content_line.strip().split()[-1]
                        break

        if debug_mode == '1':
            self.progress("Adding ESP32 memory protection and "
                          "debugging options (DEBUG MODE)")

            config_lines.append(
                "CONFIG_HEAP_POISONING_COMPREHENSIVE=y")
            config_lines.append(
                "# CONFIG_HEAP_POISONING_LIGHT is not set")
            config_lines.append("CONFIG_HEAP_TRACING=y")
            config_lines.append("CONFIG_HEAP_TRACING_STACK_DEPTH=10")
            config_lines.append("CONFIG_HEAP_TASK_TRACKING=y")
            config_lines.append(
                "CONFIG_ESP_SYSTEM_USE_FRAME_POINTER=y")
            config_lines.append(
                "CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS=y")
            config_lines.append(
                "CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY=y")
            config_lines.append("CONFIG_ESP_TASK_WDT_INIT=y")
            config_lines.append("CONFIG_ESP_TASK_WDT_TIMEOUT_S=5")
            config_lines.append(
                "CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL=2")
            config_lines.append(
                "CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y")
            config_lines.append(
                "CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF=y")
            config_lines.append(
                "CONFIG_ESP_COREDUMP_CHECKSUM_CRC32=y")
            config_lines.append(
                "CONFIG_ESP_COREDUMP_MAX_TASKS_NUM=64")
            config_lines.append(
                "CONFIG_ESP_SYSTEM_PANIC_PRINT_HALT=y")
            config_lines.append("CONFIG_ESP_DEBUG_STUBS_ENABLE=y")

        else:
            self.progress("Using ESP32 performance mode (PRODUCTION) "
                          "- debugging disabled")

            config_lines.append(
                "# CONFIG_HEAP_POISONING_COMPREHENSIVE is not set")
            config_lines.append(
                "# CONFIG_HEAP_POISONING_LIGHT is not set")
            config_lines.append(
                "# CONFIG_HEAP_TRACING is not set")
            config_lines.append(
                "# CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY "
                "is not set")
            config_lines.append(
                "CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE=y")
            config_lines.append(
                "CONFIG_ESP_COREDUMP_ENABLE_TO_NONE=y")
            config_lines.append(
                "# CONFIG_ESP_DEBUG_STUBS_ENABLE is not set")
            config_lines.append("CONFIG_ESP_TASK_WDT_INIT=n")
            config_lines.append("CONFIG_ESP_INT_WDT=n")
            config_lines.append(
                "CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL=0")

        # CAN/TWAI configuration
        if self.can_pins:
            config_lines.append(
                "# CAN/TWAI reliability and performance optimization")
            config_lines.append("CONFIG_TWAI_ISR_IN_IRAM=y")
            config_lines.append(
                "CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID=y")
            config_lines.append(
                "CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT=y")
            config_lines.append(
                "CONFIG_TWAI_ERRATA_FIX_TX_INTR_LOST=y")

        # SPIFFS filesystem configuration for flash logging
        if (hasattr(self, 'intdefines')
                and self.intdefines.get(
                    'HAL_LOGGING_FILESYSTEM_ENABLED') == 1):
            config_lines.append(
                "# SPIFFS filesystem for internal flash logging")
            config_lines.append("CONFIG_SPIFFS_SUPPORTED=y")
            config_lines.append("CONFIG_SPIFFS_CACHE=y")
            config_lines.append("CONFIG_SPIFFS_CACHE_WR=y")
            config_lines.append("CONFIG_SPIFFS_CACHE_STATS=y")
            config_lines.append("CONFIG_SPIFFS_PAGE_CHECK=n")
            config_lines.append("CONFIG_SPIFFS_GC_MAX_RUNS=50")
            config_lines.append(
                "# CONFIG_SPIFFS_GC_STATS is not set")
            config_lines.append("CONFIG_SPIFFS_PAGE_SIZE=512")
            config_lines.append("CONFIG_SPIFFS_OBJ_NAME_LEN=32")
            config_lines.append(
                "# CONFIG_SPIFFS_USE_MAGIC is not set")
            config_lines.append(
                "# CONFIG_SPIFFS_USE_MAGIC_LENGTH is not set")
            config_lines.append(
                "# CONFIG_SPIFFS_FOLLOW_SYMLINKS is not set")
            config_lines.append("CONFIG_VFS_SUPPORT_IO=y")
            config_lines.append("CONFIG_VFS_SUPPORT_DIR=y")
            config_lines.append(
                "# CONFIG_VFS_SUPPORT_SELECT is not set")

        return config_lines

    def write_esp_idf_config(self, filename="sdkconfig.board"):
        '''Write ESP-IDF configuration to file'''
        config_lines = self.generate_esp_idf_config()
        if not config_lines:
            return

        fname = os.path.join(self.outdir, filename)
        self.progress(f"Writing ESP-IDF config to {fname}")
        with open(fname, "w") as f:
            f.write("# Auto-generated ESP-IDF configuration "
                    "for %s\n\n" % self.board)
            for line in config_lines:
                f.write(line + "\n")

    def copy_partition_table(self):
        '''Copy board-specific partition table to build directory'''
        if not self.partition_table_filename:
            return

        # Find the source partition file in the board's hwdef directory
        hwdef_dir = os.path.dirname(self.hwdef[0])
        src = os.path.join(hwdef_dir, self.partition_table_filename)

        if not os.path.exists(src):
            self.error(f"Partition table file not found: {src}")
            return

        # Copy to build output directory
        import shutil
        dst = os.path.join(self.outdir, self.partition_table_filename)
        shutil.copy2(src, dst)
        self.progress(f"Copied partition table: {src} -> {dst}")

    def process_line_define(self, line, depth, a):
        '''handle both numerical and string defines'''
        # Call parent for numerical defines
        super().process_line_define(line, depth, a)

        # Also handle string defines
        result = re.match(
            r'define\s+([A-Z_0-9_]+(?:\([^)]*\))?)\s+(.+)', line)
        if result:
            name = result.group(1)
            value = result.group(2)
            # Store string defines separately
            if name not in self.intdefines:
                if not hasattr(self, 'strdefines'):
                    self.strdefines = {}
                self.strdefines[name] = value

    def write_hwdef_header_content(self, f):
        '''write hwdef.h content'''
        for name in sorted(self.intdefines.keys()):
            f.write("#define %s %s\n" % (name, self.intdefines[name]))

        if hasattr(self, 'strdefines'):
            for name in sorted(self.strdefines.keys()):
                f.write("#define %s %s\n" % (name, self.strdefines[name]))
        f.write("\n")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description='ESP32 hwdef processor')
    parser.add_argument('-D', '--outdir', required=True,
                        help='output directory')
    parser.add_argument('--quiet', action='store_true',
                        help='quiet operation')
    parser.add_argument('hwdef', nargs='+', help='hwdef files')
    args = parser.parse_args()

    eh = ESP32HWDef(args.outdir, args.hwdef, quiet=args.quiet)
    sys.exit(eh.run())
