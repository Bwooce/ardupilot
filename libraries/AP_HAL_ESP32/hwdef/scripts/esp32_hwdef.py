#!/usr/bin/env python3
'''
setup board.h for ESP32
AP_FLAKE8_CLEAN
'''

import os
import sys
import shlex
import re

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

    def __init__(self, outdir, hwdef, quiet=False, mcu='esp32', **kwargs):
        super(ESP32HWDef, self).__init__(outdir=outdir, hwdef=hwdef, quiet=quiet, **kwargs)
        self.board = os.path.basename(os.path.dirname(hwdef[0]))
        self.mcu = mcu
        self.rcin_pin = None
        self.advanced_build = False
        self.reserved_pins = set()
        self.input_only_pins = set()
        self.strapping_pins = set()
        self.init_hardware_constraints()
        self.serial_pins = {}
        self.rcout_pins = []
        self.spi_buses = []
        self.spi_devices = []
        self.i2c_buses = []
        self.adc_pins = []
        self.pin_assignments = {}
        self.rcin_pin = None
        self.sdspi = None
        self.flash_size_mb = None
        self.psram_size = None
        self.partition_table_filename = None
        self.strdefines = {}

    def init_hardware_constraints(self):
        chip = self.CHIP_DATA.get(self.mcu.upper())
        if chip:
            self.reserved_pins.update(chip.get('reserved', set()))
            self.adc_capable_pins = chip.get('adc', set())
            self.input_only_pins = chip.get('input_only', set())
            self.strapping_pins = chip.get('strapping', set())
            self.progress(f"{self.mcu.upper()} constraints loaded")

    def validate_pin_assignment(self, pin_num, function, pin_name):
        try:
            pstr = str(pin_num).replace('GPIO_NUM_', '')
            if pstr.startswith('ADC1_GPIO'):
                pstr = pstr.replace('ADC1_GPIO', '').replace('_CHANNEL', '')
            if 'ADC_CHANNEL_' in pstr:
                return True
            pin_num_int = int(pstr)
        except Exception:
            return False

        if self.advanced_build:
            if pin_num_int in self.reserved_pins:
                self.error(f"Pin {pin_num_int} is reserved for system use")
            if pin_num_int in self.pin_assignments:
                existing = self.pin_assignments[pin_num_int]
                self.error(f"Pin conflict: GPIO{pin_num_int} assigned to "
                           f"'{existing}' and '{function} {pin_name}'")
            if pin_num_int in self.input_only_pins and function in \
               ['UART_TX', 'RCOUT', 'SPI_SCK', 'SPI_MOSI', 'I2C_SDA', 'CAN_TX']:
                self.error(f"Pin {pin_num_int} is input-only - "
                           f"cannot be used for output {function}")
            if pin_num_int in self.strapping_pins:
                self.progress(f"WARNING: GPIO{pin_num_int} is a strapping pin")

        self.pin_assignments[pin_num_int] = f"{function} {pin_name}"
        return True

    def _is_pin_assignment_line(self, line):
        '''Check if line is a pin assignment using space-separated syntax'''
        line = line.strip()
        if not line or line.startswith('#') or line.startswith('define'):
            return False

        parts = line.split()
        if len(parts) >= 2:
            first_part = parts[0].upper()
            if ('_BAUD' in first_part or '_PROTOCOL' in first_part or
                '_SPEED' in first_part or first_part.endswith('_ENABLED') or
                first_part.startswith('ESP32_')):
                return False
            if ('_PIN' in first_part or first_part.startswith('CAN_') or
                (first_part.startswith('SERIAL') and '_PIN' in first_part) or
                first_part in ['KEY_BOOT'] or first_part.startswith('RMT_')):
                return True
        return False

    def process_line(self, line, depth=0):
        line = line.strip()
        if not line or line.startswith('#'):
            return

        self.alllines.append(line)

        if line.startswith("MCU"):
            self.advanced_build = True
            self.mcu = line.split()[1]
            self.env_vars['MCU'] = self.mcu
            self.init_hardware_constraints()
            super(ESP32HWDef, self).process_line(line, depth)
            return

        if self.advanced_build and self._is_pin_assignment_line(line):
            p = shlex.split(line)
            pin_name, pin_value = p[0].upper(), p[1]
            
            # Serial pin handling
            if pin_name.startswith('SERIAL') and '_PIN' in pin_name:
                if self.validate_pin_assignment(pin_value, 'UART', pin_name):
                    m = re.match(r'SERIAL(\d+)_(TX|RX)_PIN', pin_name)
                    if m:
                        num, ptype = m.group(1), m.group(2).lower()
                        self.serial_pins[f'{ptype}_{num}'] = pin_value
            
            # RCOUT pin handling
            elif pin_name.startswith('RCOUT') and pin_name.endswith('_PIN'):
                if self.validate_pin_assignment(pin_value, 'RCOUT', pin_name):
                    m = re.match(r'RCOUT(\d+)_PIN', pin_name)
                    if m:
                        idx = int(m.group(1)) - 1
                        while len(self.rcout_pins) <= idx:
                            self.rcout_pins.append(None)
                        self.rcout_pins[idx] = pin_value

            # I2C pin handling
            elif pin_name.startswith('I2C') and ('_SDA_PIN' in pin_name or '_SCL_PIN' in pin_name):
                if self.validate_pin_assignment(pin_value, 'I2C', pin_name):
                    m = re.match(r'I2C(\d+)_(SDA|SCL)_PIN', pin_name)
                    if m:
                        num, ptype = int(m.group(1)), m.group(2).lower()
                        bus_entry = next((b for b in self.i2c_buses if b['num'] == num), None)
                        if not bus_entry:
                            bus_entry = {'num': num, 'sda': None, 'scl': None}
                            self.i2c_buses.append(bus_entry)
                        bus_entry[ptype] = pin_value

            # SPI pin handling
            elif pin_name.startswith('SPI') and ('_SCK_PIN' in pin_name or '_MISO_PIN' in pin_name or '_MOSI_PIN' in pin_name):
                if self.validate_pin_assignment(pin_value, 'SPI', pin_name):
                    m = re.match(r'SPI(\d+)_(SCK|MISO|MOSI)_PIN', pin_name)
                    if m:
                        num, ptype = int(m.group(1)), m.group(2).lower()
                        bus_entry = next((b for b in self.spi_buses if b['num'] == num), None)
                        if not bus_entry:
                            bus_entry = {'num': num, 'sck': None, 'miso': None, 'mosi': None}
                            self.spi_buses.append(bus_entry)
                        bus_entry[ptype] = pin_value

            # SPI device chip select pins
            elif pin_name.endswith('_CS_PIN'):
                if self.validate_pin_assignment(pin_value, 'SPI_CS', pin_name):
                    device_name = pin_name.replace('_CS_PIN', '').lower()
                    self.spi_devices.append({'name': device_name, 'cs': pin_value})

            # ADC pin handling
            elif pin_name.startswith('ADC') and pin_name.endswith('_PIN'):
                if self.validate_pin_assignment(pin_value, 'ADC', pin_name):
                    m = re.match(r'ADC(\d+)_PIN', pin_name)
                    if m:
                        self.adc_pins.append({'channel': int(m.group(1)), 'pin': pin_value})

            # RCInput pin handling
            elif pin_name == 'RCIN_PIN':
                if self.validate_pin_assignment(pin_value, 'RCIN', pin_name):
                    self.rcin_pin = pin_value

            return

        if line.startswith("ESP32_SERIAL"):
            p = shlex.split(line)
            if len(p) == 4:
                num, tx, rx = p[1].replace('UART_NUM_', ''), p[2], p[3]
                self.validate_pin_assignment(tx, 'UART_TX', f'SERIAL{num}')
                self.validate_pin_assignment(rx, 'UART_RX', f'SERIAL{num}')
                self.serial_pins[f'tx_{num}'] = tx.replace('GPIO_NUM_', '')
                self.serial_pins[f'rx_{num}'] = rx.replace('GPIO_NUM_', '')
            return
        elif line.startswith("ESP32_RCOUT"):
            p = shlex.split(line)
            for pin in p[1:]:
                self.validate_pin_assignment(pin, 'RCOUT',
                                             f'CH{len(self.rcout_pins)+1}')
                self.rcout_pins.append(pin.replace('GPIO_NUM_', ''))
            return
        elif line.startswith("ESP32_SPIBUS"):
            p = shlex.split(line)
            if len(p) == 6:
                host_str = p[1].upper()
                if 'VSPI' in host_str or 'SPI3' in host_str:
                    num = 3
                elif 'HSPI' in host_str or 'SPI2' in host_str:
                    num = 2
                else:
                    num = 1
                dma, mosi, miso, sck = p[2], p[3], p[4], p[5]
                self.validate_pin_assignment(mosi, 'SPI_MOSI', f'BUS{num}')
                self.validate_pin_assignment(miso, 'SPI_MISO', f'BUS{num}')
                self.validate_pin_assignment(sck, 'SPI_SCK', f'BUS{num}')
                self.spi_buses.append({
                    'num': num,
                    'dma': dma,
                    'sck': sck.replace('GPIO_NUM_', ''),
                    'miso': miso.replace('GPIO_NUM_', ''),
                    'mosi': mosi.replace('GPIO_NUM_', '')
                })
            return
        elif line.startswith("ESP32_SPIDEV"):
            p = shlex.split(line)
            if len(p) == 8:
                name, bus, dev, cs, mode, low, high = p[1], p[2], p[3], p[4], p[5], p[6], p[7]
                self.validate_pin_assignment(cs, 'SPI_CS', name)
                self.spi_devices.append({
                    'name': name,
                    'bus': bus,
                    'device': dev,
                    'cs': cs.replace('GPIO_NUM_', ''),
                    'mode': mode,
                    'lspeed': low,
                    'hspeed': high
                })
            return
        elif line.startswith("ESP32_I2CBUS"):
            p = shlex.split(line)
            if len(p) >= 4:
                num = int(p[1].replace('I2C_NUM_', ''))
                sda, scl = p[2], p[3]
                self.validate_pin_assignment(sda, 'I2C_SDA', f'BUS{num}')
                self.validate_pin_assignment(scl, 'I2C_SCL', f'BUS{num}')
                self.i2c_buses.append({
                    'num': num,
                    'sda': sda.replace('GPIO_NUM_', ''),
                    'scl': scl.replace('GPIO_NUM_', '')
                })
            return
        elif line.startswith("ESP32_SD_SPI"):
            p = shlex.split(line)
            if len(p) == 5:
                bus, miso, mosi, sck, cs = p[1], p[2], p[3], p[4], p[5]
                self.sdspi = {
                    'bus': bus,
                    'miso': miso.replace('GPIO_NUM_', ''),
                    'mosi': mosi.replace('GPIO_NUM_', ''),
                    'sck': sck.replace('GPIO_NUM_', ''),
                    'cs': cs.replace('GPIO_NUM_', '')
                }
            return
        elif line.startswith("FLASH_SIZE_MB"):
            self.flash_size_mb = int(line.split()[1])
            return
        elif line.startswith("PSRAM_SIZE"):
            self.psram_size = line.split()[1]
            return
        elif line.startswith("PARTITION_TABLE_CUSTOM_FILENAME"):
            self.partition_table_filename = line.split()[1]
            return

        super(ESP32HWDef, self).process_line(line, depth)

    def write_hwdef_header(self, outfilename):
        with open(outfilename, "w") as f:
            f.write("/* Auto-generated hwdef.h */\n")
            f.write("#pragma once\n")
            f.write("#include <AP_HAL/board/esp32.h>\n")
            for d in sorted(self.intdefines.keys()):
                f.write(f"#undef {d}\n#define {d} {self.intdefines[d]}\n")
            
            if hasattr(self, 'strdefines'):
                for name in sorted(self.strdefines.keys()):
                    undef_name = name.split('(')[0]
                    f.write(f"#undef {undef_name}\n#define {name} {self.strdefines[name]}\n")

            # Automatically handle ADC configuration if not explicitly defined
            if not self.adc_pins and 'HAL_USE_ADC' not in self.intdefines:
                f.write("#undef HAL_DISABLE_ADC_DRIVER\n#define HAL_DISABLE_ADC_DRIVER 1\n")
                f.write("#undef HAL_USE_ADC\n#define HAL_USE_ADC 0\n")

            chip = self.CHIP_DATA.get(self.mcu.upper())
            if chip:
                for feat in chip.get('features', []):
                    if feat not in self.intdefines:
                        f.write(f"#ifndef {feat}\n#define {feat} 1\n#endif\n")

            if self.serial_pins:
                f.write("\n#define HAL_ESP32_UART_DEVICES \\\n")
                entries = []
                for i in range(3):
                    tx = self.serial_pins.get(f'tx_{i}')
                    rx = self.serial_pins.get(f'rx_{i}')
                    if tx and rx:
                        entries.append(f"    {{ .rx = GPIO_NUM_{rx}, .tx = GPIO_NUM_{tx} }}")
                f.write(",\\\n".join(entries) + "\n")

            if self.rcout_pins:
                valid = [p for p in self.rcout_pins if p is not None]
                f.write(f"\n#ifndef HAL_ESP32_RCOUT_MAX\n#define "
                        f"HAL_ESP32_RCOUT_MAX {len(valid)}\n#endif\n")
                f.write(f"#ifndef HAL_ESP32_RCOUT\n#define HAL_ESP32_RCOUT "
                        f"{{ {','.join(['GPIO_NUM_'+str(p) for p in valid])} }}\n#endif\n")

            if self.spi_buses:
                f.write("\n#define HAL_ESP32_SPI_BUSES \\\n")
                entries = []
                for b in self.spi_buses:
                    dma = b.get('dma', '1')
                    entries.append(f"    {{.host=SPI{b['num']}_HOST, .dma_ch={dma}, "
                                   f".mosi=GPIO_NUM_{b['mosi']}, .miso=GPIO_NUM_{b['miso']}, "
                                   f".sclk=GPIO_NUM_{b['sck']}}}")
                f.write(",\\\n".join(entries) + "\n")

            if self.spi_devices:
                f.write("\n#define HAL_ESP32_SPI_DEVICES \\\n")
                entries = []
                for i, dev in enumerate(self.spi_devices):
                    bus = dev.get('bus', '0')
                    mode = dev.get('mode', '0')
                    lspeed = dev.get('lspeed', '1*MHZ')
                    hspeed = dev.get('hspeed', '10*MHZ')
                    entries.append(f"    {{.name=\"{dev['name']}\", .bus={bus}, .device={i}, "
                                   f".cs=GPIO_NUM_{dev['cs']}, .mode={mode}, "
                                   f".lspeed={lspeed}, .hspeed={hspeed}}}")
                f.write(",\\\n".join(entries) + "\n")

            if self.i2c_buses:
                f.write("\n#define HAL_ESP32_I2C_BUSES \\\n")
                entries = []
                for b in self.i2c_buses:
                    entries.append(f"    {{.port=I2C_NUM_{b['num']}, .sda=GPIO_NUM_{b['sda']}, "
                                   f".scl=GPIO_NUM_{b['scl']}, .speed=400*KHZ, .internal=true}}")
                f.write(",\\\n".join(entries) + "\n")

            if self.adc_pins:
                f.write("\n#define HAL_ESP32_ADC_PINS \\\n")
                entries = []
                for a in self.adc_pins:
                    entries.append(f"    {{ADC1_GPIO{a['pin']}_CHANNEL, 11, {a['pin']}}}")
                f.write(",\\\n".join(entries) + "\n")

            if self.rcin_pin:
                f.write(f"\n#define HAL_ESP32_RCIN GPIO_NUM_{self.rcin_pin}\n")

            if self.sdspi:
                f.write(f"\n#ifndef HAL_ESP32_SD_SPI\n#define HAL_ESP32_SD_SPI "
                        f"{{.host=SPI{self.sdspi['bus']}_HOST, .dma_ch=1, "
                        f".mosi={self.sdspi['mosi']}, .miso={self.sdspi['miso']}, "
                        f".sclk={self.sdspi['sclk']}, .cs={self.sdspi['cs']}}}\n#endif\n")

            f.write("\n")
            self.write_IMU_config(f)
            self.write_BARO_config(f)
            self.write_MAG_config(f)


    def generate_esp_idf_config(self):
        config_lines = []
        flash_size = self.flash_size_mb
        if not flash_size:
            # Chip-specific defaults if not specified
            if self.mcu.upper() == 'ESP32S3':
                flash_size = 8
            else:
                flash_size = 4
        
        # Set the selected flash size and its corresponding string value
        config_lines.append(f"CONFIG_ESPTOOLPY_FLASHSIZE_{flash_size}MB=y")
        config_lines.append(f'CONFIG_ESPTOOLPY_FLASHSIZE="{flash_size}MB"')
        # Explicitly disable other common flash sizes to ensure override
        for size in [1, 2, 4, 8, 16, 32, 64, 128]:
            if size != flash_size:
                config_lines.append(f"# CONFIG_ESPTOOLPY_FLASHSIZE_{size}MB is not set")

        if self.psram_size:
            config_lines.append("CONFIG_SPIRAM=y")
            config_lines.append("CONFIG_SPIRAM_TYPE_AUTO=y")
            config_lines.append("CONFIG_SPIRAM_MODE_QUAD=y")
        if self.partition_table_filename:
            abs_path = os.path.join(self.outdir, self.partition_table_filename)
            config_lines.append("CONFIG_PARTITION_TABLE_CUSTOM=y")
            config_lines.append(f'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="{abs_path}"')
            config_lines.append("CONFIG_PARTITION_TABLE_OFFSET=0x10000")


        hal_with_wifi = self.intdefines.get('HAL_WITH_WIFI', '1')
        if str(hal_with_wifi) == '0':
            config_lines.append("# CONFIG_ESP_WIFI_ENABLED is not set")
        else:
            config_lines.append("CONFIG_ESP_WIFI_ENABLED=y")

        # Essential coredump and panic behavior for ArduPilot on ESP32
        config_lines.extend([
            "CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y",
            "CONFIG_ESP_SYSTEM_PANIC_PRINT_HALT=y",
            "CONFIG_ESP_COREDUMP_MAX_TASKS_NUM=64",
            "CONFIG_ESP_TASK_WDT_INIT=n",
        ])

        return config_lines

    def write_esp_idf_config(self, filename="sdkconfig.board"):
        config_lines = self.generate_esp_idf_config()
        if not config_lines:
            return
        fname = os.path.join(self.outdir, filename)
        with open(fname, "w") as f:
            f.write(f"# Auto-generated ESP-IDF configuration for {self.board}\n")
            for line in config_lines:
                f.write(line + "\n")

    def copy_partition_table(self):
        if not self.partition_table_filename:
            return
        hwdef_dir = os.path.dirname(self.hwdef[0])
        src = os.path.join(hwdef_dir, self.partition_table_filename)
        if os.path.exists(src):
            import shutil
            dst = os.path.join(self.outdir, self.partition_table_filename)
            shutil.copy2(src, dst)

    def process_line_define(self, line, depth, a):
        '''handle both numerical and string defines'''
        super().process_line_define(line, depth, a)
        result = re.match(r'define\s+([A-Z_0-9_]+(?:\([^)]*\))?)\s+(.+)', line)
        if result:
            name, value = result.group(1), result.group(2)
            if name not in self.intdefines:
                self.strdefines[name] = value

    def run(self):
        super(ESP32HWDef, self).run()
        self.write_esp_idf_config()
        self.copy_partition_table()
        return 0


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='ESP32 hwdef processor')
    parser.add_argument('-D', '--outdir', required=True, help='output directory')
    parser.add_argument('--quiet', action='store_true', help='quiet operation')
    parser.add_argument('--mcu', default='esp32', help='MCU type')
    parser.add_argument('hwdef', nargs='+', help='hwdef files')
    args = parser.parse_args()

    eh = ESP32HWDef(args.outdir, args.hwdef, quiet=args.quiet, mcu=args.mcu)
    sys.exit(eh.run())
