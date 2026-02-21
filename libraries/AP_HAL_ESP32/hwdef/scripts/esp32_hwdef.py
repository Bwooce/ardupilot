import os
import sys
import re
import shlex
import argparse

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef

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
            'features': ['HAL_ESP32_HAS_MCPWM', 'HAL_ESP32_HAS_USB_SERIAL_JTAG', 'HAL_ESP32_LARGE_BUFFERS']
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
            'features': ['HAL_ESP32_HAS_MCPWM', 'HAL_ESP32_HAS_MIPI', 'HAL_ESP32_HAS_USB_SERIAL_JTAG', 'HAL_ESP32_LARGE_BUFFERS']
        }
    }

    def __init__(self, outdir, hwdef, quiet=False, **kwargs):
        super(ESP32HWDef, self).__init__(outdir=outdir, hwdef=hwdef, quiet=quiet, **kwargs)
        self.board = os.path.basename(os.path.dirname(hwdef[0]))
        self.mcu = 'esp32'
        self.advanced_build = False
        self.esp32_serials = []
        self.esp32_rcout = []
        self.esp32_spibus = []
        self.esp32_spidev = []
        self.esp32_i2cbus = []
        self.esp32_adcs = []
        self.reserved_pins = set()
        self.input_only_pins = set()
        self.strapping_pins = set()
        self.pin_assignments = {}
        self.flash_size_mb = None
        self.psram_size = None
        self.partition_table_filename = None

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
            if 'ADC_CHANNEL_' in pstr or 'HOST' in pstr or 'CH_AUTO' in pstr or 'SERIAL_NUM' in pstr or 'UART_NUM' in pstr:
                return True
            pin_num_int = int(pstr)
        except:
            return False
            
        if self.advanced_build:
            if pin_num_int in self.reserved_pins:
                self.error(f"Pin {pin_num_int} is reserved for system use")
            if pin_num_int in self.pin_assignments:
                existing = self.pin_assignments[pin_num_int]
                self.error(f"Pin conflict: GPIO{pin_num_int} assigned to '{existing}' and '{function} {pin_name}'")
            if pin_num_int in self.input_only_pins and function in ['UART_TX', 'RCOUT', 'SPI_SCK', 'SPI_MOSI', 'I2C_SDA', 'CAN_TX']:
                self.error(f"Pin {pin_num_int} is input-only - cannot be used for output {function}")
            if pin_num_int in self.strapping_pins:
                self.progress(f"WARNING: GPIO{pin_num_int} is a strapping pin")
                
        self.pin_assignments[pin_num_int] = f"{function} {pin_name}"
        return True

    def process_line(self, line, depth=0):
        line = line.strip()
        if not line or line.startswith('#'): return
        
        # Keep track of all lines for output, like ChibiOS
        self.alllines.append(line)
        
        a = shlex.split(line, posix=False)
        
        # MCU directive enables advanced logic and dynamic sdkconfig
        if a[0] == "MCU":
            self.advanced_build = True
            self.mcu = a[1]
            self.env_vars['MCU'] = self.mcu
            self.init_hardware_constraints()
            super(ESP32HWDef, self).process_line(line, depth)
            return

        # Platform-specific tags (Supported in both V1 and V2 modes)
        if a[0] == "ESP32_SERIAL":
            if len(a) == 4:
                self.validate_pin_assignment(a[2], 'UART_RX', f'SERIAL{a[1]}')
                self.validate_pin_assignment(a[3], 'UART_TX', f'SERIAL{a[1]}')
                self.esp32_serials.append(a[1:])
            return
        elif a[0] == "ESP32_RCOUT":
            for pin in a[1:]:
                self.validate_pin_assignment(pin, 'RCOUT', f'CH{len(self.esp32_rcout)+1}')
                self.esp32_rcout.append(pin)
            return
        elif a[0] == "ESP32_SPIBUS":
            if len(a) == 5:
                (host, dma_ch, mosi, miso, sclk) = a[1:]
                self.validate_pin_assignment(mosi, 'SPI_MOSI', host)
                self.validate_pin_assignment(miso, 'SPI_MISO', host)
                self.validate_pin_assignment(sclk, 'SPI_SCK', host)
                self.esp32_spibus.append(a[1:])
            return
        elif a[0] == "ESP32_SPIDEV":
            if len(a) == 8:
                (name, bus, device, cs, mode, lowspeed, highspeed) = a[1:]
                self.validate_pin_assignment(cs, 'SPI_CS', name)
                self.esp32_spidev.append(a[1:])
            return
        elif a[0] == "ESP32_I2CBUS":
            if len(a) >= 4:
                (port, sda, scl) = a[1:4]
                self.validate_pin_assignment(sda, 'I2C_SDA', port)
                self.validate_pin_assignment(scl, 'I2C_SCL', port)
                self.esp32_i2cbus.append(a[1:])
            return
        elif a[0] == "ESP32_ADC_PIN":
            if len(a) == 4:
                (pin, gain, ardupilot_pin) = a[1:]
                self.validate_pin_assignment(pin, 'ADC', f'CH{ardupilot_pin}')
                self.esp32_adcs.append(a[1:])
            return
        elif a[0] == "FLASH_SIZE_MB":
            self.flash_size_mb = int(a[1])
            return
        elif a[0] == "PSRAM_SIZE":
            self.psram_size = a[1]
            return

        super(ESP32HWDef, self).process_line(line, depth)
        
        if a[0] == "RESERVED_PINS":
            for pin in a[1:]:
                self.reserved_pins.add(int(pin.replace('GPIO_NUM_', '')))

    def write_hwdef_header_content(self, f):
        self.advanced_build = ('MCU' in self.env_vars)
        
        # Write all definitions from alllines, preserving strings
        for d in self.alllines:
            if d.startswith('define '):
                name = d.split()[1].split('(')[0]
                f.write("#undef %s\n#define %s\n" % (name, d[7:]))
            elif d.startswith('undef '):
                f.write("#undef %s\n" % d[6:])

        if self.advanced_build:
            f.write("#ifndef HAL_ESP32_HWDEF_V2\n#define HAL_ESP32_HWDEF_V2 1\n#endif\n")
            chip = self.CHIP_DATA.get(self.mcu.upper())
            if chip:
                for feat in chip['features']:
                    f.write("#ifndef %s\n#define %s 1\n#endif\n" % (feat, feat))

        self.write_SPI_bus_table(f)
        self.write_SPI_device_table(f)
        self.write_IMU_config(f)
        self.write_MAG_config(f)
        self.write_BARO_config(f)
        self.write_SERIAL_config(f)
        self.write_RCOUT_config(f)
        self.write_ADC_config(f)
        self.write_I2C_config(f)

    def write_device_table_esp32(self, f, name, definition, devices):
        '''write a device table, but ALWAYS define the macro for ESP32 HAL'''
        f.write(f"\n/* {name} */\n#ifndef {definition}\n#define {definition} \\\n")
        if not devices:
            f.write("    { }") # Empty entry
        else:
            f.write(',\\\n'.join(devices))
        f.write("\n#endif\n")

    def write_SPI_bus_table(self, f):
        buslist = []
        for bus in self.esp32_spibus:
            if len(bus) != 5: continue
            (host, dma_ch, mosi, miso, sclk) = bus
            buslist.append(f"{{ .host={host}, .dma_ch={dma_ch}, .mosi={mosi}, .miso={miso}, .sclk={sclk} }}")
        self.write_device_table_esp32(f, "SPI buses", "HAL_ESP32_SPI_BUSES", buslist)

    def write_SPI_device_table(self, f):
        devlist = []
        for dev in self.esp32_spidev:
            if len(dev) != 7: continue
            (name, bus, device, cs, mode, lowspeed, highspeed) = dev
            devlist.append(f'{{.name= "{name}", .bus={bus}, .device={device}, .cs={cs}, .mode={mode}, .lspeed={lowspeed}, .hspeed={highspeed}}}')
        self.write_device_table_esp32(f, "SPI devices", "HAL_ESP32_SPI_DEVICES", devlist)

    def write_SERIAL_config(self, f):
        seriallist = []
        for serial in self.esp32_serials:
            if len(serial) != 3: continue
            (port, rxpin, txpin) = serial
            seriallist.append(f"{{ .port={port}, .rx={rxpin}, .tx={txpin} }}")
        self.write_device_table_esp32(f, "serial devices", "HAL_ESP32_UART_DEVICES", seriallist)

    def write_RCOUT_config(self, f):
        f.write(f"\n#ifndef HAL_ESP32_RCOUT_MAX\n#define HAL_ESP32_RCOUT_MAX {len(self.esp32_rcout)}\n#endif\n")
        f.write(f"#ifndef HAL_ESP32_RCOUT\n#define HAL_ESP32_RCOUT {{{','.join(self.esp32_rcout)}}}\n#endif\n")

    def write_ADC_config(self, f):
        adclist = []
        for adc in self.esp32_adcs:
            if len(adc) != 3: continue
            (pin, gain, ardupilot_pin) = adc
            adclist.append(f"{{ {pin}, {gain}, {ardupilot_pin} }}")
        self.write_device_table_esp32(f, "ADC pins", "HAL_ESP32_ADC_PINS", adclist)

    def write_I2C_config(self, f):
        buslist = []
        for bus in self.esp32_i2cbus:
            if len(bus) < 3: continue
            port, sda, scl = bus[0:3]
            speed = bus[3] if len(bus) > 3 else "400*KHZ"
            internal = bus[4] if len(bus) > 4 else "1"
            soft = bus[5] if len(bus) > 5 else "0"
            buslist.append(f"{{ .port={port}, .sda={sda}, .scl={scl}, .speed={speed}, .internal={internal}, .soft={soft} }}")
        self.write_device_table_esp32(f, "I2C buses", "HAL_ESP32_I2C_BUSES", buslist)

    def generate_esp_idf_config(self):
        if not self.advanced_build: return []
        config_lines = []
        if self.flash_size_mb:
            config_lines.append(f"CONFIG_ESPTOOLPY_FLASHSIZE_{self.flash_size_mb}MB=y")
        if self.psram_size:
            config_lines.append("CONFIG_SPIRAM=y")
            config_lines.append("CONFIG_SPIRAM_TYPE_AUTO=y")
            config_lines.append("CONFIG_SPIRAM_MODE_QUAD=y")
        
        hal_with_wifi = self.intdefines.get('HAL_WITH_WIFI', '1')
        if str(hal_with_wifi) == '0':
            config_lines.append("# CONFIG_ESP_WIFI_ENABLED is not set")
        else:
            config_lines.append("CONFIG_ESP_WIFI_ENABLED=y")
        return config_lines

    def write_esp_idf_config(self, filename="sdkconfig.board"):
        if not self.advanced_build: return
        config_lines = self.generate_esp_idf_config()
        if not config_lines: return
        fname = os.path.join(self.outdir, filename)
        with open(fname, "w") as f:
            f.write("# Auto-generated config\n")
            for line in config_lines: f.write(line + "\n")

if __name__ == '__main__':
    parser = argparse.ArgumentParser("esp32_hwdef.py")
    parser.add_argument(
        '-D', '--outdir', type=str, default="/tmp", help='Output directory')
    parser.add_argument(
        'hwdef', type=str, nargs='+', default=None, help='hardware definition file')
    parser.add_argument(
        '--quiet', action='store_true', default=False, help='quiet running')

    args = parser.parse_args()

    eh = ESP32HWDef(
        outdir=args.outdir,
        hwdef=args.hwdef,
        quiet=args.quiet,
    )
    eh.run()
    eh.write_esp_idf_config()
