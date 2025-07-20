import os
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef

class ESP32HWDef(hwdef.HWDef):
    def __init__(self, outdir, hwdef, **kwargs):
        super(ESP32HWDef, self).__init__(outdir=outdir, hwdef=hwdef, **kwargs)
        self.generate_defines = True
        self.board = os.path.basename(os.path.dirname(hwdef[0]))
        self.mcu = 'esp32'
        self.psram_config = {}  # Store PSRAM configuration

    def run(self):
        self.process_hwdefs()
        if self.generate_defines:
            self.generate_hwdef_h()
        # Generate ESP-IDF config if PSRAM is configured
        self.write_esp_idf_config()
        return 0

    def process_line(self, line, depth=0):
        super(ESP32HWDef, self).process_line(line, depth)
        if line.startswith("MCU"):
            self.mcu = line.split()[1]
            self.env_vars['MCU'] = self.mcu
        elif line.startswith("PSRAM_SIZE"):
            parts = line.split()
            if len(parts) >= 2:
                self.psram_config['size'] = parts[1]
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

    def generate_hwdef_h(self):
        '''generate a hwdef.h file'''
        self.progress("Generating hwdef.h")
        fname = os.path.join(self.outdir, "hwdef.h")
        f = open(fname, "w")
        f.write("/* Auto-generated hwdef.h for %s */\n\n" % self.board)
        f.write("#pragma once\n\n")
        for d in sorted(self.intdefines.keys()):
            f.write("#define %s %s\n" % (d, self.intdefines[d]))
        f.close()

    def get_env_vars(self):
        return self.env_vars

    def get_psram_config(self):
        '''Return PSRAM configuration dictionary'''
        return self.psram_config

    def has_psram(self):
        '''Check if board has PSRAM configured'''
        return 'size' in self.psram_config and self.psram_config['size'] != '0MB'

    def generate_esp_idf_config(self):
        '''Generate ESP-IDF configuration settings based on PSRAM config'''
        config_lines = []
        
        if self.has_psram():
            self.progress("Generating ESP-IDF PSRAM configuration")
            
            # Enable PSRAM - correct symbols for ESP-IDF v5.4
            config_lines.append("CONFIG_ESP32S3_SPIRAM_SUPPORT=y")
            config_lines.append("CONFIG_SPIRAM_USE_MALLOC=y")
            
            # Set PSRAM mode based on hwdef
            mode = self.psram_config.get('mode', 'QUAD').upper()
            if mode == 'OPI':
                config_lines.append("CONFIG_SPIRAM_MODE_OCT=y")
                config_lines.append("# CONFIG_SPIRAM_MODE_QUAD is not set")
            else:
                config_lines.append("CONFIG_SPIRAM_MODE_QUAD=y")
                config_lines.append("# CONFIG_SPIRAM_MODE_OCT is not set")
            
            # Set malloc thresholds
            threshold = self.psram_config.get('malloc_threshold', '16384')
            config_lines.append(f"CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL={threshold}")
            
            reserve = self.psram_config.get('reserve_internal', '16384')
            config_lines.append(f"CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL={reserve}")
            
            # Allow BSS segment in SPIRAM
            config_lines.append("CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y")
            
        else:
            self.progress("No PSRAM configured - SPIRAM disabled")
            config_lines.append("# CONFIG_ESP32S3_SPIRAM_SUPPORT is not set")
            
        return config_lines

    def write_esp_idf_config(self, filename="sdkconfig.board"):
        '''Write ESP-IDF configuration to file'''
        config_lines = self.generate_esp_idf_config()
        if not config_lines:
            return
            
        fname = os.path.join(self.outdir, filename)
        self.progress(f"Writing ESP-IDF config to {fname}")
        with open(fname, "w") as f:
            f.write("# Auto-generated ESP-IDF configuration for %s\n\n" % self.board)
            for line in config_lines:
                f.write(line + "\n")

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='ESP32 hwdef processor')
    parser.add_argument('-D', '--outdir', required=True, help='output directory')
    parser.add_argument('hwdef', nargs='+', help='hwdef files')
    args = parser.parse_args()

    eh = ESP32HWDef(args.outdir, args.hwdef)
    sys.exit(eh.run())