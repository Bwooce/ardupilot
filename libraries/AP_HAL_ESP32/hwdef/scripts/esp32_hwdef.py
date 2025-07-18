

import os
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef

class ESP32HWDef(hwdef.HWDef):
    def __init__(self, outdir, hwdef, **kwargs):
        super(ESP32HWDef, self).__init__(outdir=outdir, hwdef=hwdef, **kwargs)
        self.generate_defines = True
        self.board = os.path.basename(os.path.dirname(hwdef[0]))

    def run(self):
        self.process_hwdefs()
        if self.generate_defines:
            self.generate_hwdef_h()
        return 0

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

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='ESP32 hwdef processor')
    parser.add_argument('-D', '--outdir', required=True, help='output directory')
    parser.add_argument('hwdef', nargs='+', help='hwdef files')
    args = parser.parse_args()

    eh = ESP32HWDef(args.outdir, args.hwdef)
    sys.exit(eh.run())

