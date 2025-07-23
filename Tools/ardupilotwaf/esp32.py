# encoding: utf-8

# flake8: noqa

"""
Waf tool for ESP32 build
"""

from waflib import Build, ConfigSet, Configure, Context, Task, Utils
from waflib import Errors, Logs
from waflib.TaskGen import before, after_method, before_method, feature
from waflib.Configure import conf
from collections import OrderedDict

import os
import shutil
import sys
import re
import pickle
import subprocess

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_ESP32/hwdef/scripts'))
import esp32_hwdef

def configure(cfg):
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()
    def bldpath(path):
        return bldnode.make_node(path).abspath()

    #Load cmake builder and make
    cfg.load('cmake')

    #define env and location for the cmake esp32 file
    env = cfg.env
    mcu = env.MCU
    env.AP_HAL_ESP32 = srcpath('libraries/AP_HAL_ESP32/targets/'+mcu+'/esp-idf')
    env.AP_PROGRAM_FEATURES += ['esp32_ap_program']
    env.append_value('DEFINES', 'USE_USER_HELPERS=1')

    env.ESP_IDF_PREFIX_REL = 'esp-idf'

    prefix_node = bldnode.make_node(env.ESP_IDF_PREFIX_REL)
    env.ESP32_TARGET = mcu
    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')
    env.APJ_TOOL = srcpath('Tools/scripts/apj_tool.py')

    #Check if esp-idf env are loaded, or load it
    try:
        env.IDF = os.environ['IDF_PATH']
    except:
        env.IDF = cfg.srcnode.abspath()+"/modules/esp_idf"
    print("USING EXPRESSIF IDF:"+str(env.IDF))

    # setup cmake
    cfg.env['IDF_TARGET'] = mcu
    
    # Process hwdef files to generate board-specific configuration
    board_name = getattr(cfg.options, 'board', '').replace('esp32', '')
    if board_name:
        hwdef_dir = os.path.join(cfg.srcnode.abspath(), f'libraries/AP_HAL_ESP32/hwdef/{cfg.options.board}')
        hwdef_file = os.path.join(hwdef_dir, 'hwdef.dat')
        
        if os.path.exists(hwdef_file):
            # Run the ESP32 hwdef processor
            hwdef_script = os.path.join(cfg.srcnode.abspath(), 'libraries/AP_HAL_ESP32/hwdef/scripts/esp32_hwdef.py')
            if os.path.exists(hwdef_script):
                import subprocess
                build_dir = cfg.bldnode.abspath()
                cmd = [cfg.env.PYTHON[0], hwdef_script, '-D', build_dir, hwdef_file]
                print(f"Processing hwdef: {' '.join(cmd)}")
                try:
                    subprocess.run(cmd, check=True, cwd=cfg.srcnode.abspath())
                except subprocess.CalledProcessError as e:
                    cfg.fatal(f"hwdef processing failed: {e}")
    
    # Build list of sdkconfig.defaults files: target-level + board-level (if exists)
    target_sdkconfig = os.path.join(cfg.srcnode.abspath(), f'libraries/AP_HAL_ESP32/targets/{mcu}/esp-idf/sdkconfig.defaults')
    board_sdkconfig = os.path.join(cfg.bldnode.abspath(), 'sdkconfig.board')
    
    sdkconfig_list = [target_sdkconfig]
    if os.path.exists(board_sdkconfig):
        # Create a combined sdkconfig.defaults that includes both target and board configs
        combined_sdkconfig = os.path.join(cfg.bldnode.abspath(), 'sdkconfig.combined')
        print(f"Creating combined ESP-IDF config: {combined_sdkconfig}")
        
        with open(combined_sdkconfig, 'w') as combined_file:
            # Include target-level configuration
            with open(target_sdkconfig, 'r') as target_file:
                combined_file.write(f"# Target-level configuration from {target_sdkconfig}\n")
                combined_file.write(target_file.read())
                combined_file.write("\n")
            
            # Include board-specific configuration
            with open(board_sdkconfig, 'r') as board_file:
                combined_file.write(f"# Board-specific configuration from {board_sdkconfig}\n")
                combined_file.write(board_file.read())
        
        cfg.env['ESP_IDF_SDKCONFIG_DEFAULTS'] = combined_sdkconfig
        print(f"Using combined ESP-IDF config: {combined_sdkconfig}")
    else:
        cfg.env['ESP_IDF_SDKCONFIG_DEFAULTS'] = target_sdkconfig
    cfg.env['PROJECT_DIR'] = cfg.bldnode.abspath()
    cfg.env['SDKCONFIG'] = os.path.join(cfg.bldnode.abspath(), 'esp-idf_build/sdkconfig')
    cfg.env['PYTHON'] = cfg.env.get_flat('PYTHON')

# delete the output sdkconfig file when the input defaults changes. we take the
# stamp as the output so we can compute the path to the sdkconfig, yet it
# doesn't have to exist when we're done.
class clean_sdkconfig(Task.Task):
    def keyword(self):
        return "delete sdkconfig generated from"

    def run(self):
        prefix = ".clean-stamp-"
        for out in self.outputs:
            if not out.name.startswith(prefix):
                raise ValueError("not a stamp file: "+out)
            dest = out.parent.abspath()+"/"+out.name[len(prefix):]
            if os.path.exists(dest):
                os.unlink(dest)

            # waf needs the output to exist after the task, so touch it
            open(out.abspath(), "w").close()

def pre_build(self):
    """Configure esp-idf as lib target"""
    lib_vars = OrderedDict()
    lib_vars['ARDUPILOT_CMD'] = self.cmd
    lib_vars['WAF_BUILD_TARGET'] = self.targets
    lib_vars['ARDUPILOT_LIB'] = self.bldnode.find_or_declare('lib/').abspath()
    lib_vars['ARDUPILOT_BIN'] = self.bldnode.find_or_declare('lib/bin').abspath()
    target = self.env.ESP32_TARGET
    esp_idf = self.cmake(
            name='esp-idf',
            cmake_vars=lib_vars,
            cmake_src='libraries/AP_HAL_ESP32/targets/'+target+'/esp-idf',
            cmake_bld='esp-idf_build',
            )

    esp_idf_showinc = esp_idf.build('showinc', target='esp-idf_build/includes.list')

    # task to delete the sdkconfig (thereby causing it to be regenerated) when
    # the .defaults changes. it uses a stamp to find the sdkconfig. changing
    # the sdkconfig WILL NOT cause it to be deleted as it's not an input. this
    # is by design so the user can tweak it for testing purposes.
    clean_sdkconfig_task = esp_idf_showinc.create_task("clean_sdkconfig",
        src=self.srcnode.find_or_declare(self.env.AP_HAL_ESP32+"/sdkconfig.defaults"),
        tgt=self.bldnode.find_or_declare("esp-idf_build/.clean-stamp-sdkconfig"))

    esp_idf_showinc.post()

    # ensure the sdkconfig will be deleted before the cmake configure occurs
    # that regenerates it
    esp_idf_showinc.cmake_config_task.set_run_after(clean_sdkconfig_task)

    from waflib import Task
    class load_generated_includes(Task.Task):
        """After includes.list generated include it in env"""
        always_run = True
        def run(tsk):
            bld = tsk.generator.bld
            includes = bld.bldnode.find_or_declare('esp-idf_build/includes.list').read().split()
            #print(includes)
            bld.env.prepend_value('INCLUDES', includes)

    tsk = load_generated_includes(env=self.env)
    tsk.set_inputs(self.path.find_resource('esp-idf_build/includes.list'))
    self.add_to_group(tsk)


@feature('esp32_ap_program')
@after_method('process_source')
def esp32_firmware(self):
    self.link_task.always_run = True
    esp_idf = self.bld.cmake('esp-idf')

    build = esp_idf.build('all', target='esp-idf_build/ardupilot.bin')
    build.post()

    build.cmake_build_task.set_run_after(self.link_task)

    # optional upload is last
    if self.bld.options.upload:
        flasher = esp_idf.build('flash')
        flasher.post()
