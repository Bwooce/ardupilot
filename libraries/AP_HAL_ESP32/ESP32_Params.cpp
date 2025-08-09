/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ESP32_Params.h"
#include "ESP32_Debug.h"

namespace ESP32 {

ESP32Params* esp32_params() {
    return ESP32Params::get_singleton();
}

// Parameter table definition
const struct AP_Param::GroupInfo ESP32Params::var_info[] = {
    // @Param: DEBUG_LEVEL
    // @DisplayName: ESP32 HAL Debug Level
    // @Description: Controls ESP32 HAL debug output via MAVLink STATUSTEXT messages. 0=Disabled, 1=Errors only, 2=Warnings+Errors, 3=Info+Warnings+Errors, 4=Verbose, 5=All debug output
    // @Values: 0:Disabled, 1:Errors, 2:Warnings, 3:Info, 4:Verbose, 5:All
    // @User: Advanced
    // @RebootRequired: False
    AP_GROUPINFO("DEBUG_LEVEL", 1, ESP32Params, debug_level, ESP32_DEBUG_INFO),
    
    AP_GROUPEND
};

void ESP32Params::init() {
    // Apply debug level setting to debug system
    if (esp32_debug()) {
        esp32_debug()->set_level(debug_level.get());
    }
}

} // namespace ESP32