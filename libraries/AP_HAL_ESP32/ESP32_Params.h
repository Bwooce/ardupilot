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

#pragma once

#include <AP_Param/AP_Param.h>

/*
  ESP32 HAL Parameters
  
  Provides runtime configuration for ESP32-specific features including
  debug output levels, buffer sizes, and performance tuning.
*/

namespace ESP32 {

class ESP32Params {
public:
    static const struct AP_Param::GroupInfo var_info[];
    
    // Get singleton instance
    static ESP32Params* get_singleton() {
        static ESP32Params instance;
        return &instance;
    }
    
    // Debug level parameter (0=disabled, 1=errors, 2=warnings, 3=info, 4=verbose, 5=all)
    AP_Int8 debug_level;
    
    // Initialize parameters and apply settings
    void init();
    
private:
    ESP32Params() = default;
    ESP32Params(const ESP32Params&) = delete;
    ESP32Params& operator=(const ESP32Params&) = delete;
};

// Global access function
ESP32Params* esp32_params();

} // namespace ESP32