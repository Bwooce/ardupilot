/*
 * PicoLibC <cmath> compatibility shim for esp-dsp.
 *
 * IDF 6.0+ uses PicoLibC instead of newlib. PicoLibC's <cmath> doesn't
 * place math functions (cos, sin, atan, etc.) in the std:: namespace.
 * esp-dsp's kalman module uses std::cos/sin/atan which fail to compile.
 *
 * This header is force-included via -include for the esp-dsp target only.
 * It pulls the C math.h names into namespace std so std::cos etc. resolve.
 */
#pragma once

#ifndef __ASSEMBLER__
#include <math.h>
#include <stdlib.h>
#endif

#ifdef __cplusplus
namespace std {
    using ::cos;
    using ::sin;
    using ::atan;
    using ::sqrt;
    using ::fabs;
    using ::abs;
}
#endif
