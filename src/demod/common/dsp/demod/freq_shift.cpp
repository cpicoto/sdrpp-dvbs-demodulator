#include "freq_shift.h"

// Fix for MSVC M_PI definition
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace dsp {
    int FreqShift::process(int count, complex_t* in, complex_t* out) {
        for (int i = 0; i < count; i++) {
            complex_t shift = math::phasor(-curr_phase);
            complex_t x = in[i] * shift;
            curr_phase += curr_freq;
            // Wrap phase
            while (curr_phase > (2 * M_PI))
                curr_phase -= 2 * M_PI;
            while (curr_phase < (-2 * M_PI))
                curr_phase += 2 * M_PI;
            out[i] = x;
        }
        return count;
    }
}
