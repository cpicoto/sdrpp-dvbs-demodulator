#include "s2_scrambling.h"

/*
Adapted from LeanDVB
*/

namespace dsp {
    namespace dvbs2 {
        S2Scrambling::S2Scrambling(int codenum) {
            uint32_t stx = 0x00001, sty = 0x3ffff;
            // x starts at codenum, wraps at index 2^18-1 by design
            for (int i = 0; i < codenum; ++i)
                stx = lfsr_x(stx);
            // First half of sequence is LSB of scrambling angle
            for (int i = 0; i < 131072; ++i) {
                int zn = (stx ^ sty) & 1;
                Rn[i] = zn;
                stx = lfsr_x(stx);
                sty = lfsr_y(sty);
            }
            // Second half is MSB
            for (int i = 0; i < 131072; ++i) {
                int zn = (stx ^ sty) & 1;
                Rn[i] |= zn << 1;
                stx = lfsr_x(stx);
                sty = lfsr_y(sty);
            }
        }

        S2Scrambling::~S2Scrambling() {
        }

        void S2Scrambling::reset() {
            pos = 0;
        }

        complex_t S2Scrambling::descramble(complex_t &p) {
            r = Rn[pos++];

            switch (r) {
            case 3:
                res.re = -p.im;
                res.im = p.re;
                break;
            case 2:
                res.re = -p.re;
                res.im = -p.im;
                break;
            case 1:
                res.re = p.im;
                res.im = -p.re;
                break;
            default:
                res = p;
            }

            return res;
        }

        complex_t S2Scrambling::scramble(complex_t &p) {
            r = Rn[pos++];

            switch (r) {
            case 3:
                res.im = -p.re;
                res.re = p.im;
                break;
            case 2:
                res.re = -p.re;
                res.im = -p.im;
                break;
            case 1:
                res.im = p.re;
                res.re = -p.im;
                break;
            default:
                res = p;
            }

            return res;
        }
    }
}
