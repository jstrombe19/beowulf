#include "fft_q15.h"
#include <stdlib.h>
#include <math.h>

// ---------- fixed-point helpers ----------
static inline int16_t q15_mul(int16_t a, int16_t b) {
    // (a*b)>>15 with rounding, using 32-bit intermediate
    int32_t t = (int32_t)a * (int32_t)b;      // 31..-31
    t += (1 << 14);                           // round
    return (int16_t)(t >> 15);
}

static inline void cplx_mul_q15(int16_t ar, int16_t ai,
                                int16_t br, int16_t bi,
                                int16_t *cr, int16_t *ci) {
    // (a+jb)*(b+jd) scaled back to Q15
    int32_t r = (int32_t)ar * br - (int32_t)ai * bi; // Q30
    int32_t i = (int32_t)ar * bi + (int32_t)ai * br; // Q30
    // round and shift
    r += (1<<14); i += (1<<14);
    *cr = (int16_t)(r >> 15);
    *ci = (int16_t)(i >> 15);
}

// ---------- twiddles ----------
static int16_t *tw_r = NULL;
static int16_t *tw_i = NULL;
static uint16_t tw_N = 0;

bool fft_q15_init(uint16_t N) {
    if (N < 2) return false;
    // must be power of two
    if ( (N & (N-1)) != 0 ) return false;

    if (tw_N == N && tw_r && tw_i) return true;

    free(tw_r); free(tw_i);
    tw_r = (int16_t*)malloc((N/2) * sizeof(int16_t));
    tw_i = (int16_t*)malloc((N/2) * sizeof(int16_t));
    if (!tw_r || !tw_i) return false;

    // Generate twiddles: W_N^k = exp(-j*2*pi*k/N)
    for (uint16_t k = 0; k < N/2; ++k) {
        double ang = -2.0 * M_PI * (double)k / (double)N;
        double cr = cos(ang);
        double ci = sin(ang);
        int32_t qr = (int32_t)lrint(cr * 32767.0);
        int32_t qi = (int32_t)lrint(ci * 32767.0);
        if (qr > 32767) qr = 32767; if (qr < -32768) qr = -32768;
        if (qi > 32767) qi = 32767; if (qi < -32768) qi = -32768;
        tw_r[k] = (int16_t)qr;
        tw_i[k] = (int16_t)qi;
    }
    tw_N = N;
    return true;
}

// bit-reverse index
static uint16_t brev(uint16_t x, uint16_t bits) {
    uint16_t r = 0;
    for (uint16_t i=0;i<bits;i++){ r = (uint16_t)((r<<1) | (x & 1)); x >>= 1; }
    return r;
}

void fft_q15(cq15_t *x, uint16_t N) {
    // radix-2 DIT, in-place
    // 1) bit-reverse
    uint16_t bits = 0; while ((1u<<bits) < N) bits++;
    for (uint16_t i=0;i<N;i++) {
        uint16_t j = brev(i, bits);
        if (j > i) { cq15_t t = x[i]; x[i] = x[j]; x[j] = t; }
    }

    // 2) stages
    for (uint16_t len = 2; len <= N; len <<= 1) {
        uint16_t half = len >> 1;
        uint16_t step = tw_N / len; // twiddle step
        for (uint16_t k = 0; k < N; k += len) {
            uint16_t tw = 0;
            for (uint16_t j = 0; j < half; ++j, tw += step) {
                int16_t wr = tw_r[tw];
                int16_t wi = tw_i[tw];
                // butterfly: u = x[k+j], v = x[k+j+half] * W
                int16_t vr, vi;
                cplx_mul_q15(x[k+j+half].r, x[k+j+half].i, wr, wi, &vr, &vi);

                int32_t ur = x[k+j].r;
                int32_t ui = x[k+j].i;

                int32_t tr = ur + vr;
                int32_t ti = ui + vi;
                int32_t sr = ur - vr;
                int32_t si = ui - vi;

                // optional scaling to avoid overflow: shift down 1 bit per stage
                // This keeps headroom when many stages accumulate.
                x[k+j].r       = (int16_t)(tr >> 1);
                x[k+j].i       = (int16_t)(ti >> 1);
                x[k+j+half].r  = (int16_t)(sr >> 1);
                x[k+j+half].i  = (int16_t)(si >> 1);
            }
        }
    }
}

void hann_q15(cq15_t *x, uint16_t N) {
    for (uint16_t n=0; n<N; ++n) {
        // w[n] = 0.5*(1 - cos(2*pi*n/(N-1)))
        double w = 0.5 * (1.0 - cos(2.0 * M_PI * n / (double)(N-1)));
        int16_t q = (int16_t)lrint(w * 32767.0);
        x[n].r = q15_mul(x[n].r, q);
        x[n].i = q15_mul(x[n].i, q);
    }
}

void pack_real_q15(const int16_t *in, cq15_t *out, uint16_t N) {
    for (uint16_t k=0;k<N;k++){ out[k].r = in[k]; out[k].i = 0; }
}

