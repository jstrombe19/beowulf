#pragma once
#include <stdint.h>
#include <stdbool.h>

// Q15 complex numbers: real and imag are int16_t
typedef struct { int16_t r, i; } cq15_t;

// Generate twiddle tables for N (power of two). Call once at boot.
bool fft_q15_init(uint16_t N);

// In-place radix-2 DIT FFT (N power of two). Data is interleaved complex.
void fft_q15(cq15_t *x, uint16_t N);

// Optional: apply Hann window (Q15) on real input packed as complex (imag=0).
// If your data is real-only, you can store it in x[k].r and set x[k].i=0.
void hann_q15(cq15_t *x, uint16_t N);

// Utility: pack a real array (Q15) into complex buffer (imag=0)
void pack_real_q15(const int16_t *in, cq15_t *out, uint16_t N);

