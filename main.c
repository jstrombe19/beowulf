#include "pico/stdlib.h"
#include "fft_q15.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef N_FFT
#define N_FFT 2048
#endif

#ifndef N_ITER
#define N_ITER 500   // choose 100..1000
#endif

#ifndef DO_HANN
#define DO_HANN 0    // set 1 to include Hann window cost
#endif

#ifndef INCLUDE_WINDOW_IN_TIMING
#define INCLUDE_WINDOW_IN_TIMING 0 // 1 to include window time in T_comp
#endif

static void gen_test_signal_q15(int16_t *dst, uint16_t N) {
    // Sum of a few tones, safely within Q15
    const double f1 = 123.0, f2 = 371.0, f3 = 777.0; // Hz (arbitrary)
    const double fs = 48000.0;                        // pretend sample rate (for synthesis only)
    for (uint16_t n=0;n<N;n++){
        double t = (double)n / fs;
        double s = 0.6*sin(2*M_PI*f1*t) + 0.3*sin(2*M_PI*f2*t) + 0.2*sin(2*M_PI*f3*t);
        int32_t q = (int32_t)lrint(s * 32767.0);
        if (q > 32767) q = 32767; if (q < -32768) q = -32768;
        dst[n] = (int16_t)q;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(500); // allow USB CDC to come up

    printf("RP2040 Q15 FFT benchmark N=%d, ITERS=%d\n", N_FFT, N_ITER);

    if (!fft_q15_init(N_FFT)) {
        printf("fft_q15_init failed (N must be power of two).\n");
        while (1) tight_loop_contents();
    }

    // Buffers
    static cq15_t x[N_FFT];
    static int16_t real_in[N_FFT];

    gen_test_signal_q15(real_in, N_FFT);
    pack_real_q15(real_in, x, N_FFT);

#if DO_HANN
    if (!INCLUDE_WINDOW_IN_TIMING) {
        hann_q15(x, N_FFT); // pre-apply window outside the timed region
    }
#endif

    // Warm-up
    for (int i=0;i<5;i++) { fft_q15(x, N_FFT); }

    // Timed iterations
    uint64_t tmin = UINT64_MAX, tmax = 0, tsum = 0;

    for (int it=0; it<N_ITER; ++it) {
        // refresh input each iteration (optional). Keep same signal here:
        pack_real_q15(real_in, x, N_FFT);

        uint64_t t0 = time_us_64();

#if DO_HANN && INCLUDE_WINDOW_IN_TIMING
        hann_q15(x, N_FFT);
#endif
        fft_q15(x, N_FFT);

        uint64_t t1 = time_us_64();
        uint64_t dt = t1 - t0;
        if (dt < tmin) tmin = dt;
        if (dt > tmax) tmax = dt;
        tsum += dt;
    }

    double avg_us = (double)tsum / (double)N_ITER;
    double ffts_per_sec = 1e6 / avg_us;

    printf("Results (microseconds): min=%.0f  avg=%.1f  max=%.0f  |  FFT/s=%.1f\n",
           (double)tmin, avg_us, (double)tmax, ffts_per_sec);

#if DO_HANN && INCLUDE_WINDOW_IN_TIMING
    printf("Includes Hann window time.\n");
#elif DO_HANN
    printf("Hann window not included in timing (applied outside loop).\n");
#else
    printf("No window applied.\n");
#endif

    // Optionally print a few magnitudes to verify output shape
    for (int k=0; k<8; ++k) {
        int32_t r = x[k].r, i = x[k].i;
        int32_t mag2 = r*r + i*i;
        printf("Bin %d: r=%ld i=%ld | mag2=%ld\n", k, (long)r, (long)i, (long)mag2);
    }

    while (1) { sleep_ms(1000); }
}

