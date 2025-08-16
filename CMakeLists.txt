cmake_minimum_required(VERSION 3.13)
include(./pico-sdk/external/pico_sdk_import.cmake)

project(pico_fft_bench C CXX ASM)
pico_sdk_init()

add_executable(fft_bench
    main.c
    fft_q15.c
)
target_link_libraries(fft_bench pico_stdlib)
pico_enable_stdio_usb(fft_bench 1)
pico_enable_stdio_uart(fft_bench 0)
pico_add_extra_outputs(fft_bench)

# Optional: change iterations and flags at build time
# e.g., cmake -DN_ITER=1000 -DDO_HANN=1 -DINCLUDE_WINDOW_IN_TIMING=1 ..
target_compile_definitions(fft_bench PRIVATE
    N_FFT=2048
    N_ITER=${N_ITER}
    DO_HANN=${DO_HANN}
    INCLUDE_WINDOW_IN_TIMING=${INCLUDE_WINDOW_IN_TIMING}
)

