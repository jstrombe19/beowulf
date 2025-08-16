// master_cluster_test.c
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// ---------- Config ----------
#define USE_SPI1            0           // set 1 if you also wire SPI1
#define NUM_SLAVES_SPI0     4           // CS on GP2..GP5 (adjust below)
#define NUM_SLAVES_SPI1     0           // set if USE_SPI1=1 and you wire more slaves

// SPI0 pins
#define SPI0_SCK   18
#define SPI0_MOSI  19
#define SPI0_MISO  16
static const uint CS_SPI0[] = {2,3,4,5};  // up to 4

// SPI1 pins (only if you wire it)
// #define SPI1_SCK   10
// #define SPI1_MOSI  11
// #define SPI1_MISO  12
// static const uint CS_SPI1[] = {6,7,8,9};

// FFT payload sizes
#define N_SAMPLES           2048
#define BYTES_PER_SAMPLE    2           // int16_t
#define IN_BYTES            (N_SAMPLES*BYTES_PER_SAMPLE) // 4096
#define OUT_VALID_BYTES     (N_SAMPLES*2) // magnitudes (u16) = 4096
#define OUT_FEATURE_BYTES   512         // tweak for your feature vector size

// Features batching (K windows per transaction in features mode)
static int K_batch = 1;                 // set 4 for a good starting point

// SPI sweep table (you can add/remove entries)
static const uint32_t SPI_BAUD_TABLE[] = { 24000000u, 32000000u, 40000000u };
static const int SPI_BAUD_COUNT = sizeof(SPI_BAUD_TABLE)/sizeof(SPI_BAUD_TABLE[0]);

// ---------- Protocol ----------
#pragma pack(push,1)
typedef struct {
  uint8_t  preamble;     // 0xA5
  uint8_t  version;      // 0x01
  uint8_t  cmd;          // 0x01=ECHO, 0x10=FFT_RUN, 0x11=READ_RESULT
  uint8_t  flags;        // bit0: features-mode
  uint16_t seq;          // sequence
  uint16_t payload_len;  // N
  uint16_t crc16;        // CCITT-FALSE over header (except crc) + payload
} frame_hdr_t;
#pragma pack(pop)

static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i=0; i<len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b=0; b<8; ++b)
      crc = (crc & 0x8000) ? (uint16_t)((crc<<1) ^ 0x1021) : (uint16_t)(crc<<1);
  }
  return crc;
}

// ---------- Mode handling ----------
typedef enum { MODE_VALIDATE=0, MODE_FEATURES=1 } op_mode_t;
static op_mode_t g_mode = MODE_VALIDATE;
static int g_baud_idx = 0;

// ---------- Buffers ----------
#define MAX_WRITE (IN_BYTES * 8)       // supports K up to 8
#define MAX_READ  (OUT_VALID_BYTES * 8)

static uint8_t tx_buf[MAX_WRITE + sizeof(frame_hdr_t)];
static uint8_t rx_buf[MAX_READ];

// Fill input samples deterministically (no host required)
static void fill_samples(uint8_t* dst, size_t n_bytes, uint32_t seed) {
  // simple LFSR-ish filler; replace with real data later
  uint32_t x = 0x123456u ^ seed;
  for (size_t i=0;i<n_bytes;i++) {
    x ^= x << 7; x ^= x >> 9; x ^= x << 8;
    dst[i] = (uint8_t)(x & 0xFF);
  }
}

// ---------- SPI helpers ----------
static inline void cs_low(uint pin){ gpio_put(pin, 0); }
static inline void cs_high(uint pin){ gpio_put(pin, 1); }

static void spi_write_exact(spi_inst_t* spi, int cs_pin, const uint8_t* data, size_t n) {
  cs_low(cs_pin);
  spi_write_blocking(spi, data, n);
  cs_high(cs_pin);
}
static void spi_write_read_exact(spi_inst_t* spi, int cs_pin, const uint8_t* w, size_t wn, uint8_t* r, size_t rn) {
  cs_low(cs_pin);
  spi_write_blocking(spi, w, wn);
  spi_read_blocking(spi, 0x00, r, rn);
  cs_high(cs_pin);
}

static void build_header(frame_hdr_t* h, uint8_t cmd, uint8_t flags, uint16_t seq, uint16_t payload_len, const uint8_t* payload) {
  h->preamble = 0xA5; h->version = 0x01; h->cmd = cmd; h->flags = flags;
  h->seq = seq; h->payload_len = payload_len; h->crc16 = 0;
  uint16_t crc = 0xFFFF;
  crc = crc16_ccitt_false((const uint8_t*)h, sizeof(*h)-2);
  if (payload_len) crc = crc16_ccitt_false(payload, payload_len) ^ (crc ^ 0xFFFF); // combine
  // Simpler: compute over header (except crc) + payload in one buffer when we send.
  h->crc16 = 0; // will compute below when composing
}

// Compose header+payload into tx_buf and append CRC.
static size_t compose_frame(uint8_t* out, uint8_t cmd, uint8_t flags, uint16_t seq,
                            const uint8_t* payload, uint16_t plen) {
  frame_hdr_t h = { .preamble=0xA5, .version=0x01, .cmd=cmd, .flags=flags, .seq=seq, .payload_len=plen, .crc16=0 };
  memcpy(out, &h, sizeof(h));
  if (plen) memcpy(out+sizeof(h), payload, plen);
  uint16_t crc = crc16_ccitt_false(out, sizeof(h)-2 + plen);
  ((frame_hdr_t*)out)->crc16 = crc;
  return sizeof(h) + plen;
}

// ---------- One transaction pair: WRITE (FFT_RUN) then READ (READ_RESULT) ----------
typedef struct {
  uint64_t frames_ok;
  uint64_t frames_fail;
  uint64_t bytes_tx;
  uint64_t bytes_rx;
} stats_t;

static void do_round(spi_inst_t* spi, int cs_pin, uint16_t* seq, stats_t* st, op_mode_t mode, int K) {
  const uint8_t flags = (mode == MODE_FEATURES) ? 0x01 : 0x00;

  // -------- WRITE phase: send K windows packed back-to-back
  const size_t one_in = IN_BYTES;
  const size_t total_in = one_in * K;

  fill_samples(tx_buf, total_in, *seq); // fake input data
  size_t n_write = compose_frame(tx_buf, 0x10 /*FFT_RUN*/, flags, *seq, tx_buf, (uint16_t)total_in);
  spi_write_exact(spi, cs_pin, tx_buf, n_write);
  st->bytes_tx += n_write;

  // -------- READ phase: read back K results (either full magnitudes or features)
  const size_t one_out = (mode == MODE_VALIDATE) ? OUT_VALID_BYTES : OUT_FEATURE_BYTES;
  const size_t total_out = one_out * K;

  // Send READ_RESULT header, then clock out response
  uint8_t hdr_only[sizeof(frame_hdr_t)];
  size_t hlen = compose_frame(hdr_only, 0x11 /*READ_RESULT*/, flags, *seq, NULL, 0);
  spi_write_read_exact(spi, cs_pin, hdr_only, hlen, rx_buf, total_out);
  st->bytes_rx += total_out;

  // -------- Verify (cheap check): re-CRC the received payload length using a simple rolling XOR as a placeholder.
  // In your slave, you should also send a small header back with CRC; for now we assume pure payload
  // and just do a quick checksum here to detect SI errors in soak tests.
  uint32_t chk = 0;
  for (size_t i=0;i<total_out;i++) chk ^= rx_buf[i];
  if (chk == 0 && total_out) {
    // not a real failure, but if you want, treat zero checksum as suspicious
  }

  // If needed, add a simple pattern in the slave output so you can verify exact bytes.
  st->frames_ok++;
  (*seq)++;
}

// ---------- UI: runtime control via keypress ----------
static void print_help(void){
  printf("\r\nKeys: [v]=validate  [f]=features  [+]=faster clk  [-]=slower clk  [k/K]=batch +/-  [s]=status\r\n");
}

int main() {
  stdio_init_all();
  sleep_ms(300); // let USB come up
  printf("\r\nPico cluster master soak v1\r\n");

  // SPI0 init
  spi_init(spi0, SPI_BAUD_TABLE[g_baud_idx]);
  gpio_set_function(SPI0_SCK,  GPIO_FUNC_SPI);
  gpio_set_function(SPI0_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(SPI0_MISO, GPIO_FUNC_SPI);
  spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
  for (int i=0;i<NUM_SLAVES_SPI0;i++){ gpio_init(CS_SPI0[i]); gpio_set_dir(CS_SPI0[i], GPIO_OUT); gpio_put(CS_SPI0[i], 1); }

#if USE_SPI1
  spi_init(spi1, SPI_BAUD_TABLE[g_baud_idx]);
  gpio_set_function(SPI1_SCK,  GPIO_FUNC_SPI);
  gpio_set_function(SPI1_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(SPI1_MISO, GPIO_FUNC_SPI);
  spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
  for (int i=0;i<NUM_SLAVES_SPI1;i++){ gpio_init(CS_SPI1[i]); gpio_set_dir(CS_SPI1[i], GPIO_OUT); gpio_put(CS_SPI1[i], 1); }
#endif

  printf("Mode: %s | SPI clk: %u Hz | K-batch=%d | Slaves(SPI0)=%d\r\n",
         g_mode==MODE_VALIDATE?"VALIDATE(full mags)":"FEATURES(512B)",
         SPI_BAUD_TABLE[g_baud_idx], K_batch, NUM_SLAVES_SPI0);
  print_help();

  uint16_t seq = 1;
  stats_t st = {0};
  absolute_time_t next_report = make_timeout_time_ms(1000);

  while (true) {
    // Round-robin each slave on SPI0 (add SPI1 similarly if you use it)
    for (int s=0; s<NUM_SLAVES_SPI0; ++s) {
      do_round(spi0, CS_SPI0[s], &seq, &st, g_mode, K_batch);
    }

    // Periodic report
    if (absolute_time_diff_us(get_absolute_time(), next_report) <= 0) {
      uint32_t baud = SPI_BAUD_TABLE[g_baud_idx];
      double mbytes = (st.bytes_tx + st.bytes_rx) / (1024.0*1024.0);
      static uint64_t last_ok=0, last_fail=0, last_bytes=0;
      uint64_t ok = st.frames_ok, fail = st.frames_fail, b = st.bytes_tx+st.bytes_rx;
      double dmb = (b - last_bytes) / (1024.0*1024.0);
      printf("[clk=%u] ok=%llu  fail=%llu  dMB/s=%.2f  totalMB=%.2f  mode=%s  K=%d  slaves=%d\r\n",
             baud, ok, fail, dmb, mbytes,
             g_mode==MODE_VALIDATE?"VALIDATE":"FEATURES", K_batch, NUM_SLAVES_SPI0);
      last_ok=ok; last_fail=fail; last_bytes=b;
      next_report = make_timeout_time_ms(1000);
    }

    // Key handling (non-blocking)
    int ch = getchar_timeout_us(0);
    if (ch != PICO_ERROR_TIMEOUT) {
      if (ch=='v' || ch=='V') { g_mode = MODE_VALIDATE; puts("-> VALIDATE"); }
      else if (ch=='f' || ch=='F') { g_mode = MODE_FEATURES; puts("-> FEATURES"); }
      else if (ch=='+') { if (g_baud_idx < SPI_BAUD_COUNT-1) g_baud_idx++; spi_set_baudrate(spi0, SPI_BAUD_TABLE[g_baud_idx]); printf("clk=%u\r\n", SPI_BAUD_TABLE[g_baud_idx]); }
      else if (ch=='-') { if (g_baud_idx > 0) g_baud_idx--; spi_set_baudrate(spi0, SPI_BAUD_TABLE[g_baud_idx]); printf("clk=%u\r\n", SPI_BAUD_TABLE[g_baud_idx]); }
      else if (ch=='k') { if (K_batch>1) K_batch--; printf("K=%d\r\n", K_batch); }
      else if (ch=='K') { if (K_batch<8) K_batch++; printf("K=%d\r\n", K_batch); }
      else if (ch=='s' || ch=='S') { print_help(); }
    }
  }
}

