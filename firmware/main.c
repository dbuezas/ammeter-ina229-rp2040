// INA229 bench ammeter -- RP2040-Zero, SPI0.
//
// Streams current and voltage over USB as CSV rows, or every ADC conversion as binary
// packets. Each row also carries the INA229's 40-bit CHARGE and ENERGY accumulators. The
// chip updates these itself on every conversion, so totals taken from them are exact even
// when the host misses rows. Use the accumulators for totals and the sampled values for
// the shape of the current.
//
// Wiring:
//     INA229 CLK -> GP2 (SPI0 SCK)      INA229 SI  -> GP3 (SPI0 TX / MOSI)
//     INA229 SO  -> GP4 (SPI0 RX/MISO)  INA229 CS  -> GP5 (plain GPIO)
//     INA229 ALT -> GP6 (input; edges are only counted, as a wiring check)
// The shunt goes in the supply rail of the device under test, IN+ towards the supply.
// VBUS measures that rail against GND. VCC/GND power the INA229 from the RP2040.
//
// CURRENT_LSB is set to the shunt ADC's full scale divided by 2^19. With that choice,
// SHUNT_CAL is 4096 for every shunt and both ranges, and the CURRENT register covers
// exactly the ADC's range. The only value the firmware needs is the shunt resistance.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

// ---------------------------------------------------------------- wiring
#define PIN_SCK   2
#define PIN_MOSI  3
#define PIN_MISO  4
#define PIN_CS    5
#define PIN_ALERT 6
#define SPI_PORT  spi0
#define SPI_HZ    10000000     // the part's maximum

// ---------------------------------------------------------------- INA229 registers
#define R_CONFIG      0x00     // 16b  RST | RSTACC | CONVDLY[7:0] | TEMPCOMP | ADCRANGE
#define R_ADC_CONFIG  0x01     // 16b  MODE[3:0] | VBUSCT[2:0] | VSHCT[2:0] | VTCT[2:0] | AVG[2:0]
#define R_SHUNT_CAL   0x02     // 16b  15-bit calibration word
#define R_VSHUNT      0x04     // 24b  signed 20-bit in [23:4]
#define R_VBUS        0x05     // 24b  unsigned 20-bit in [23:4]
#define R_DIETEMP     0x06     // 16b  signed, 7.8125 m degC/LSB
#define R_CURRENT     0x07     // 24b  signed 20-bit in [23:4], CURRENT_LSB per count
#define R_POWER       0x08     // 24b  unsigned
#define R_ENERGY      0x09     // 40b  unsigned
#define R_CHARGE      0x0A     // 40b  signed
#define R_DIAG_ALRT   0x0B     // 16b  latch/alert config + the overflow sticky bits
#define R_MFG_ID      0x3E     // 16b  reads 0x5449 ("TI")
#define R_DEVICE_ID   0x3F     // 16b  reads 0x229x

#define MFG_ID_TI     0x5449
#define DEVICE_ID_229 0x229    // top 12 bits of R_DEVICE_ID; low 4 are the die revision

// DIAG_ALRT sticky bits reported on every row
#define D_ENERGYOF (1u << 11)  // the 40-bit energy accumulator wrapped
#define D_CHARGEOF (1u << 10)  // the 40-bit charge accumulator wrapped
#define D_MATHOF   (1u <<  9)  // current/power math overflowed -- the scaling is wrong

// A shunt reading this close to the 20-bit limit means the ADC clipped. Clipping gives no
// error: the CHARGE accumulator adds up the clipped value, so totals come out too low.
// Each row counts the clipped samples so this can be seen. Only sampled conversions are
// counted, so a very short spike can still clip unseen.
#define SAT_COUNTS 520000   // 2^19 is 524288

// Conversion times the ADC_CONFIG fields can select, in microseconds, index = field value.
static const uint16_t CT_US[8] = {50, 84, 150, 280, 540, 1052, 2074, 4120};
static const uint16_t AVG_N[8] = {1, 4, 16, 64, 128, 256, 512, 1024};

// ---------------------------------------------------------------- SPI primitives
// A transaction is one command byte -- the 6-bit register address, then a zero bit,
// then read(1)/write(0) -- followed by the register's bytes, MSB first. Mode 1.

static inline void cs_lo(void) { gpio_put(PIN_CS, 0); }
static inline void cs_hi(void) { gpio_put(PIN_CS, 1); }

// Read a register into a buffer as raw bytes. The binary stream sends samples exactly as
// the chip gives them; the host scales them with current_lsb from the header.
static void reg_read_raw(uint8_t reg, uint8_t *dst, int nbytes) {
    uint8_t cmd = (uint8_t)((reg << 2) | 1);
    uint8_t tx[5] = {0};
    cs_lo();
    spi_write_blocking(SPI_PORT, &cmd, 1);
    spi_write_read_blocking(SPI_PORT, tx, dst, nbytes);
    cs_hi();
}

static uint64_t reg_read(uint8_t reg, int nbytes) {
    uint8_t cmd = (uint8_t)((reg << 2) | 1);
    uint8_t rx[5] = {0};
    uint8_t tx[5] = {0};
    cs_lo();
    spi_write_blocking(SPI_PORT, &cmd, 1);
    spi_write_read_blocking(SPI_PORT, tx, rx, nbytes);
    cs_hi();
    uint64_t v = 0;
    for (int i = 0; i < nbytes; i++) v = (v << 8) | rx[i];
    return v;
}

static void reg_write(uint8_t reg, uint16_t value) {
    uint8_t buf[3] = {(uint8_t)(reg << 2), (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    cs_lo();
    spi_write_blocking(SPI_PORT, buf, 3);
    cs_hi();
}

// 20-bit signed payload sitting in bits [23:4] of a 24-bit register.
static int32_t s20_of(uint64_t raw24) {
    return (int32_t)((uint32_t)raw24 << 8) >> 12;
}
static int64_t s40_of(uint64_t raw40) {
    return (int64_t)(raw40 << 24) >> 24;
}

// ---------------------------------------------------------------- configuration
// The firmware cannot measure the shunt, so the host must tell it. A wrong value gives
// wrong numbers with no error. The host tools always set it; this default (1 ohm, coarse
// range) is only used when nothing sets it.
static double  cfg_shunt_ohm = 1.0;
static int     cfg_range     = 0;     // 1 = +/-40.96 mV (fine), 0 = +/-163.84 mV (coarse)
static int     cfg_avg_i     = 0;     // index into AVG_N
static int     cfg_vbusct_i  = 2;     // 150 us
// Shunt conversion time: 280 us. Shorter conversions give fewer usable ADC codes. I
// measured the gap between neighbouring codes on an idle supply (the uA column is for
// 1 ohm on the coarse range, where one code is 312.5 nA):
//
//     VSHCT   conv/s   code spacing   effective resolution
//      4120      243        4 counts      1.25 uA
//      1052      951        4 counts      1.25 uA
//       280     3571        4 counts      1.25 uA   <- fastest at full resolution
//        84    11905       76 counts        24 uA
//        50    20000      300 counts        94 uA
//
// At 50 us, 93 % of idle samples land on the same code. Use 280 us or slower for
// microamp work.
static int     cfg_vshct_i   = 3;     // 280 us
static int     cfg_vtct_i    = 0;     // 50 us  -- temperature is not important here
static uint16_t cfg_cal      = 4096;  // see the header: 4096 for every shunt and range
static double  cfg_rate_hz   = 100.0; // rows per second
// Shunt-only mode (ADC_CONFIG MODE 0xA) instead of bus + shunt + temperature (0xF).
// In the full sequence, one cycle takes vbusct + vshct + vtct, so the shunt is sampled
// less often. Shunt-only samples the shunt every vshct: 20 kHz at 50 us, against 800 Hz
// for the default sequence. VBUS and DIETEMP then keep their last value, so do not use
// this mode when you need to watch the bus voltage.
static int     cfg_shunt_only = 0;

// ---------------------------------------------------------------- binary sample stream
//
// Every conversion, sent raw, with no averaging. At 3.5 kHz this is about 10 kB/s, well
// within what USB carries. Samples are grouped into packets so each USB write carries
// many of them.
//
//   magic 0xA5 0x5A | seq u16 | count u16 | t_us u32 | q_raw 5B | samples 3B*count | crc16
//
// t_us and q_raw are read at the first sample of the packet. q_raw is the chip's charge
// accumulator. The host compares the charge change between packets with the sum of the
// samples it got; if the loop missed conversions, the two do not match. A gap in seq
// shows a lost packet.
#define BIN_MAGIC0 0xA5
#define BIN_MAGIC1 0x5A
#define BIN_BATCH  128
#define BIN_HDR    15
static int      cfg_bin = 0;
// Which register the binary stream reads. CURRENT is calculated by the chip from VSHUNT,
// so it can only update as fast as that calculation. VSHUNT is the raw ADC output. Both
// use the same scale factor on the host.
static uint8_t  cfg_src = R_CURRENT;
static uint8_t  bin_buf[BIN_HDR + 3 * BIN_BATCH + 2];
static uint16_t bin_n = 0, bin_seq = 0;

static uint16_t crc16_ccitt(const uint8_t *p, int n) {
    uint16_t c = 0xFFFF;
    for (int i = 0; i < n; i++) {
        c ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; b++)
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}

static void bin_flush(void) {
    if (!bin_n) return;
    bin_buf[4] = (uint8_t)(bin_n & 0xFF);
    bin_buf[5] = (uint8_t)(bin_n >> 8);
    int len = BIN_HDR + 3 * bin_n;
    uint16_t c = crc16_ccitt(bin_buf, len);
    bin_buf[len] = (uint8_t)(c & 0xFF);
    bin_buf[len + 1] = (uint8_t)(c >> 8);
    stdio_put_string((const char *)bin_buf, len + 2, false, false);
    bin_seq++;
    bin_n = 0;
}
// Flips the sign of the reported values, for when IN+ and IN- are swapped. It changes
// only the CSV output; the registers and the binary stream stay as the chip reports them.
// To check the wiring, add a known load: the reading must go up.
static int     cfg_invert    = 0;
// The reading at zero current, in microamps (shunt shorted, load removed). It is
// subtracted from the calculated values (a row's i_mean, and charge and mean current in
// `stat`), never from the raw columns. It is an offset of the chip, so `invert` flips its
// sign along with the rest.
//
// It depends on the shunt. This value was measured on a 0.2 ohm shunt; on 1 ohm the real
// offset is about 0.08 uA, so this over-corrects by about 0.3 uA. Measure it again for
// your shunt and set it with `set offset`.
static double  cfg_offset_ua = -0.40;
// Compact rows: microsecond timestamp, raw charge count, summed current, sample count,
// peak and clip count. They are short enough to send every few milliseconds, which is
// what you need to see short events. The charge count is absolute, so the charge of any
// event is the difference of two rows.
static int     cfg_compact   = 0;

static double current_lsb;            // amps per CURRENT count
static uint32_t conv_period_us;       // one full bus+shunt+temp cycle, averaging included

static void recompute_scale(void) {
    double v_fs = cfg_range ? 0.04096 : 0.16384;      // shunt ADC full scale, volts
    current_lsb = v_fs / (cfg_shunt_ohm * 524288.0);  // 2^19 counts to full scale
    conv_period_us = (uint32_t)(cfg_shunt_only
                        ? CT_US[cfg_vshct_i]
                        : CT_US[cfg_vbusct_i] + CT_US[cfg_vshct_i] + CT_US[cfg_vtct_i])
                   * AVG_N[cfg_avg_i];
}

static void apply_config(void) {
    recompute_scale();
    reg_write(R_CONFIG, (uint16_t)(cfg_range ? (1u << 4) : 0));
    uint16_t adc = (uint16_t)(((cfg_shunt_only ? 0xA : 0xF) << 12)   // shunt only / all
                            | (cfg_vbusct_i << 9)
                            | (cfg_vshct_i  << 6)
                            | (cfg_vtct_i   << 3)
                            | (cfg_avg_i));
    reg_write(R_ADC_CONFIG, adc);
    reg_write(R_SHUNT_CAL, cfg_cal);
    reg_write(R_DIAG_ALRT, (uint16_t)(1u << 14));      // CNVR: pulse ALERT when a conversion lands
}

// The previous row's charge and time, for the row-to-row difference. Cleared together
// with the chip's accumulators, or the first row after a reset would be wrong.
static int64_t  prev_q_raw  = 0;
static uint64_t prev_row_us = 0;

// Min/max of the samples since the last row. Filled by the sample loop, reset by each row.
static double   i_min = 0, i_max = 0;
static uint32_t i_n = 0;
static uint32_t i_sat = 0;   // clipped samples -- see SAT_COUNTS
// Sum of the raw CURRENT counts sampled in this row. Sent as sum and count, so the host
// does the division.
//
// This gives a finer reading than the charge difference. One charge count is one
// CURRENT_LSB in coulombs (312.5 nC at 1 ohm, coarse), so over a 5 ms row one count is
// already 62.5 uA. The CURRENT register resolves 312.5 nA per sample.
static int64_t  i_raw_sum = 0;

static void accum_zero(void) {
    reg_write(R_CONFIG, (uint16_t)((1u << 14) | (cfg_range ? (1u << 4) : 0)));  // RSTACC
    reg_write(R_DIAG_ALRT, (uint16_t)(1u << 14));                               // clears sticky bits
    prev_q_raw  = 0;
    prev_row_us = time_us_64();
}

// ---------------------------------------------------------------- ALERT pin
// ALERT is open-drain and active low, held up by the internal pull-up. It cannot be used
// to time the sampling: in continuous mode a conversion is always ready, so the chip
// keeps it asserted. The edges are only counted as a wiring check.
static volatile uint32_t alert_edges = 0;
static void alert_isr(uint gpio, uint32_t events) { (void)gpio; (void)events; alert_edges++; }

// ---------------------------------------------------------------- reporting
static bool     streaming = false;
static uint64_t t_zero_us = 0;
static uint32_t rows_emitted = 0;   // rows the firmware sent. Compare with the rows the
                                    // host got, to tell a lost row from an unsent one.

static void print_header(void) {
    printf("# ina229 rp2040 ammeter\n");
    printf("# shunt=%.6f ohm range=%d (%s) avg=%u vbusct=%uus vshct=%uus vtct=%uus invert=%d\n",
           cfg_shunt_ohm, cfg_range, cfg_range ? "+/-40.96mV" : "+/-163.84mV",
           AVG_N[cfg_avg_i], CT_US[cfg_vbusct_i], CT_US[cfg_vshct_i], CT_US[cfg_vtct_i],
           cfg_invert);
    printf("# current_lsb=%.9f A  shunt_cal=%u  conv_period=%u us (%.0f Hz)  rate=%.2f Hz"
           "  shuntonly=%d\n",
           current_lsb, cfg_cal, conv_period_us, 1e6 / (double)conv_period_us,
           cfg_rate_hz, cfg_shunt_only);
    printf("# energy_J = e_raw * 51.2 * current_lsb   charge_C = q_raw * current_lsb\n");
    printf("# zero offset = %.3f uA, subtracted from i_mean and from stat; raw columns are raw\n",
           cfg_offset_ua);
    printf("# columns: S,t_ms,dt_ms,i_mean_A,q_raw,e_raw,vbus_V,i_min_A,i_max_A,n,sat,tempC,alrt,flags\n");
    printf("# sat > 0 means the shunt ADC clipped -- the totals are UNDERSTATED, use a smaller shunt\n");
    if (cfg_compact)
        printf("# COMPACT: Q,t_us,q_raw,i_sum,n,imax_raw,sat"
               "  (current = i_sum/n * current_lsb; charge = q_raw * current_lsb)\n");
}

static void print_stat(void) {
    uint64_t e_raw = reg_read(R_ENERGY, 5);
    int64_t  q_raw = s40_of(reg_read(R_CHARGE, 5));
    uint16_t diag  = (uint16_t)reg_read(R_DIAG_ALRT, 2);
    double   secs  = (double)(time_us_64() - t_zero_us) / 1e6;
    // The offset is a current, so its effect on the charge grows with the elapsed time.
    double   q_c   = (double)q_raw * current_lsb * (cfg_invert ? -1.0 : 1.0)
                     - cfg_offset_ua * 1e-6 * secs;
    double   e_j   = (double)e_raw * 16.0 * 3.2 * current_lsb;
    // Energy is calculated as charge x bus voltage, not read from the ENERGY register.
    // The chip adds up POWER, which is always positive, so near zero current the noise
    // does not cancel and the register reads far too high (I measured 1.4 mJ against
    // 0.015 mJ, 91x). The register value is still printed: above a few hundred microamps
    // it is fine, and a big gap between the two shows the current is down in the noise.
    uint32_t vbus_raw = (uint32_t)(reg_read(R_VBUS, 3) >> 4) & 0xFFFFF;
    double   vbus     = (double)vbus_raw * 195.3125e-6;
    double   e_qv     = q_c * vbus;
    printf("# stat elapsed=%.3f s  charge=%.6f C (%.6f mAh)  energy=%.6f J (%.6f mWh)"
           "  mean_current=%.9f A  vbus=%.4f V  energy_reg=%.6f J  rows=%lu  flags=%04x\n",
           secs, q_c, q_c / 3.6, e_qv, e_qv / 3.6, secs > 0 ? q_c / secs : 0.0,
           vbus, e_j, (unsigned long)rows_emitted, diag);
}

// ---------------------------------------------------------------- commands
static bool arg_index(const char *what, int want, const uint16_t *table, int *out) {
    for (int i = 0; i < 8; i++) if (table[i] == want) { *out = i; return true; }
    printf("# ERR %s: allowed", what);
    for (int i = 0; i < 8; i++) printf(" %u", table[i]);
    printf("\n");
    return false;
}

static void do_id(void) {
    uint16_t mfg = (uint16_t)reg_read(R_MFG_ID, 2);
    uint16_t dev = (uint16_t)reg_read(R_DEVICE_ID, 2);
    bool ok = (mfg == MFG_ID_TI) && ((dev >> 4) == DEVICE_ID_229);
    printf("# id mfg=%04x device=%04x rev=%u -> %s\n", mfg, dev, dev & 0xF,
           ok ? "INA229 present, SPI wiring good"
              : "NOT an INA229 reply -- check CS/SCK/SI/SO and that VCC is 3V3");
}

// One CSV row: read the meter, print it, and reset the min/max window.
//
// The row's current is the charge difference since the last row divided by the time, not
// the mean of the sampled values. The chip counts every conversion, so this is exact even
// when the loop was busy. If a row is lost, the next row still gives the right average
// over the whole gap. The sampled min/max columns show short peaks.
static void emit_row(void) {
    uint64_t now      = time_us_64();
    uint32_t vbus_raw = (uint32_t)(reg_read(R_VBUS, 3) >> 4) & 0xFFFFF;
    int16_t  t_raw    = (int16_t)reg_read(R_DIETEMP, 2);
    uint64_t e_raw    = reg_read(R_ENERGY, 5);
    int64_t  q_raw    = s40_of(reg_read(R_CHARGE, 5));
    uint16_t diag     = (uint16_t)reg_read(R_DIAG_ALRT, 2);

    uint64_t dt_us = now - prev_row_us;
    double   i_avg = dt_us ? (double)(q_raw - prev_q_raw) * current_lsb
                             / ((double)dt_us / 1e6)
                           : 0.0;
    prev_q_raw  = q_raw;
    prev_row_us = now;

    if (cfg_invert) {
        i_avg = -i_avg;
        q_raw = -q_raw;
        double t = i_min; i_min = -i_max; i_max = -t;
    }
    i_avg -= cfg_offset_ua * 1e-6;   // calculated values only; q_raw below stays raw

    // The accumulators are sent as raw integer counts. A float would lose the small
    // digits once the total gets large. The header gives current_lsb for scaling.
    //
    // The row is formatted first and sent as one string. printf() sends one character at
    // a time, and when a row was split over USB packets that way, its first 64 bytes got
    // lost.
    //
    // static: the main stack is only 2 KB.
    static char row[192];
    if (cfg_compact) {
        // t_us, charge accumulator, summed CURRENT counts, how many were summed, peak, sat
        snprintf(row, sizeof(row), "Q,%llu,%lld,%lld,%lu,%ld,%lu",
                 (unsigned long long)(now - t_zero_us),
                 (long long)q_raw,
                 (long long)i_raw_sum,
                 (unsigned long)i_n,
                 (long)(i_max / current_lsb),
                 (unsigned long)i_sat);
        puts(row);
        rows_emitted++;
        i_n = 0;
        i_sat = 0;
        i_raw_sum = 0;
        return;
    }
    snprintf(row, sizeof(row),
             "S,%llu,%llu,%.9f,%lld,%llu,%.6f,%.9f,%.9f,%lu,%lu,%.3f,%lu,%04x",
             (unsigned long long)((now - t_zero_us) / 1000),
             (unsigned long long)(dt_us / 1000),
             i_avg,
             (long long)q_raw,
             (unsigned long long)e_raw,
             (double)vbus_raw * 195.3125e-6,
             i_min, i_max, (unsigned long)i_n, (unsigned long)i_sat,
             (double)t_raw * 0.0078125,
             (unsigned long)alert_edges,
             diag & (D_ENERGYOF | D_CHARGEOF | D_MATHOF));
    puts(row);
    rows_emitted++;
    i_n = 0;
    i_sat = 0;
}

static void handle(char *line) {
    while (*line == ' ') line++;
    if (!*line) return;

    if (!strcmp(line, "help")) {
        printf("# id | cfg | start [keep] | stop | zero | stat | mark <text> | reg <hex>\n"
               "# set shunt <ohms> | set range <0|1> | set avg <n> | set vshct <us>\n"
               "# set vbusct <us> | set vtct <us> | set rate <Hz> | set cal <n>\n");
    } else if (!strcmp(line, "id")) {
        do_id();
    } else if (!strcmp(line, "alerttest")) {
        // Is the chip holding ALERT low, or is the pull-up not working? With all alerts
        // off, the pin must go high. If it stays low with the pull-up on, check the
        // wiring.
        for (int phase = 0; phase < 3; phase++) {
            const char *what;
            if (phase == 0) { reg_write(R_DIAG_ALRT, 0x0000); what = "all alerts OFF"; }
            else if (phase == 1) { gpio_pull_up(PIN_ALERT); what = "alerts OFF, pull-up on"; }
            else { reg_write(R_DIAG_ALRT, (uint16_t)(1u << 14)); what = "CNVR alert ON"; }
            sleep_ms(20);
            uint32_t lo = 0, tot = 0;
            uint64_t end = time_us_64() + 50000;
            while (time_us_64() < end) { if (!gpio_get(PIN_ALERT)) lo++; tot++; }
            printf("# alerttest %-24s pin low %.1f%%  DIAG_ALRT=%04x\n",
                   what, tot ? 100.0 * lo / tot : 0.0,
                   (unsigned)reg_read(R_DIAG_ALRT, 2));
        }
    } else if (!strcmp(line, "alertscan")) {
        // Sample the ALERT pin for 200 ms to see if it pulses once per conversion, is
        // stuck, or rises too slowly. Only the weak internal pull-up holds it high, so with
        // long jumper wires it can read low more often than it really is.
        uint32_t lo = 0, tot = 0, fall = 0;
        int prev = 1;
        uint64_t end = time_us_64() + 200000;   // 200 ms, no SPI traffic during it
        while (time_us_64() < end) {
            int v = gpio_get(PIN_ALERT);
            if (v == 0) lo++;
            if (prev == 1 && v == 0) fall++;
            prev = v;
            tot++;
        }
        printf("# alertscan 200 ms: %lu samples, low %.1f%%, falling edges %lu"
               " (%.0f/s)  conv_period=%u us -> expect %.0f/s\n",
               (unsigned long)tot, tot ? 100.0 * lo / tot : 0.0,
               (unsigned long)fall, fall * 5.0, conv_period_us,
               1e6 / (double)conv_period_us);
    } else if (!strcmp(line, "selftest")) {
        // Numbered lines in two sizes, so the host can tell which ones went missing and
        // whether the line length matters.
        static char pad[200];
        for (int i = 0; i < 40; i++) {
            memset(pad, 'a' + (i % 26), 50);
            pad[50] = 0;
            printf("T,%02d,short,%s\n", i, pad);
        }
        for (int i = 0; i < 10; i++) {
            memset(pad, 'A' + (i % 26), 180);
            pad[180] = 0;
            printf("T,%02d,long,%s\n", i, pad);
        }
        printf("T,done\n");
    } else if (!strcmp(line, "clk")) {
        // USB needs exactly 48 MHz. If the board's crystal is not the 12 MHz the SDK
        // expects, USB still connects but loses packets under steady traffic.
        printf("# clk sys=%lu ref=%lu usb=%lu peri=%lu xosc_assumed=%d MHz\n",
               (unsigned long)clock_get_hz(clk_sys),
               (unsigned long)clock_get_hz(clk_ref),
               (unsigned long)clock_get_hz(clk_usb),
               (unsigned long)clock_get_hz(clk_peri),
               XOSC_MHZ);
        printf("# clk_usb must read 48000000 exactly\n");
    } else if (!strcmp(line, "cfg")) {
        print_header();
    } else if (!strncmp(line, "start", 5)) {
        if (!strstr(line, "keep")) { accum_zero(); t_zero_us = time_us_64(); }
        print_header();
        streaming = true;
    } else if (!strcmp(line, "stop")) {
        streaming = false;
        if (cfg_bin) bin_flush();
        print_stat();
    } else if (!strcmp(line, "zero")) {
        accum_zero();
        t_zero_us = time_us_64();
        printf("# zeroed\n");
    } else if (!strcmp(line, "r")) {
        // Send one row on request. The host asks and then waits, so no row is sent while
        // nobody reads, and a lost reply can be asked for again. Free-running streaming
        // loses about half its rows on this setup.
        emit_row();
    } else if (!strcmp(line, "stat")) {
        print_stat();
    } else if (!strncmp(line, "mark ", 5)) {
        printf("M,%llu,%s\n", (unsigned long long)((time_us_64() - t_zero_us) / 1000), line + 5);
    } else if (!strncmp(line, "reg ", 4)) {
        int r = (int)strtol(line + 4, NULL, 16);
        int n = (r == R_ENERGY || r == R_CHARGE) ? 5
              : (r >= R_VSHUNT && r <= R_POWER && r != R_DIETEMP) ? 3 : 2;
        printf("# reg %02x = %010llx (%d bytes)\n", r,
               (unsigned long long)reg_read((uint8_t)r, n), n);
    } else if (!strncmp(line, "set ", 4)) {
        char *k = line + 4;
        char *v = strchr(k, ' ');
        if (!v) { printf("# ERR set needs a value\n"); return; }
        *v++ = 0;
        if (!strcmp(k, "shunt")) {
            double d = atof(v);
            if (d <= 0) { printf("# ERR shunt must be > 0\n"); return; }
            cfg_shunt_ohm = d;
        } else if (!strcmp(k, "range")) {
            cfg_range = atoi(v) ? 1 : 0;
        } else if (!strcmp(k, "avg")) {
            if (!arg_index("avg", atoi(v), AVG_N, &cfg_avg_i)) return;
        } else if (!strcmp(k, "vshct")) {
            if (!arg_index("vshct", atoi(v), CT_US, &cfg_vshct_i)) return;
        } else if (!strcmp(k, "vbusct")) {
            if (!arg_index("vbusct", atoi(v), CT_US, &cfg_vbusct_i)) return;
        } else if (!strcmp(k, "vtct")) {
            if (!arg_index("vtct", atoi(v), CT_US, &cfg_vtct_i)) return;
        } else if (!strcmp(k, "rate")) {
            double d = atof(v);
            if (d <= 0 || d > 2000) { printf("# ERR rate must be 0<Hz<=2000\n"); return; }
            cfg_rate_hz = d;
        } else if (!strcmp(k, "shuntonly")) {
            cfg_shunt_only = atoi(v) ? 1 : 0;
        } else if (!strcmp(k, "src")) {
            cfg_src = (atoi(v) == 4) ? R_VSHUNT : R_CURRENT;
        } else if (!strcmp(k, "bin")) {
            cfg_bin = atoi(v) ? 1 : 0;
            bin_n = 0;
            bin_seq = 0;
            // stdio turns every 0x0A byte into 0x0D 0x0A, which would corrupt binary
            // data. Turn that off while binary mode is on.
            stdio_set_translate_crlf(&stdio_usb, !cfg_bin);
        } else if (!strcmp(k, "invert")) {
            cfg_invert = atoi(v) ? 1 : 0;
        } else if (!strcmp(k, "offset")) {
            cfg_offset_ua = atof(v);
        } else if (!strcmp(k, "compact")) {
            cfg_compact = atoi(v) ? 1 : 0;
        } else if (!strcmp(k, "cal")) {
            cfg_cal = (uint16_t)(atoi(v) & 0x7FFF);
        } else {
            printf("# ERR unknown setting '%s'\n", k);
            return;
        }
        apply_config();
        print_header();
    } else {
        printf("# ERR unknown command '%s' -- try help\n", line);
    }
}

// ---------------------------------------------------------------- main
int main(void) {
    stdio_init_all();

    spi_init(SPI_PORT, SPI_HZ);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);  // INA229 is SPI mode 1
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    gpio_init(PIN_ALERT);
    gpio_set_dir(PIN_ALERT, GPIO_IN);
    gpio_pull_up(PIN_ALERT);
    gpio_set_irq_enabled_with_callback(PIN_ALERT, GPIO_IRQ_EDGE_FALL, true, &alert_isr);

    sleep_ms(10);
    reg_write(R_CONFIG, 1u << 15);   // RST -- start from the power-on state
    sleep_ms(5);
    apply_config();
    accum_zero();
    t_zero_us = time_us_64();

    printf("\n# ina229 ammeter ready -- 'id' to check wiring, 'help' for commands\n");

    char line[128];
    int  len = 0;

    uint64_t next_sample_us = time_us_64();
    uint64_t next_row_us    = time_us_64();

    // Input is checked every 5 ms, not on every pass. getchar and the USB background task
    // share one mutex, and the task skips its turn when it cannot get it (see
    // pico_stdio_usb/stdio_usb.c). Checking input non-stop starves USB: the send buffer
    // fills and rows get dropped. In my test, 1 of 100 rows arrived.
    uint64_t next_input_us = time_us_64();

    for (;;) {
        uint64_t now = time_us_64();

        if (now >= next_input_us) {
            next_input_us = now + 5000;
            int c = getchar_timeout_us(0);
            while (c != PICO_ERROR_TIMEOUT) {
                if (c == '\n' || c == '\r') {
                    if (len) { line[len] = 0; handle(line); len = 0; }
                } else if (len < (int)sizeof(line) - 1) {
                    line[len++] = (char)c;
                }
                c = getchar_timeout_us(0);
            }
        }

        if (cfg_bin && streaming && stdio_usb_connected()) {
            // Binary mode: read one conversion, store it, nothing else.
            //
            // Read the time again here. `now` was read before the input check, so on the
            // pass that runs `start` it is older than the new t_zero_us. The first packet
            // then got a negative time, which wraps to about 4295 s as unsigned.
            now = time_us_64();
            // Sampling is timed by the conversion period (ALERT cannot be used, see above).
            // See the table at cfg_vshct_i for the resolution each setting gives.
            if (now >= next_sample_us) {
                next_sample_us = now + conv_period_us;
                if (bin_n == 0) {
                    bin_buf[0] = BIN_MAGIC0; bin_buf[1] = BIN_MAGIC1;
                    bin_buf[2] = (uint8_t)(bin_seq & 0xFF);
                    bin_buf[3] = (uint8_t)(bin_seq >> 8);
                    uint32_t t = (uint32_t)(now - t_zero_us);
                    bin_buf[6] = (uint8_t)t; bin_buf[7] = (uint8_t)(t >> 8);
                    bin_buf[8] = (uint8_t)(t >> 16); bin_buf[9] = (uint8_t)(t >> 24);
                    uint64_t qr = reg_read(R_CHARGE, 5);
                    for (int b = 0; b < 5; b++)
                        bin_buf[10 + b] = (uint8_t)(qr >> (8 * (4 - b)));
                }
                reg_read_raw(cfg_src, &bin_buf[BIN_HDR + 3 * bin_n], 3);
                if (++bin_n >= BIN_BATCH) bin_flush();
            }
            uint64_t a2 = time_us_64();
            uint64_t s2 = (next_sample_us > a2) ? (next_sample_us - a2) : 0;
            if (s2 > 200) sleep_us(200); else if (s2 > 8) sleep_us(s2 - 4);
            continue;
        }

        if (now >= next_sample_us) {
            next_sample_us = now + conv_period_us;
            int32_t raw_i = s20_of(reg_read(R_CURRENT, 3));
            double  i_a   = (double)raw_i * current_lsb;
            if (i_n == 0) { i_min = i_max = i_a; }
            else if (i_a < i_min) i_min = i_a;
            else if (i_a > i_max) i_max = i_a;
            if (raw_i >= SAT_COUNTS || raw_i <= -SAT_COUNTS) i_sat++;
            i_raw_sum += raw_i;
            i_n++;
        }

        // No rows while no host is connected. Writes would be thrown away anyway, and
        // skipping the row keeps prev_q_raw/prev_row_us, so the first row after the host
        // comes back gives the right average over the whole gap.
        if (streaming && stdio_usb_connected() && now >= next_row_us) {
            next_row_us = now + (uint64_t)(1e6 / cfg_rate_hz);
            emit_row();
        }

        // Sleep until the next sample is due, but at least 20 us, so the USB task always
        // gets a turn (see the input check above).
        uint64_t after = time_us_64();
        uint64_t slack = (next_sample_us > after) ? (next_sample_us - after) : 0;
        sleep_us(slack > 200 ? 200 : (slack < 20 ? 20 : slack));
    }
}
