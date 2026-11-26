/* ============================================================================
 * PROJECT   : TENNIS Dt LOGGER — LIVE + RAW VERSION (PCB Version 2.5)
 * HARDWARE  : MSP430FR2355 (see note) + LIS3DHTR (I2C) + RN4678 (UART/Bluetooth)
 *             NOTE: the original source header says FR2433, but the registers it
 *             drives (P4.x with UCA1/UCB1, Timer_B0, DCOFTRIMEN) do not exist on
 *             an FR2433 — select the chip actually printed on the board in CCS
 *             (FR2355 matches this pin/peripheral set).
 *
 * ALWAYS-ON. Accelerometer @ 400 Hz (see MEASURED note below), four causal detectors, results pushed
 * live over Bluetooth while streaming is ON.
 *
 *   Tf  Trunk  1-3 Hz      negative VALLEY of the filtered signal
 *   C   Chest  2.5-7 Hz    peak of the envelope (largest one in window)
 *   W   Wrist  8-15 Hz     peak of the envelope (latest one in window)  (back to 8-15 Hz on 2026-10-27;
 *                          6-12 Hz was tried 2026-10-22 to 10-27)
 *   I   Impact 20-40 Hz    peak of the envelope  (t = 0)
 *
 * WHERE THE BANDS COME FROM (checked 2026-10-12/15 on the 14 Sep recordings):
 *   - FFT of the raw composite |a| (3 s around 139 video-confirmed Shimmer shots and
 *     101 Paul-board shots, STEP1): Impact energy 15-35 Hz on the Paul board -> 20-40 Hz kept;
 *     a trunk bump at 1.0-1.4 Hz on both sensors -> 1-3 Hz kept.
 *   - Chest and Wrist show no peak of their own with one sensor on the wrist: their bands are
 *     the Shimmer Python reference bands. Wrist was 10.5-12.5 Hz until 2026-10-06 (too narrow:
 *     the Session 34 swing reached only 2.7 m/s^2), now 8-15 Hz like the reference.
 *   - Slow bands must stay below ~15 Hz: they run at 50 Hz after a 16 Hz anti-alias filter.
 *
 * BUTTON: short press = streaming ON/OFF (a PERF line is sent on OFF)
 *         long press  = reset all detectors (re-runs the warm-up)
 *
 * MEASURED ON THE BOARD (PERF lines, CPU at 8 MHz, no compiler optimisation):
 *   400 Hz first try : 176 Hz effective, I2C 1.74 ms + processing 3.9 ms, every sample late.
 *   200 Hz           : 197 Hz effective, I2C 1.13 ms + processing 3.86 ms, 4310/4436 late.
 * The software-float filters cost ~4 ms per sample at 8 MHz, so neither rate really fitted.
 *
 * THIS VERSION: 400 Hz again (2.5 ms period), made to fit by
 *   - running the CPU at 24 MHz instead of 8 MHz (3x faster; FRAM wait states enabled),
 *   - decimating by 8 instead of 4, so the three slow bands still run at 50 Hz and cost
 *     half as much per raw sample as before.
 * EXPECTED (not yet measured): I2C ~0.75 ms + processing ~1.1 ms average.
 * CHECK THE PERF LINE: rate_hz must read ~400 and late should be close to 0.
 * Still worth doing: compiler optimisation level 3 in the CCS project properties.
 *
 * WHAT THIS VERSION ADDS vs main_live_multirate.c
 *  1. DELAY COMPENSATION. Causal filters delay each band by a different
 *     amount (Trunk ~565 ms, Chest ~560 ms, Wrist ~200 ms, Impact ~55 ms, including the
 *     envelope smoothing; see the DELAY_xxx_MS defines). The Python reference uses
 *     zero-phase filtfilt(), so it has none of that. Each event time is
 *     shifted back by its band's estimated delay so Dt values are
 *     comparable with the reference. Validate them against the Python
 *     reference on a recorded session (roadmap Step 3).
 *  2. DEFERRED SHOT ASSEMBLY. Because of those delays, Wrist/Trunk events
 *     are detected AFTER the Impact fires. The shot is therefore assembled
 *     ASSEMBLY_HOLDOFF_MS after Impact, choosing per band the event inside
 *     the Python search window (Trunk: latest, Chest: largest, Wrist:
 *     latest), from a short event history. Output arrives ~1 s after the
 *     swing.
 *  3. NON-BLOCKING UART. Text is queued in a ring buffer and sent from the
 *     main loop. Before, a ~110-byte SHOT line blocked for ~10 ms, i.e.
 *     about 4 accelerometer samples were lost every shot.
 *  4. WARM-UP. Filters start at zero while gravity is a DC step, which
 *     rings for seconds in the narrow bands. Detection is disabled for
 *     WARMUP_MS after boot/reset, then "WARMUP_DONE" is sent.
 *  5. TIMING INSTRUMENTATION (roadmap Step 4). PERF line reports I2C read
 *     time, processing time, late samples and dropped TX bytes.
 *  6. Atomic 32-bit ms read (ms_tick is updated in an interrupt).
 *  7. I2C_DELAY_CYCLES is a macro (see the note next to it).
 *  8. ENV debug line once per second while streaming (ENV_DEBUG), used to tune thresholds.
 *  9. If several Impact pulses fire within the hold-off, the LARGEST one is kept as the
 *     shot's t = 0 (before, a second pulse flushed the first shot too early).
 * 10. WAVE lines (WAVE_STREAM): the five waveforms at 50 Hz while streaming, so the PC can draw
 *     each shot as curves with Impact at t = 0 (see the WAVE comment further down).
 * 11. A LARGER pulse may start during the refractory time (PeakDetector_Process): before, a
 *     small bump just before the real impact blocked it for 300 ms, and the shot was reported
 *     late with the tail's peak (seen at 86.2 s in Session 34).
 *
 * NEW IN THIS FILE (2026-10-27, raw + MCU results in the same session):
 * 12. RAW LINES (RAW_STREAM): every accelerometer sample the detectors used is also sent,
 *     8 samples per 'R' line (see the RAW comment further down). The PC script emu_logger.py
 *     records them next to the SHOT lines, so Python can run the reference on EXACTLY the
 *     same data and compare its Dt with the MCU's, shot by shot, on the same ms clock.
 * 13. INTERRUPT-DRIVEN UART TX. Before, bytes were only sent from the main loop, so the UART
 *     sat idle during the ~1.8 ms of I2C + processing of each sample: only ~3 kB/s could get
 *     out. The raw stream needs ~4.3 kB/s, so bytes now go out from the USCI_A1 interrupt.
 *     CTS is still respected (the interrupt stops itself, the main loop restarts it).
 * 14. TX buffer 512 -> 2048 bytes (~350 ms of output), and a string is now queued whole or
 *     dropped whole, so a full buffer never produces half lines. tx_drops in PERF must be 0.
 *
 * 15. PRIORITY FOR SHOT LINES (2026-10-27, after the first court test): when the Bluetooth
 *     link slows down (range, body in the way), the module holds CTS and the TX buffer fills.
 *     Data lines (R raw, w wave, ENV) are now only queued while TX_RESERVE bytes stay free,
 *     so the reserve is always there for SHOT / PERF / status lines: a weak link loses raw
 *     data but never a shot result. Every line is built whole first (no half lines).
 *     PERF reports tx_drops (data bytes dropped) and tx_key_drops (must stay 0).
 *     On the court test the link fell to ~220 B/s for 3 minutes: 12 of 20 SHOT lines were lost.
 * 16. PerfNowCycles: fixed a race at the 1 ms timer wrap that made some PERF times huge
 *     (i2c_max_us = 178956794 = 2^32/24 = one wrap).
 *
 * 17. IMPACT CAP ON THE SLOW BANDS (2026-10-27). The ball impact (up to 16 g) leaked into the
 *     1-3 / 2.5-7 / 8-15 Hz filters and, once shifted back by their ~0.2-0.6 s delays, showed up
 *     BEFORE the impact as fake Trunk/Chest events (Tf ~ -0.27 s, Chest before Trunk on 88/99
 *     swings). |a| is now capped at SLOW_CAP = 3.5 g before the slow bands only (the Impact band
 *     still sees the real peak). 3.5 g is where the Shimmer3 saturates (2 g per axis), and it was
 *     the best fix in trunk_fix_check.py. Thresholds, delays and the Chest envelope were then
 *     retuned on the 101 Paul swings (Sessions 34-38, 41) against the Python reference with the
 *     same cap (5bands_dt_V2806_1_cap35.py). Checked with THIS code on the 101 swings:
 *       dtTfC > 0 (trunk before chest): 11/99 before -> 99/99 after
 *       vs Shimmer, median |error|: TfC 206 -> 49 ms, CW 151 -> 64 ms, TfI 103 -> 93 ms,
 *       TfW 66 -> 82 ms (the Wrist event is still the weak point, also in the Python reference).
 *
 * 18. FIR HILBERT ENVELOPE (2026-11-09, ENVELOPE_FIR = 1). Chest, Wrist and Impact now use the same
 *     envelope as the Python reference, |y + j.H(y)|, computed in real time: a FIR filter shifts
 *     the band signal by 90 deg (H), and env = sqrt(y_delayed^2 + H^2). Coefficients: scipy
 *     remez(type='hilbert'), odd length, every second coefficient is 0:
 *       Chest  27 taps at 50 Hz (fixed delay 260 ms), gain 0.99-1.01 over 2.5-7 Hz
 *       Wrist  15 taps at 50 Hz (140 ms), gain 1.000 over 8-15 Hz
 *       Impact 27 taps at 400 Hz (32.5 ms), gain 0.99-1.01 over 20-40 Hz
 *     Checked on recorded swings: this envelope equals scipy hilbert() within 0.4 %, shifted by
 *     a fixed (N-1)/2 samples. Chest and Wrist events are now every LOCAL MAXIMUM of the envelope
 *     (same rule as compute_peak_time in Python), not a pulse with a 70 % fall. Impact keeps the
 *     pulse detector (no smoothing, threshold 5.5). Tuned on the 101 swings vs the capped Python
 *     reference (V2806_1), events within 20 ms of Python: Chest 45 -> 54, Wrist 25 -> 57;
 *     median |error| Chest 25 -> 16 ms, Wrist 56 -> 13 ms; Trunk unchanged; Impact 99/101 found,
 *     0.49 false shots/min (same as before). Set ENVELOPE_FIR to 0 for the old smoothing envelope.
 *     NOTE: the MCU now follows Python closely, so its Wrist-based dt also inherit the Python
 *     reference's own disagreement with the Shimmer (see the comparison in the 2026-11-09 notes).
 *
 * 19. SPEED (2026-11-10). First board test of item 18: rate_hz 359, late 62 %, processing
 *     1.8 ms + I2C 0.84 ms > 2.5 ms per sample. The Impact envelope's square root (400 per second,
 *     software floating point) is removed: the Impact detector now works on the squared envelope
 *     with squared threshold and fall ratio (same peaks); the root is taken only for the values
 *     sent (Ipk, ENV, WAVE). Checked on the 101 swings: identical SHOT lines.
 *
 * 20. SPEED, step 2 (2026-11-10). After item 19 the board reached 379 Hz (processing 1.69 ms +
 *     I2C 0.84 ms, still > 2.5 ms). IMPACT_ENVELOPE_FIR = 0: Impact goes back to the cheap
 *     |y| + smoothing envelope (its timing was already within 3 ms of Python); Chest and Wrist
 *     keep the FIR Hilbert envelope (they run at 50 Hz, so they cost little).
 *
 * 21. BLUETOOTH, step 2 (2026-11-10). With items 19-20 the board reached 403 Hz (processing 1.37 ms
 *     + I2C 0.84 ms), but the link only delivered ~5.3 kB/s on the desk for ~5.75 kB/s produced:
 *     7 % of the raw lines were dropped (tx_drops; shot results all arrived). WAVE lines are now
 *     OFF (-1.35 kB/s): live_monitor.py computes the same curves from the raw samples. The TX
 *     interrupt is also restarted from the 1 ms timer after a CTS pause.
 *
 * BLUETOOTH LOAD while streaming (115200 baud = 11.5 kB/s max):
 *   R ~4250 B/s + ENV/SHOT ~100 B/s = ~4.4 kB/s (WAVE off; it added ~1350 B/s).
 *   If tx_drops is not 0, set WAVE_STREAM to 0 first (the PC can rebuild the curves from R).
 *
 * THRESHOLDS (m/s^2, with the 3.5 g cap): Trunk -3.0, Chest 0.3, Wrist 0.5, Impact 3.5.
 * (Without the cap they were -6.0 / 5.0 / 4.0: those values were set on the impact leak.)
 * The detection figures below are from BEFORE the cap (old thresholds); see item 17 for after.
 * Checked by running THIS file
 * on a PC (HOST_SIM) over 101 recorded Paul-board swings (Sessions 34-38, 41, 14 Sep 2026):
 * Impact 99/101 shots (timing error median 3 ms, max 25 ms), 0.49 false shots per minute;
 * events found in their windows: Trunk 99/101, Chest 99/101, Wrist 81/101; all four events
 * in 81/101 shots. (Chest was 6/101 with DELAY_CHEST_MS = 265: the delay was too short.)
 * NOT YET TESTED ON THE BOARD OUTSIDE: re-check with the ENV line.
 * ============================================================================ */

#ifndef HOST_SIM
#include <msp430.h>
#endif
#include <stdint.h>
#include <math.h>

/* ============================================================================
 * PORTABLE CORE (also compiled on a PC for testing, see host_sim.c)
 * ============================================================================ */

#define SAMPLE_RATE_HZ       400        /* accelerometer ODR (CTRL_REG1 = 0x77) */
#define DECIM_FACTOR         8          /* 400 Hz / 8 = 50 Hz low-rate branch */
#define ACC_CONV_FACTOR      0.0073575f /* raw count -> m/s^2 (16G, same as accel_receiver.py) */

#define WARMUP_MS            6000UL
#define ASSEMBLY_HOLDOFF_MS  1000UL

/* Causal delay per band (band-pass + anti-alias + envelope smoothing), in ms,
 * subtracted from the detected event time. Trunk, Chest and Impact were checked against the
 * Python event times on 101 recorded swings (Trunk 564 ms, Chest ~560 ms, Impact 58 ms).
 * Wrist is an estimate (filter group delay + envelope lag; board vs Python median +8 ms). */
/* With the 3.5 g cap (2026-10-27), tuned on the 101 swings against the capped Python reference:
 *   Trunk 570 ms: 83/101 swings within 20 ms of Python (median |error| 8 ms)
 *   Chest 200 ms (with the faster 8 Hz envelope): median |error| 25 ms, found on 99/99
 *   Wrist 170 ms: median |error| 56 ms
 * Before the cap: Trunk 565, Chest 560, Wrist 200 (those were partly aligned on the impact leak). */
/* Envelope used for Chest / Wrist / Impact: 1 = FIR Hilbert (same as Python), 0 = |y| + smoothing. */
#define ENVELOPE_FIR         1
/* Impact only (runs at 400 Hz, so it decides the processor load): 0 = |y| + smoothing (cheap),
 * 1 = FIR Hilbert. Board test 2026-11-10: with the FIR on Impact the board only reached 379 Hz.
 * Impact timing vs Python on the 101 swings: smoothing median 3 ms (max 25), FIR 2 ms (max 20). */
#define IMPACT_ENVELOPE_FIR  0

#define DELAY_TRUNK_MS       570
#if ENVELOPE_FIR
/* FIR Hilbert envelope (item 18): band-pass delay + FIR delay ((N-1)/2 samples), measured against
 * the Python event times on the 101 swings. FIR part: Chest 260, Wrist 140, Impact 32.5 ms. */
#define DELAY_CHEST_MS       500
#define DELAY_WRIST_MS       305
#if IMPACT_ENVELOPE_FIR
#define DELAY_IMPACT_MS      80
#else
#define DELAY_IMPACT_MS      55
#endif
#else
#define DELAY_CHEST_MS       200
#define DELAY_WRIST_MS       170
#define DELAY_IMPACT_MS      55
#endif

/* |a| cap for the slow bands only (Trunk / Chest / Wrist): 3.5 g in m/s^2. See item 17. */
#define SLOW_CAP             34.3232750f

/* Search windows relative to Impact (t=0), ms — same as the Python reference */
#define TRUNK_WIN_MIN_MS  (-1200L)
#define TRUNK_WIN_MAX_MS  (-200L)
#define CHEST_WIN_MIN_MS  (-400L)
#define CHEST_WIN_MAX_MS  (-50L)
#define WRIST_WIN_MIN_MS  (-200L)
#define WRIST_WIN_MAX_MS  (0L)

volatile uint32_t ms_tick = 0;
volatile uint8_t  g_streaming = 0;
volatile uint32_t shot_count = 0;
static uint8_t    decim_counter = 0;
static uint8_t    g_detect_on = 0;
static uint32_t   g_warmup_end_ms = 0;

static uint32_t GetMs(void) {
    uint32_t a, b;
    do { a = ms_tick; b = ms_tick; } while (a != b);
    return a;
}

/* ---------------- non-blocking UART TX ring buffer ---------------- */
#define TX_BUF_SIZE 2048                /* power of two (FR2355 has 4 KB of RAM) */
static char              tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0, tx_tail = 0;
static uint32_t          tx_drops = 0;      /* data bytes (R / w / ENV) dropped: link too slow */
static uint32_t          tx_key_drops = 0;  /* SHOT / PERF / status bytes dropped: must stay 0 */
#define TX_RESERVE 512                      /* bytes kept free for SHOT / PERF / status lines */

#ifndef HOST_SIM
static void UART_Kick(void);            /* MCU layer: starts the TX interrupt */
#endif

/* Queues a whole line; never blocks. If it does not fit, the WHOLE line is dropped (no half
 * lines on the PC side). key = 1 for SHOT / PERF / status lines: they may use the reserve.
 * key = 0 for data lines (raw, wave, ENV): they are dropped first when the link is slow. */
static void UART_Queue(const char *str, uint8_t key) {
    uint16_t len = 0, space, i;
    while (str[len]) len++;
    space = (uint16_t)((tx_tail - tx_head - 1) & (TX_BUF_SIZE - 1));
    if (len > space || (!key && (uint16_t)(len + TX_RESERVE) > space)) {
        if (key) tx_key_drops += len; else tx_drops += len;
        return;
    }
    for (i = 0; i < len; i++) {
        tx_buf[tx_head] = str[i];
        tx_head = (uint16_t)((tx_head + 1) & (TX_BUF_SIZE - 1));
    }
#ifndef HOST_SIM
    UART_Kick();
#endif
}

void UART1_Send_String(const char *str) { UART_Queue(str, 1); }   /* status lines */
static void UART1_Send_Data(const char *str) { UART_Queue(str, 0); }

/* Line builder: a SHOT / PERF / ENV line is built here whole, then queued in one go. */
#define LB_SIZE 200
static char    lb[LB_SIZE];
static uint8_t lb_n = 0;

static void LB_Str(const char *str) {
    while (*str && lb_n < LB_SIZE - 1) lb[lb_n++] = *str++;
}
static void LB_U32(uint32_t val) {
    char buf[11];
    int8_t i = 10;
    buf[10] = '\0';
    if (val == 0) { LB_Str("0"); return; }
    while (val > 0 && i > 0) { i--; buf[i] = (char)('0' + (val % 10)); val /= 10; }
    LB_Str(&buf[i]);
}
static void LB_I32(int32_t val) {
    if (val < 0) { LB_Str("-"); LB_U32((uint32_t)(-val)); }
    else         { LB_U32((uint32_t)val); }
}
static void SendField_I32(const char *key, uint8_t valid, int32_t value) {
    LB_Str(key); LB_Str("=");
    if (valid) LB_I32(value); else LB_Str("NA");
    LB_Str(";");
}
static void SendField_U32(const char *key, uint32_t value) {
    LB_Str(key); LB_Str("="); LB_U32(value); LB_Str(";");
}
static void LB_Send(uint8_t key) {
    lb[lb_n] = '\0';
    UART_Queue(lb, key);
    lb_n = 0;
}

/* Integer square root (bit-by-bit). Replaces sqrtf(): the software-float square
 * root was one of the most expensive operations per sample on this MCU. */
static uint16_t isqrt32(uint32_t val) {
    uint32_t res = 0;
    uint32_t bit = 1UL << 30;
    while (bit > val) bit >>= 2;
    while (bit != 0) {
        if (val >= res + bit) { val -= res + bit; res = (res >> 1) + bit; }
        else                  { res >>= 1; }
        bit >>= 2;
    }
    return (uint16_t)res;
}

/* ---------------- RAW stream (raw samples for the Python reference) ----------------
 * While streaming, every sample read from the accelerometer is sent, 8 per line:
 *   'R' + 8 hex  ms clock of the FIRST sample (full 32 bits, same clock as SHOT "I=")
 *       + 2 hex  block counter (0-255, restarts at each STREAM_START; a jump = lost line)
 *       + 8 x (3 hex X, 3 hex Y, 3 hex Z)  + CR LF                      = 85 bytes / 20 ms
 * Each axis is the LIS3DH value >> 4 as 12-bit two's complement. In high-resolution mode
 * (CTRL_REG4 = 0xB8) the 4 low bits are always 0, so this is LOSSLESS: the PC shifts it
 * back << 4 and gets the exact same 16-bit counts as the old flash dump.
 * Set RAW_STREAM to 0 to remove. */
#define RAW_STREAM 1
#if RAW_STREAM
#define RAW_BLOCK     8
#define RAW_LINE_LEN  (1 + 8 + 2 + RAW_BLOCK * 9 + 2)
static const char RAW_HEX[] = "0123456789ABCDEF";
static char    raw_line[RAW_LINE_LEN + 1];
static char   *raw_p = raw_line;
static uint8_t raw_n = 0;
static uint8_t raw_seq = 0;

static char *Raw_Hex12(char *p, int16_t value) {
    uint16_t v = (uint16_t)value >> 4;          /* unsigned shift: keeps the 12 data bits */
    *p++ = RAW_HEX[(v >> 8) & 0xF];
    *p++ = RAW_HEX[(v >> 4) & 0xF];
    *p++ = RAW_HEX[v & 0xF];
    return p;
}

static void Raw_Add(int16_t x, int16_t y, int16_t z, uint32_t now) {
    char *p;
    if (!g_streaming) { raw_n = 0; raw_seq = 0; return; }
    if (raw_n == 0) {
        int8_t s;
        p = raw_line;
        *p++ = 'R';
        for (s = 28; s >= 0; s -= 4) *p++ = RAW_HEX[(now >> s) & 0xF];
        *p++ = RAW_HEX[(raw_seq >> 4) & 0xF];
        *p++ = RAW_HEX[raw_seq & 0xF];
    } else {
        p = raw_p;
    }
    p = Raw_Hex12(p, x);
    p = Raw_Hex12(p, y);
    p = Raw_Hex12(p, z);
    raw_p = p;
    if (++raw_n >= RAW_BLOCK) {
        *p++ = '\r'; *p++ = '\n'; *p = '\0';
        UART1_Send_Data(raw_line);
        raw_seq++;
        raw_n = 0;
    }
}
#endif

/* ---------------- biquad cascade (Direct-Form-II-Transposed) ---------------- */
#define BQ_SECTIONS 4
#define AA_SECTIONS 2

static float BiquadCascade(const float *b0, const float *b1, const float *b2,
                            const float *a1, const float *a2,
                            float *z1, float *z2, uint8_t n_sections, float x) {
    uint8_t s;
    float y;
    for (s = 0; s < n_sections; s++) {
        y     = b0[s] * x + z1[s];
        z1[s] = b1[s] * x - a1[s] * y + z2[s];
        z2[s] = b2[s] * x - a2[s] * y;
        x = y;
    }
    return x;
}

/* Anti-alias low-pass, order 4, 16 Hz, Fs = 400 Hz (runs on every sample, feeds the decimated branch) */
static const float AA_B0[AA_SECTIONS] = { 0.0001832160f, 1.0000000000f };
static const float AA_B1[AA_SECTIONS] = { 0.0003664320f, 2.0000000000f };
static const float AA_B2[AA_SECTIONS] = { 0.0001832160f, 1.0000000000f };
static const float AA_A1[AA_SECTIONS] = { -1.5752399778f, -1.7688278599f };
static const float AA_A2[AA_SECTIONS] = { 0.6263342592f, 0.8262013329f };
static float aa_z1[AA_SECTIONS], aa_z2[AA_SECTIONS];

/* Trunk 1-3 Hz, Chest 2.5-7 Hz, Wrist 8-15 Hz — designed for Fs = 50 Hz (decimated by 8).
 * All are scipy butter(4, band, btype='band', fs=50, output='sos').
 * Wrist history: 10.5-12.5 Hz until 2026-10-06, then 8-15 Hz (Shimmer Python reference band),
 * 6-12 Hz from 2026-10-22 (found the wrist event more often in the PC simulation: 87/101),
 * back to 8-15 Hz on 2026-10-27 so the MCU uses the SAME band as the Python reference
 * and the midterm results. Note: the top of the band (15 Hz) is close to the 16 Hz
 * anti-alias low-pass, which takes about 2 dB off there. */
static const float TRUNK_B0[BQ_SECTIONS] = { 0.0001832160f, 1.0000000000f, 1.0000000000f, 1.0000000000f };
static const float TRUNK_B1[BQ_SECTIONS] = { 0.0003664320f, 2.0000000000f, -2.0000000000f, -2.0000000000f };
static const float TRUNK_B2[BQ_SECTIONS] = { 0.0001832160f, 1.0000000000f, 1.0000000000f, 1.0000000000f };
static const float TRUNK_A1[BQ_SECTIONS] = { -1.6776208078f, -1.8128195968f, -1.7450037807f, -1.9344328787f };
static const float TRUNK_A2[BQ_SECTIONS] = { 0.7465797296f, 0.8389382062f, 0.8690062384f, 0.9507426948f };
static float trunk_z1[BQ_SECTIONS], trunk_z2[BQ_SECTIONS];

static const float CHEST_B0[BQ_SECTIONS] = { 0.0033628151f, 1.0000000000f, 1.0000000000f, 1.0000000000f };
static const float CHEST_B1[BQ_SECTIONS] = { 0.0067256303f, 2.0000000000f, -2.0000000000f, -2.0000000000f };
static const float CHEST_B2[BQ_SECTIONS] = { 0.0033628151f, 1.0000000000f, 1.0000000000f, 1.0000000000f };
static const float CHEST_A1[BQ_SECTIONS] = { -1.1822432269f, -1.5131338078f, -1.1424911391f, -1.7869881201f };
static const float CHEST_A2[BQ_SECTIONS] = { 0.5150261084f, 0.6558063227f, 0.7457043834f, 0.8846392484f };
static float chest_z1[BQ_SECTIONS], chest_z2[BQ_SECTIONS];

static const float WRIST_B0[BQ_SECTIONS] = { 0.0148267455f, 1.0000000000f, 1.0000000000f, 1.0000000000f };
static const float WRIST_B1[BQ_SECTIONS] = { 0.0296534910f, -2.0000000000f, 2.0000000000f, -2.0000000000f };
static const float WRIST_B2[BQ_SECTIONS] = { 0.0148267455f, 1.0000000000f, 1.0000000000f, 1.0000000000f };
static const float WRIST_A1[BQ_SECTIONS] = { 0.0847975374f, -0.4649850437f, 0.4902714305f, -0.9036790820f };
static const float WRIST_A2[BQ_SECTIONS] = { 0.3990501199f, 0.4217988304f, 0.7252303202f, 0.7508860919f };
static float wrist_z1[BQ_SECTIONS], wrist_z2[BQ_SECTIONS];

/* Impact 20-40 Hz — Fs = 400 Hz */
static const float IMPACT_B0[BQ_SECTIONS] = { 0.0004165992f, 1.0000000000f, 1.0000000000f, 1.0000000000f };
static const float IMPACT_B1[BQ_SECTIONS] = { 0.0008331984f, 2.0000000000f, -2.0000000000f, -2.0000000000f };
static const float IMPACT_B2[BQ_SECTIONS] = { 0.0004165992f, 1.0000000000f, 1.0000000000f, 1.0000000000f };
static const float IMPACT_A1[BQ_SECTIONS] = { -1.4941763257f, -1.6448831345f, -1.5176347350f, -1.8210534220f };
static const float IMPACT_A2[BQ_SECTIONS] = { 0.7165735039f, 0.7756657764f, 0.8576289713f, 0.9193949438f };
static float impact_z1[BQ_SECTIONS], impact_z2[BQ_SECTIONS];

/* ---------------- peak detector (Chest / Wrist / Impact) ---------------- */
typedef struct {
    float    env_state;
    uint8_t  in_pulse;
    float    peak_val;
    uint32_t peak_ms;
    uint32_t last_event_ms;
    float    alpha;
    float    threshold;
    float    fall_ratio;
    uint32_t refractory_ms;
    uint32_t last_confirmed_ms;
    float    last_confirmed_val;
    uint8_t  local_max;        /* 1: input is an envelope, every local maximum above threshold is an event */
    uint8_t  n_seen;
    float    prev1, prev2;     /* last two envelope values (local-maximum mode) */
    uint32_t prev1_ms;
} PeakDetector;

static uint8_t PeakDetector_Process(PeakDetector *d, float filtered_value, uint32_t now_ms) {
    float rectified;
    if (d->local_max) {
        /* Same rule as Python compute_peak_time(): sample j is a peak if
         * env[j-1] < env[j] >= env[j+1]. Sample j is the previous one, so the event is
         * known one sample later (20 ms at 50 Hz), with the time of sample j. */
        uint8_t ev = 0;
        if (d->n_seen >= 2 && d->prev2 < d->prev1 && d->prev1 >= filtered_value && d->prev1 > d->threshold) {
            d->last_confirmed_ms  = d->prev1_ms;
            d->last_confirmed_val = d->prev1;
            d->last_event_ms      = d->prev1_ms;
            ev = 1;
        }
        d->prev2 = d->prev1; d->prev1 = filtered_value; d->prev1_ms = now_ms;
        if (d->n_seen < 2) d->n_seen++;
        d->env_state = filtered_value;
        return ev;
    }
    rectified = fabsf(filtered_value);
    d->env_state = d->alpha * d->env_state + (1.0f - d->alpha) * rectified;

    if (!d->in_pulse) {
        /* Start a pulse after the refractory time, OR earlier if this one is already bigger
         * than the last confirmed pulse (otherwise a small bump hides the real peak). */
        if (d->env_state > d->threshold &&
            ((now_ms - d->last_event_ms) > d->refractory_ms || d->env_state > d->last_confirmed_val)) {
            d->in_pulse = 1;
            d->peak_val = d->env_state;
            d->peak_ms  = now_ms;
        }
    } else {
        if (d->env_state > d->peak_val) {
            d->peak_val = d->env_state;
            d->peak_ms  = now_ms;
        } else if (d->env_state < d->peak_val * d->fall_ratio) {
            d->last_confirmed_ms  = d->peak_ms;
            d->last_confirmed_val = d->peak_val;
            d->last_event_ms      = d->peak_ms;
            d->in_pulse = 0;
            return 1;
        }
    }
    return 0;
}

static void PeakDetector_Reset(PeakDetector *d) {
    d->env_state = 0.0f; d->in_pulse = 0; d->peak_val = 0.0f; d->peak_ms = 0; d->last_event_ms = 0;
    d->last_confirmed_val = 0.0f; d->last_confirmed_ms = 0;
    d->n_seen = 0; d->prev1 = 0.0f; d->prev2 = 0.0f; d->prev1_ms = 0;
}

/* ---------------- FIR Hilbert envelope (item 18) ----------------
 * H[n] = sum over odd k of g_k * (x[n-c-k] - x[n-c+k]),  c = (N-1)/2,
 * env[n] = sqrt(x[n-c]^2 + H[n]^2)  ->  the Python envelope |hilbert(x)|, delayed by c samples.
 * g_k = remez(N, [lo, fs/2-lo], [1], type='hilbert') at offsets c+1, c+3, ... (others are 0).
 * Chest (27 taps at 50 Hz) and Impact (27 taps at 400 Hz) have the same normalised band
 * (lo = 0.04 fs), hence the same coefficients. */
#define FIR27_N 27
#define FIR15_N 15
static const float FIR27_G[7] = { -0.6321743831f, -0.1991476702f, -0.1064511592f, -0.0635005451f,
                                  -0.0382663986f, -0.0220953596f, -0.0149924415f };
static const float FIR15_G[4] = { -0.6101728004f, -0.1434097380f, -0.0401191574f, -0.0070748699f };

typedef struct {
    float       *buf;      /* last N input samples (circular) */
    const float *g;        /* coefficients for k = 1, 3, 5, ... */
    uint8_t      n, ng, pos;
} FirHilbert;

static float chest_fir_buf[FIR27_N], wrist_fir_buf[FIR15_N];
#if IMPACT_ENVELOPE_FIR
static float impact_fir_buf[FIR27_N];
#endif
static FirHilbert chest_fir  = { chest_fir_buf,  FIR27_G, FIR27_N, 7, 0 };
static FirHilbert wrist_fir  = { wrist_fir_buf,  FIR15_G, FIR15_N, 4, 0 };
#if IMPACT_ENVELOPE_FIR
static FirHilbert impact_fir = { impact_fir_buf, FIR27_G, FIR27_N, 7, 0 };
#endif

/* Returns the SQUARED envelope (x[n-c]^2 + H^2): no square root, which costs a lot on the MSP430
 * (no floating-point hardware). Used at 400 Hz for Impact; FirHilbert_Env() adds the root. */
static float FirHilbert_Env2(FirHilbert *f, float x) {
    uint8_t c = (uint8_t)((f->n - 1) >> 1), i;
    int16_t p = f->pos, a;
    float h = 0.0f, xc;
    f->buf[p] = x;                                  /* newest sample = x[n] */
    a = p - c; if (a < 0) a += f->n;                /* x[n-c] */
    xc = f->buf[a];
    for (i = 0; i < f->ng; i++) {
        uint8_t k = (uint8_t)(2 * i + 1);
        int16_t m1 = a - k, m2 = a + k;             /* x[n-c-k] and x[n-c+k] */
        if (m1 < 0) m1 += f->n;
        if (m2 >= f->n) m2 -= f->n;
        h += f->g[i] * (f->buf[m1] - f->buf[m2]);
    }
    if (++f->pos >= f->n) f->pos = 0;
    return xc * xc + h * h;
}

static float FirHilbert_Env(FirHilbert *f, float x) {
    return sqrtf(FirHilbert_Env2(f, x));
}

/* Impact value in m/s^2 from what the Impact detector holds (squared when ENVELOPE_FIR). */
static float ImpactValue(float v) {
#if ENVELOPE_FIR && IMPACT_ENVELOPE_FIR
    return sqrtf(v);
#else
    return v;
#endif
}

static void FirHilbert_Reset(FirHilbert *f) {
    uint8_t i;
    for (i = 0; i < f->n; i++) f->buf[i] = 0.0f;
    f->pos = 0;
}

/* ---------------- valley detector (Trunk) — signed signal, no envelope ---------------- */
typedef struct {
    uint8_t  in_trough;
    float    trough_val;
    uint32_t trough_ms;
    uint32_t last_event_ms;
    float    threshold_neg;
    float    recover_ratio;
    uint32_t refractory_ms;
    uint32_t last_confirmed_ms;
    float    last_confirmed_val;
} ValleyDetector;

static uint8_t ValleyDetector_Process(ValleyDetector *d, float v, uint32_t now_ms) {
    if (!d->in_trough) {
        if (v < d->threshold_neg && (now_ms - d->last_event_ms) > d->refractory_ms) {
            d->in_trough = 1; d->trough_val = v; d->trough_ms = now_ms;
        }
    } else {
        if (v < d->trough_val) {
            d->trough_val = v; d->trough_ms = now_ms;
        } else if (v > d->trough_val * d->recover_ratio) {
            d->last_confirmed_ms  = d->trough_ms;
            d->last_confirmed_val = d->trough_val;
            d->last_event_ms      = d->trough_ms;
            d->in_trough = 0;
            return 1;
        }
    }
    return 0;
}

static void ValleyDetector_Reset(ValleyDetector *d) {
    d->in_trough = 0; d->trough_val = 0.0f; d->trough_ms = 0; d->last_event_ms = 0;
}

/* Detector instances (units: m/s^2). alpha = exp(-2*pi*fc/fs) of the envelope smoothing:
 * Chest fc 8 Hz (was 1.5 Hz before the cap) and Wrist fc 5 Hz at 50 Hz, Impact fc 10 Hz at 400 Hz.
 * Thresholds with the 3.5 g cap (item 17); before the cap: Trunk -6.0, Chest 5.0, Wrist 4.0. */
static ValleyDetector trunk_det = { .threshold_neg = -3.0f, .recover_ratio = 0.70f, .refractory_ms = 300 };
#if ENVELOPE_FIR
/* FIR Hilbert envelope (item 18): Chest / Wrist = every local maximum of the envelope (threshold 0,
 * like Python); Impact = pulse detector on the unsmoothed envelope (alpha 0), threshold 5.5. */
static PeakDetector chest_det  = { .alpha = 0.0f, .threshold = 0.0f,  .fall_ratio = 0.70f, .refractory_ms = 120, .local_max = 1 };
static PeakDetector wrist_det  = { .alpha = 0.0f, .threshold = 0.0f,  .fall_ratio = 0.70f, .refractory_ms = 50,  .local_max = 1 };
/* Impact works on the SQUARED envelope (no square root at 400 Hz, see item 19):
 * threshold 5.5^2 = 30.25 and fall ratio 0.7^2 = 0.49 give exactly the same peaks. */
#if IMPACT_ENVELOPE_FIR
static PeakDetector impact_det = { .alpha = 0.0f, .threshold = 30.25f, .fall_ratio = 0.49f, .refractory_ms = 300 };
#else
static PeakDetector impact_det = { .alpha = 0.8546359991532334f, .threshold = 3.5f,  .fall_ratio = 0.70f, .refractory_ms = 300 };
#endif
#else
static PeakDetector chest_det  = { .alpha = 0.3659313069412933f, .threshold = 0.3f,  .fall_ratio = 0.70f, .refractory_ms = 120 };
static PeakDetector wrist_det  = { .alpha = 0.5334880910911033f, .threshold = 0.5f,  .fall_ratio = 0.70f, .refractory_ms = 50  };
static PeakDetector impact_det = { .alpha = 0.8546359991532334f, .threshold = 3.5f,  .fall_ratio = 0.70f, .refractory_ms = 300 };
#endif
/* Results of these thresholds on 100 recorded swings: see THRESHOLDS in the header. */

/* ---------------- event history (delay-compensated times) ---------------- */
#define HIST_N 8
typedef struct { int32_t t[HIST_N]; float v[HIST_N]; uint8_t n; uint8_t head; } EventHist;
static EventHist trunk_hist, chest_hist, wrist_hist;

static void Hist_Clear(EventHist *h) { h->n = 0; h->head = 0; }
static void Hist_Push(EventHist *h, int32_t t, float v) {
    h->t[h->head] = t; h->v[h->head] = v;
    h->head = (uint8_t)((h->head + 1) % HIST_N);
    if (h->n < HIST_N) h->n++;
}

/* mode 0: latest event inside the window; mode 1: largest-value event inside the window */
static uint8_t Hist_Select(const EventHist *h, int32_t t_imp, int32_t wmin, int32_t wmax,
                            uint8_t mode, int32_t *off_out, float *val_out) {
    uint8_t i, found = 0;
    float best_v = 0.0f;
    for (i = 0; i < h->n; i++) {                       /* newest first */
        uint8_t idx = (uint8_t)((h->head + HIST_N - 1 - i) % HIST_N);
        int32_t off = h->t[idx] - t_imp;
        if (off < wmin || off > wmax) continue;
        if (!found || (mode == 1 && h->v[idx] > best_v)) {
            found = 1; best_v = h->v[idx]; *off_out = off; *val_out = best_v;
        }
        if (mode == 0) break;
    }
    return found;
}

/* ---------------- performance statistics (roadmap Step 4) ---------------- */
typedef struct {
    uint32_t n, i2c_sum, i2c_max, proc_sum, proc_max, late;
} PerfStats;
static PerfStats perf;
static uint32_t perf_start_ms = 0;

static void Perf_Reset(void) {
    perf.n = 0; perf.i2c_sum = 0; perf.i2c_max = 0;
    perf.proc_sum = 0; perf.proc_max = 0; perf.late = 0;
    perf_start_ms = GetMs();
}
static void Perf_Add(uint32_t i2c_us, uint32_t proc_us) {
    perf.n++;
    perf.i2c_sum += i2c_us;   if (i2c_us  > perf.i2c_max)  perf.i2c_max  = i2c_us;
    perf.proc_sum += proc_us; if (proc_us > perf.proc_max) perf.proc_max = proc_us;
}
static void Perf_Report(void) {
    LB_Str("PERF;");
    {
        uint32_t elapsed = GetMs() - perf_start_ms;
        SendField_U32("n", perf.n);
        SendField_U32("rate_hz", elapsed ? (perf.n * 1000UL) / elapsed : 0);   /* should equal SAMPLE_RATE_HZ */
    }
    SendField_U32("i2c_avg_us",  perf.n ? perf.i2c_sum  / perf.n : 0);
    SendField_U32("i2c_max_us",  perf.i2c_max);
    SendField_U32("proc_avg_us", perf.n ? perf.proc_sum / perf.n : 0);
    SendField_U32("proc_max_us", perf.proc_max);
    SendField_U32("late", perf.late);
    SendField_U32("tx_drops", tx_drops);
    SendField_U32("tx_key_drops", tx_key_drops);
    LB_Str("\r\n");
    LB_Send(1);
    Perf_Reset();
}

/* ---------------- shot assembly ---------------- */
static uint8_t g_pending = 0;
static int32_t g_pending_t_imp = 0;
static uint32_t g_pending_deadline = 0;
static float g_pending_peak = 0.0f;

static void AssembleAndSendShot(void) {
    int32_t t_imp = g_pending_t_imp;
    int32_t off_trunk = 0, off_chest = 0, off_wrist = 0;
    float pk_trunk = 0.0f, pk_chest = 0.0f, pk_wrist = 0.0f;

    uint8_t trunk_valid = Hist_Select(&trunk_hist, t_imp, TRUNK_WIN_MIN_MS, TRUNK_WIN_MAX_MS, 0, &off_trunk, &pk_trunk);
    uint8_t chest_valid = Hist_Select(&chest_hist, t_imp, CHEST_WIN_MIN_MS, CHEST_WIN_MAX_MS, 1, &off_chest, &pk_chest);
    uint8_t wrist_valid = Hist_Select(&wrist_hist, t_imp, WRIST_WIN_MIN_MS, WRIST_WIN_MAX_MS, 0, &off_wrist, &pk_wrist);

    g_pending = 0;
    shot_count++;
    if (!g_streaming) return;   /* detection always runs; only transmission is gated */

    LB_Str("SHOT;");
    LB_U32(shot_count);
    LB_Str(";I=");
    LB_I32(t_imp);
    LB_Str(";");

    SendField_I32("Tf", trunk_valid, off_trunk);
    SendField_I32("C",  chest_valid, off_chest);
    SendField_I32("W",  wrist_valid, off_wrist);

    SendField_I32("dtTfC", trunk_valid && chest_valid, off_chest - off_trunk);
    SendField_I32("dtCW",  chest_valid && wrist_valid, off_wrist - off_chest);
    SendField_I32("dtTfW", trunk_valid && wrist_valid, off_wrist - off_trunk);

    /* reference-only (involve Impact; possible saturation, as in the Python spec) */
    SendField_I32("dtWI",  wrist_valid, 0 - off_wrist);
    SendField_I32("dtCI",  chest_valid, 0 - off_chest);
    SendField_I32("dtTfI", trunk_valid, 0 - off_trunk);

    /* Peak amplitudes (m/s^2 x10) of the selected events: use them to tune the thresholds */
    SendField_I32("Ipk", 1, (int32_t)(ImpactValue(g_pending_peak) * 10.0f));
    SendField_I32("Tpk", trunk_valid, (int32_t)(pk_trunk * 10.0f));
    SendField_I32("Cpk", chest_valid, (int32_t)(pk_chest * 10.0f));
    SendField_I32("Wpk", wrist_valid, (int32_t)(pk_wrist * 10.0f));

    LB_Str("\r\n");
    LB_Send(1);                 /* key line: may use the reserve, never cut */
}

static void Kinematic_Reset(void) {
    uint8_t s;
    for (s = 0; s < BQ_SECTIONS; s++) {
        trunk_z1[s] = 0; trunk_z2[s] = 0;  chest_z1[s] = 0; chest_z2[s] = 0;
        wrist_z1[s] = 0; wrist_z2[s] = 0;  impact_z1[s] = 0; impact_z2[s] = 0;
    }
    for (s = 0; s < AA_SECTIONS; s++) { aa_z1[s] = 0; aa_z2[s] = 0; }
    ValleyDetector_Reset(&trunk_det);
    PeakDetector_Reset(&chest_det);
    PeakDetector_Reset(&wrist_det);
    PeakDetector_Reset(&impact_det);
    FirHilbert_Reset(&chest_fir); FirHilbert_Reset(&wrist_fir);
#if IMPACT_ENVELOPE_FIR
    FirHilbert_Reset(&impact_fir);
#endif
    Hist_Clear(&trunk_hist); Hist_Clear(&chest_hist); Hist_Clear(&wrist_hist);
    g_pending = 0;
    shot_count = 0;
    decim_counter = 0;
    g_detect_on = 0;
    g_warmup_end_ms = GetMs() + WARMUP_MS;
}

/* Call once per RAW accelerometer sample (400 Hz). */
/* DEBUG: per-second maxima, sent as an ENV line while streaming (values x10, m/s^2).
 * Rmin/Rmax = raw |a| (about 98 at rest). I/C/W = envelope maxima, T = deepest trunk valley.
 * Compare I/C/W/T with the detector thresholds to tune them. Set ENV_DEBUG to 0 to remove. */
#define ENV_DEBUG 1
#if ENV_DEBUG
static float dbg_rmin = 1e9f, dbg_rmax = 0.0f, dbg_i = 0.0f, dbg_c = 0.0f, dbg_w = 0.0f, dbg_t = 0.0f;
static uint32_t dbg_next_ms = 0;
static float dbg_trunk_f = 0.0f;
#endif

/* WAVE: live waveform for the PC graph, one line every 20 ms (50 Hz) while streaming.
 * Format: 'w' + 6 fields of 4 hex digits + CR LF (27 bytes, ~1350 bytes/s):
 *   time (low 16 bits of the ms clock), then x10 m/s^2 as signed 16-bit:
 *   Rmax (largest raw |a| of the 8 samples), Trunk filtered value, Chest envelope,
 *   Wrist envelope, Impact envelope (largest of the 8 samples).
 * Hex is used because it needs no division (decimal printing is slow on this MCU).
 * The band values are NOT delay-compensated here; live_monitor.py shifts each curve back
 * by its DELAY_xxx_MS. Set WAVE_STREAM to 0 to remove. Watch tx_drops in the PERF line. */
#define WAVE_STREAM 0      /* OFF since 2026-11-10 (item 21): live_monitor.py rebuilds the curves from the raw data */
#if WAVE_STREAM
#define WAVE_SLOT 5                     /* a decimation slot where no slow band runs */
static float wave_rmax = 0.0f, wave_imax = 0.0f, wave_trunk = 0.0f;
static char *Wave_Hex16(char *p, int16_t value) {
    static const char HEX[] = "0123456789ABCDEF";
    uint16_t v = (uint16_t)value;
    *p++ = HEX[(v >> 12) & 0xF]; *p++ = HEX[(v >> 8) & 0xF];
    *p++ = HEX[(v >> 4) & 0xF];  *p++ = HEX[v & 0xF];
    return p;
}
#endif

static void Kinematic_ProcessSample(int16_t x_raw, int16_t y_raw, int16_t z_raw) {
    uint32_t now = GetMs();
#if RAW_STREAM
    Raw_Add(x_raw, y_raw, z_raw, now);   /* the exact input of the detectors, same ms clock */
#endif
    /* |a| in raw counts with integer maths only, then ONE float multiply to m/s^2 */
    int32_t xx = (int32_t)x_raw * x_raw;
    int32_t yy = (int32_t)y_raw * y_raw;
    int32_t zz = (int32_t)z_raw * z_raw;
    float R = (float)isqrt32((uint32_t)xx + (uint32_t)yy + (uint32_t)zz) * ACC_CONV_FACTOR;
#if ENV_DEBUG
    if (R < dbg_rmin) dbg_rmin = R;
    if (R > dbg_rmax) dbg_rmax = R;
#endif
#if WAVE_STREAM
    if (R > wave_rmax) wave_rmax = R;
#endif

    /* Filters always run (they must stay continuous); detection is gated by warm-up. */
    float f_impact = BiquadCascade(IMPACT_B0, IMPACT_B1, IMPACT_B2, IMPACT_A1, IMPACT_A2,
                                    impact_z1, impact_z2, BQ_SECTIONS, R);
#if ENVELOPE_FIR && IMPACT_ENVELOPE_FIR
    f_impact = FirHilbert_Env2(&impact_fir, f_impact);    /* squared envelope; runs always (FIR buffer continuity) */
#endif
    /* the slow bands see |a| capped at 3.5 g, so the impact spike cannot leak into them */
    float R_slow   = (R > SLOW_CAP) ? SLOW_CAP : R;
    float R_lp     = BiquadCascade(AA_B0, AA_B1, AA_B2, AA_A1, AA_A2, aa_z1, aa_z2, AA_SECTIONS, R_slow);

    if (!g_detect_on && (int32_t)(now - g_warmup_end_ms) >= 0) {
        g_detect_on = 1;
        UART1_Send_String("WARMUP_DONE\r\n");
    }

    if (g_detect_on && PeakDetector_Process(&impact_det, f_impact, now)) {
        /* Several pulses within the hold-off belong to the same swing: keep the largest. */
        if (!g_pending || impact_det.last_confirmed_val > g_pending_peak) {
            g_pending = 1;
            g_pending_t_imp = (int32_t)impact_det.last_confirmed_ms - DELAY_IMPACT_MS;
            g_pending_peak = impact_det.last_confirmed_val;
            g_pending_deadline = now + ASSEMBLY_HOLDOFF_MS;
        }
    }

    switch (decim_counter) {            /* staggered: at most one slow band per sample */
        case 0: {
            float f = BiquadCascade(TRUNK_B0, TRUNK_B1, TRUNK_B2, TRUNK_A1, TRUNK_A2,
                                     trunk_z1, trunk_z2, BQ_SECTIONS, R_lp);
#if ENV_DEBUG
            dbg_trunk_f = f;
#endif
#if WAVE_STREAM
            wave_trunk = f;
#endif
            if (g_detect_on && ValleyDetector_Process(&trunk_det, f, now))
                Hist_Push(&trunk_hist, (int32_t)trunk_det.last_confirmed_ms - DELAY_TRUNK_MS,
                          trunk_det.last_confirmed_val);
            break;
        }
        case 1: {
            float f = BiquadCascade(CHEST_B0, CHEST_B1, CHEST_B2, CHEST_A1, CHEST_A2,
                                     chest_z1, chest_z2, BQ_SECTIONS, R_lp);
#if ENVELOPE_FIR
            f = FirHilbert_Env(&chest_fir, f);
#endif
            if (g_detect_on && PeakDetector_Process(&chest_det, f, now))
                Hist_Push(&chest_hist, (int32_t)chest_det.last_confirmed_ms - DELAY_CHEST_MS,
                          chest_det.last_confirmed_val);
            break;
        }
        case 2: {
            float f = BiquadCascade(WRIST_B0, WRIST_B1, WRIST_B2, WRIST_A1, WRIST_A2,
                                     wrist_z1, wrist_z2, BQ_SECTIONS, R_lp);
#if ENVELOPE_FIR
            f = FirHilbert_Env(&wrist_fir, f);
#endif
            if (g_detect_on && PeakDetector_Process(&wrist_det, f, now))
                Hist_Push(&wrist_hist, (int32_t)wrist_det.last_confirmed_ms - DELAY_WRIST_MS,
                          wrist_det.last_confirmed_val);
            break;
        }
        default: break;
    }
#if WAVE_STREAM
    if (impact_det.env_state > wave_imax) wave_imax = impact_det.env_state;
    if (decim_counter == WAVE_SLOT) {
        if (g_streaming) {
            char line[28];
            char *p = line;
            *p++ = 'w';
            p = Wave_Hex16(p, (int16_t)(uint16_t)now);
            p = Wave_Hex16(p, (int16_t)(wave_rmax * 10.0f));
            p = Wave_Hex16(p, (int16_t)(wave_trunk * 10.0f));
            p = Wave_Hex16(p, (int16_t)(chest_det.env_state * 10.0f));
            p = Wave_Hex16(p, (int16_t)(wrist_det.env_state * 10.0f));
            p = Wave_Hex16(p, (int16_t)(ImpactValue(wave_imax) * 10.0f));   /* 50 roots/s, not 400 */
            *p++ = '\r'; *p++ = '\n'; *p = '\0';
            UART1_Send_Data(line);
        }
        wave_rmax = 0.0f; wave_imax = 0.0f;
    }
#endif
    if (++decim_counter >= DECIM_FACTOR) decim_counter = 0;

    if (g_pending && (int32_t)(now - g_pending_deadline) >= 0) AssembleAndSendShot();

#if ENV_DEBUG
    if (impact_det.env_state > dbg_i) dbg_i = impact_det.env_state;
    if (chest_det.env_state  > dbg_c) dbg_c = chest_det.env_state;
    if (wrist_det.env_state  > dbg_w) dbg_w = wrist_det.env_state;
    if (-dbg_trunk_f > dbg_t) dbg_t = -dbg_trunk_f;

    if (!g_streaming) {                 /* do not accumulate while streaming is off */
        dbg_rmin = 1e9f; dbg_rmax = 0.0f; dbg_i = 0.0f; dbg_c = 0.0f; dbg_w = 0.0f; dbg_t = 0.0f;
    } else if ((int32_t)(now - dbg_next_ms) >= 0) {
        dbg_next_ms = now + 1000;
        LB_Str("ENV;");
        SendField_I32("Rmin", 1, (int32_t)(dbg_rmin * 10.0f));
        SendField_I32("Rmax", 1, (int32_t)(dbg_rmax * 10.0f));
        SendField_I32("I", 1, (int32_t)(ImpactValue(dbg_i) * 10.0f));
        SendField_I32("C", 1, (int32_t)(dbg_c * 10.0f));
        SendField_I32("W", 1, (int32_t)(dbg_w * 10.0f));
        SendField_I32("T", 1, (int32_t)(dbg_t * 10.0f));
        LB_Str("\r\n");
        LB_Send(0);                 /* debug data: dropped first on a slow link */
        dbg_rmin = 1e9f; dbg_rmax = 0.0f; dbg_i = 0.0f; dbg_c = 0.0f; dbg_w = 0.0f; dbg_t = 0.0f;
    }
#endif
}

/* ============================================================================
 * MCU LAYER (not compiled in the PC simulation)
 * ============================================================================ */
#ifndef HOST_SIM

#define LIS3DH_I2C_ADDR      0x18
#define CTRL_REG1            0x20
#define CTRL_REG3            0x22
#define CTRL_REG4            0x23

#define BUS_SEL0             P4SEL0
#define BUS_PINS             (BIT6 | BIT7)

#define LED1                 BIT0   // P3.0
#define BTN1                 BIT3   // P2.3
#define LED_ON()             do { P3OUT &= ~LED1; } while(0)
#define LED_OFF()            do { P3OUT |= LED1; } while(0)

#define ACC_INT1_PIN         BIT0   // LIS3DH INT1 on P2.0
#define BT_CTS_PIN           BIT1   // RN4678 CTS on P3.1
#define BT_BUFFER_FULL()     (P3IN & BT_CTS_PIN)

#define LONG_PRESS_MS        3000

/* CPU / SMCLK frequency. Everything time-related below is derived from these. */
#define SMCLK_HZ             24000000UL
#define CYCLES_PER_MS        (SMCLK_HZ / 1000UL)      /* 24000 */
#define CYCLES_PER_US        (SMCLK_HZ / 1000000UL)   /* 24 */

volatile uint8_t  flag_short_press = 0;
volatile uint8_t  flag_long_press = 0;
volatile uint32_t press_start_ms = 0;
volatile uint8_t  drdy_trigger = 0;
volatile uint8_t  g_button_monitoring = 0;

/* ---- software I2C ----
 * I2C_DELAY_CYCLES: 3 delays per bit, in CPU cycles (24 per microsecond now).
 * 48 cycles = 2 us, about the same real time as the 12 cycles that worked at 8 MHz.
 * Expected 6-byte read time: ~0.75 ms (see i2c_avg_us in the PERF line).
 * If the ENV line shows garbage at rest (Rmin/Rmax far from ~98), the bus rise time is
 * too slow for this delay: raise it (72, 96...). */
#define I2C_DELAY_CYCLES 48
#define I2C_DELAY() __delay_cycles(I2C_DELAY_CYCLES)
#define SDA_HIGH() do { P4DIR &= ~BIT6; P4OUT |= BIT6; P4REN |= BIT6; } while(0)
#define SDA_LOW()  do { P4REN &= ~BIT6; P4OUT &= ~BIT6; P4DIR |= BIT6; } while(0)
#define SCL_HIGH() do { P4DIR &= ~BIT7; P4OUT |= BIT7; P4REN |= BIT7; } while(0)
#define SCL_LOW()  do { P4REN &= ~BIT7; P4OUT &= ~BIT7; P4DIR |= BIT7; } while(0)
#define SDA_READ() ((P4IN & BIT6) ? 1 : 0)

void I2C_SW_Start(void) { SDA_HIGH(); I2C_DELAY(); SCL_HIGH(); I2C_DELAY(); SDA_LOW(); I2C_DELAY(); SCL_LOW(); I2C_DELAY(); }
void I2C_SW_Stop(void)  { SDA_LOW(); I2C_DELAY(); SCL_HIGH(); I2C_DELAY(); SDA_HIGH(); I2C_DELAY(); }

uint8_t I2C_SW_WriteByte(uint8_t data) {
    uint8_t i, ack;
    for (i = 0; i < 8; i++) {
        if (data & 0x80) SDA_HIGH(); else SDA_LOW();
        I2C_DELAY(); SCL_HIGH(); I2C_DELAY(); SCL_LOW(); I2C_DELAY();
        data <<= 1;
    }
    SDA_HIGH(); I2C_DELAY(); SCL_HIGH(); I2C_DELAY();
    ack = !(SDA_READ());
    SCL_LOW(); I2C_DELAY();
    return ack;
}

uint8_t I2C_SW_ReadByte(uint8_t send_ack) {
    uint8_t i, data = 0;
    SDA_HIGH();
    for (i = 0; i < 8; i++) {
        data <<= 1;
        I2C_DELAY(); SCL_HIGH(); I2C_DELAY();
        if (SDA_READ()) data |= 0x01;
        SCL_LOW(); I2C_DELAY();
    }
    if (send_ack) SDA_LOW(); else SDA_HIGH();
    I2C_DELAY(); SCL_HIGH(); I2C_DELAY(); SCL_LOW(); I2C_DELAY();
    SDA_HIGH();
    return data;
}

void I2C_WriteReg(uint8_t reg, uint8_t data) {
    I2C_SW_Start();
    I2C_SW_WriteByte(LIS3DH_I2C_ADDR << 1);
    I2C_SW_WriteByte(reg);
    I2C_SW_WriteByte(data);
    I2C_SW_Stop();
    __delay_cycles(125UL * CYCLES_PER_US);
}

void I2C_ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t count) {
    uint8_t i;
    I2C_SW_Start();
    I2C_SW_WriteByte(LIS3DH_I2C_ADDR << 1);
    I2C_SW_WriteByte((count > 1) ? (reg | 0x80) : reg);
    I2C_SW_Start();
    I2C_SW_WriteByte((LIS3DH_I2C_ADDR << 1) | 0x01);
    for (i = 0; i < count; i++) buffer[i] = I2C_SW_ReadByte(i < (count - 1));
    I2C_SW_Stop();
}

void Bus_Switch_To_I2C(void) { BUS_SEL0 &= ~BUS_PINS; }

/* ---- timers / GPIO / interrupts ---- */
void TimerB0_Init_1ms(void) {
    TB0CCR0 = (uint16_t)(CYCLES_PER_MS - 1);   /* 1 ms tick */
    TB0CCTL0 = CCIE;
    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;
}

#pragma vector = TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void) {
    ms_tick++;
    /* Restart the UART TX interrupt every 1 ms after a CTS pause (item 21). Before, only the
     * main loop restarted it, up to ~2 ms late, which cut the Bluetooth throughput. */
    if (tx_tail != tx_head && !BT_BUFFER_FULL()) UCA1IE |= UCTXIE;
}

/* Free-running time in SMCLK cycles (24 per us), wraps every ~179 s; use differences only. */
static uint32_t PerfNowCycles(void) {
    uint32_t m1, m2; uint16_t r; uint16_t pend;
    do { m1 = ms_tick; r = TB0R; pend = TB0CCTL0 & CCIFG; m2 = ms_tick; } while (m1 != m2);
    /* The timer wrapped but its interrupt has not run yet (interrupt latency): count that ms,
     * otherwise time goes back 1 ms and a difference becomes ~2^32 cycles. */
    if (pend && r < (uint16_t)(CYCLES_PER_MS / 2)) m1++;
    return m1 * CYCLES_PER_MS + r;
}

void GPIO_Init(void) {
    P2DIR &= ~BTN1; P2REN |= BTN1; P2OUT |= BTN1;
    P2IES |= BTN1; P2IFG &= ~BTN1; P2IE |= BTN1;

    P2DIR &= ~ACC_INT1_PIN;
    P2IES &= ~ACC_INT1_PIN;
    P2IFG &= ~ACC_INT1_PIN;
    P2IE  |=  ACC_INT1_PIN;

    P3DIR &= ~BT_CTS_PIN;
    P3REN |= BT_CTS_PIN;
    P3OUT &= ~BT_CTS_PIN;
}

#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void) {
    if (P2IFG & BTN1) {
        if (!g_button_monitoring && !(P2IN & BTN1)) {
            press_start_ms = ms_tick;
            g_button_monitoring = 1;
        }
        P2IFG &= ~BTN1;
    }
    if (P2IFG & ACC_INT1_PIN) {
        drdy_trigger = 1;
        P2IFG &= ~ACC_INT1_PIN;
    }
}

void UART1_Init(void) {
    P4SEL0 |= BIT2 | BIT3; P4SEL1 &= ~(BIT2 | BIT3);
    UCA1CTLW0 = UCSWRST;
    UCA1CTLW0 |= UCSSEL__SMCLK;
    UCA1BR0 = 13; UCA1BR1 = 0; UCA1MCTLW = 0x2501;  /* 115200 baud @ 24 MHz (UCOS16, UCBRF=0, UCBRS=0x25) */
    UCA1CTLW0 &= ~UCSWRST;                          /* UCTXIFG = 1 from here: TX buffer empty */
}

/* ---- interrupt-driven TX ----
 * The interrupt fires while UCTXIE is on and the TX buffer is empty (UCTXIFG). It sends one
 * byte per interrupt, so the UART keeps running during I2C reads and filter maths.
 * When the queue is empty or the RN4678 says its buffer is full (CTS high), the interrupt
 * turns itself off WITHOUT clearing UCTXIFG; UART_Kick() turns it back on, and because
 * UCTXIFG is still set it fires immediately. UCA1IV is never read on purpose: reading it
 * would clear UCTXIFG and the next kick would never fire. Only TX interrupts are used on
 * USCI_A1 (no RX), so this vector only ever means "TX buffer empty". */
static void UART_Kick(void) {
    if (tx_tail != tx_head && !BT_BUFFER_FULL()) UCA1IE |= UCTXIE;
}

#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
    if (!(UCA1IFG & UCTXIFG)) return;
    if (tx_tail == tx_head || BT_BUFFER_FULL()) {
        UCA1IE &= ~UCTXIE;              /* main loop restarts it with UART_Kick() */
        return;
    }
    UCA1TXBUF = (uint8_t)tx_buf[tx_tail];
    tx_tail = (uint16_t)((tx_tail + 1) & (TX_BUF_SIZE - 1));
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;

    /* 24 MHz. FRAM needs 2 wait states above 16 MHz: this MUST be set before raising the clock. */
    FRCTL0 = FRCTLPW | NWAITS_2;

    __bis_SR_register(SCG0);
    CSCTL3 = SELREF__REFOCLK;
    CSCTL0 = 0;
    CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_7;   /* 24 MHz range */
    CSCTL2 = FLLD_0 + 731;                                       /* 32768 Hz x 732 = 23.99 MHz */
    __delay_cycles(3);
    __bic_SR_register(SCG0);
    while (CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));
    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;

    P3DIR |= LED1;
    LED_OFF();

    /* The W25Q128 flash is unused now but still sits on the shared bus
     * (its DO pin is P4.7 = SCL in I2C mode). Keep it deselected: CS = P4.4 high. */
    P4DIR |= BIT4;
    P4OUT |= BIT4;

    PM5CTL0 &= ~LOCKLPM5;

    UART1_Init();
    GPIO_Init();
    TimerB0_Init_1ms();
    __enable_interrupt();

    Bus_Switch_To_I2C();
    __delay_cycles(100UL * CYCLES_PER_MS);     /* 100 ms: accelerometer power-up */

    I2C_WriteReg(0x21, 0x00);
    I2C_WriteReg(CTRL_REG3, 0x10);   /* DRDY on INT1 */
    I2C_WriteReg(CTRL_REG4, 0xB8);   /* BDU, 16G, HIGH-RES (12-bit: the RAW stream relies on it) */
    I2C_WriteReg(CTRL_REG1, 0x77);   /* 400 Hz ODR, X/Y/Z enabled (0x67 would be 200 Hz) */

    {
        uint8_t dummy[6];
        I2C_ReadRegs(0x28, dummy, 6);   /* clear stale DRDY */
    }

    Kinematic_Reset();
    Perf_Reset();
    UART1_Send_String("BOOT_READY\r\n");

    while (1) {
        UART_Kick();                    /* restarts TX after a CTS pause */

        /* --- button press duration --- */
        if (g_button_monitoring) {
            uint32_t duration = GetMs() - press_start_ms;
            if (!(P2IN & BTN1)) {
                if (duration >= LONG_PRESS_MS) {
                    g_button_monitoring = 0;
                    flag_long_press = 1;
                    while (!(P2IN & BTN1));
                    __delay_cycles(50UL * CYCLES_PER_MS);      /* 50 ms debounce */
                }
            } else {
                g_button_monitoring = 0;
                if (duration >= 50) flag_short_press = 1;
            }
        }

        if (flag_long_press) {
            flag_long_press = 0;
            Kinematic_Reset();
            Perf_Reset();
            UART1_Send_String("RESET\r\n");
        }

        if (flag_short_press) {
            flag_short_press = 0;
            g_streaming = !g_streaming;
            if (g_streaming) {
                LED_ON();
                Perf_Reset();
                tx_drops = 0;
                tx_key_drops = 0;
                UART1_Send_String("STREAM_START\r\n");
            } else {
                LED_OFF();
                UART1_Send_String("STREAM_STOP\r\n");
                Perf_Report();
            }
        }

        /* --- one accelerometer sample --- */
        if (drdy_trigger || (P2IN & ACC_INT1_PIN)) {
            uint8_t raw[6] = {0};
            uint32_t t0, t1, t2;
            int16_t x_raw, y_raw, z_raw;

            drdy_trigger = 0;

            t0 = PerfNowCycles();
            I2C_ReadRegs(0x28, raw, 6);
            t1 = PerfNowCycles();

            x_raw = (int16_t)((raw[1] << 8) | raw[0]);
            y_raw = (int16_t)((raw[3] << 8) | raw[2]);
            z_raw = (int16_t)((raw[5] << 8) | raw[4]);

            Kinematic_ProcessSample(x_raw, y_raw, z_raw);
            t2 = PerfNowCycles();

            Perf_Add((t1 - t0) / CYCLES_PER_US, (t2 - t1) / CYCLES_PER_US);     /* cycles -> us */
            if (P2IN & ACC_INT1_PIN) perf.late++;          /* next sample already waiting = over budget */
        }
    }
}

#endif /* HOST_SIM */