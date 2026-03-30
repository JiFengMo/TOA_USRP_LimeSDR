#ifndef NR_GSCN_H
#define NR_GSCN_H

#include <stddef.h>
#include <stdint.h>

#include "common/error.h"

/*
 * FR1 < 3000 MHz global sync raster (TS 38.104), GSCN in [2, 7498].
 * SS_REF (Hz) = (1200*N + 50*M) kHz, N in [1,2499], M in {1,3,5}.
 */
#define NR_GSCN_FR1_LT_3G_MIN  (2u)
#define NR_GSCN_FR1_LT_3G_MAX  (7498u)

/*
 * TS 38.104 Table 5.4.3.3-1 (FR1): band n41 applicable SS raster, step = 3.
 * Case A (15 kHz SCS): GSCN 6246..6717
 * Case C (30 kHz SCS): GSCN 6252..6714
 */
#define NR_GSCN_N41_SCS15K_FIRST  (6246u)
#define NR_GSCN_N41_SCS15K_LAST   (6717u)
#define NR_GSCN_N41_SCS30K_FIRST  (6252u)
#define NR_GSCN_N41_SCS30K_LAST   (6714u)
#define NR_GSCN_SYNC_RASTER_STEP  (3u)

typedef enum {
  /* Enumerate every FR1 GSCN whose SS_REF falls in [ss_ref_lo_hz, ss_ref_hi_hz]. */
  NR_GSCN_PROFILE_FR1_FREQ_WINDOW = 0,
  /* Band n41 only, 38.104 sync raster, 30 kHz SCS (Case C). */
  NR_GSCN_PROFILE_TS38104_N41_SCS30K = 1,
  /* Band n41 only, 38.104 sync raster, 15 kHz SCS (Case A). */
  NR_GSCN_PROFILE_TS38104_N41_SCS15K = 2,
} nr_gscn_profile_t;

toa_error_t nr_gscn_fr1_lt_3ghz_to_ss_ref_hz(uint32_t gscn, double *hz_out);

toa_error_t nr_gscn_fr1_collect_in_freq_window(double fmin_hz,
                                               double fmax_hz,
                                               uint32_t stride,
                                               uint32_t *out,
                                               size_t out_cap,
                                               size_t *n_out);

toa_error_t nr_gscn_fr1_collect_range(uint32_t gscn_first,
                                      uint32_t gscn_last,
                                      uint32_t stride,
                                      uint32_t *out,
                                      size_t out_cap,
                                      size_t *n_out);

/*
 * Build candidate GSCN list for a profile.
 * - FR1_FREQ_WINDOW: same as nr_gscn_fr1_collect_in_freq_window(ss_ref_lo, ss_ref_hi, stride, ...).
 * - N41 profiles: walk 38.104 raster; if ss_ref_lo_hz < ss_ref_hi_hz, keep only SS_REF in that band;
 *   else keep full raster. Then subsample by list_stride (1 = all).
 */
toa_error_t nr_gscn_collect_for_profile(nr_gscn_profile_t profile,
                                        double ss_ref_lo_hz,
                                        double ss_ref_hi_hz,
                                        uint32_t list_stride,
                                        uint32_t *out,
                                        size_t out_cap,
                                        size_t *n_out);

/*
 * OAI-style: GSCN visible at one RX tuning 鈥� SS_REF inside [fc - bw/2, fc + bw/2].
 */
toa_error_t nr_gscn_visible_in_passband(double center_hz,
                                        double bandwidth_hz,
                                        const uint32_t *candidates,
                                        size_t n_candidates,
                                        uint32_t *out,
                                        size_t out_cap,
                                        size_t *n_out);

/*
 * Tile RX center frequencies across [cover_lo_hz, cover_hi_hz] with given BW and overlap (0..0.9).
 * Mimics scanning the band in chunks when instantaneous BW < band (OAI MR future work).
 */
toa_error_t nr_gscn_tile_centers_hz(double cover_lo_hz,
                                   double cover_hi_hz,
                                   double bw_hz,
                                   double overlap_frac,
                                   double *centers_out,
                                   size_t centers_cap,
                                   size_t *n_centers);

/* Snap GSCN index to nearest TS 38.104 n41 Case C raster point (6252..6714, step 3). */
toa_error_t nr_gscn_snap_to_n41_scs30k_raster(uint32_t g_in, uint32_t *g_out);

#endif
