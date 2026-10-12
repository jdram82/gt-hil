/*
 * sensor_vote.h - redundant-sensor handling.
 *
 * Speed: three channels, median voting plus two diagnostics
 *   - deviation: a channel more than HIL_SPEED_DEV_RPM from the median for
 *     HIL_SPEED_DEV_TIME_S is flagged bad (catches drift, offset, noise);
 *   - flat-line: a turning machine whose channel repeats the exact same value
 *     for HIL_SPEED_FLAT_TIME_S is flagged bad (catches a stuck sensor that
 *     happens to sit on the right value).
 * Bad flags are latched until speedvote_clear().
 * With two good channels left the voted value is their mean; if they then
 * disagree, or fewer than two remain, `fault` is set (cannot vote any more).
 *
 * Temperature: median of four thermocouples for control, spread = max - min.
 *
 * Pure C, no hardware calls.
 */
#ifndef SENSOR_VOTE_H
#define SENSOR_VOTE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* configuration */
    int    use_vote;          /* 0 = use channel 1 only (negative tests)  */
    int    use_diag;          /* 0 = no deviation / flat-line diagnostics */
    int    use_flat;          /* 0 = skip flat-line test (noise-free sim) */

    /* latched diagnostics */
    int    bad[3];
    int    disagree;
    int    n_good;
    int    fault;             /* cannot vote: <2 good, or 2 good disagree */
    double voted_rpm;

    /* internal */
    double dev_t[3], flat_t[3], dis_t;
    double prev[3];
    int    prev_valid;
} SpeedVote;

void speedvote_init(SpeedVote *v);
void speedvote_clear(SpeedVote *v);                       /* keeps configuration */
void speedvote_update(SpeedVote *v, const double x[3], double dt);

double tc_median4(const double t[4]);
double tc_spread4(const double t[4]);

#ifdef __cplusplus
}
#endif
#endif
