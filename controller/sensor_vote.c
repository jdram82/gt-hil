#include "sensor_vote.h"
#include "../include/hil_constants.h"
#include <math.h>
#include <string.h>

static double med3(double a, double b, double c)
{
    if (a > b) { double t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return a > b ? a : b;
}

void speedvote_init(SpeedVote *v)
{
    memset(v, 0, sizeof *v);
    v->use_vote = 1;
    v->use_diag = 1;
    v->use_flat = 1;
    v->n_good   = 3;
}

void speedvote_clear(SpeedVote *v)
{
    int vote = v->use_vote, diag = v->use_diag, flat = v->use_flat;
    speedvote_init(v);
    v->use_vote = vote; v->use_diag = diag; v->use_flat = flat;
}

void speedvote_update(SpeedVote *v, const double x[3], double dt)
{
    const double m = med3(x[0], x[1], x[2]);
    int i;

    if (!v->use_vote) {              /* single channel, no protection */
        v->voted_rpm = x[0];
        v->n_good = 3; v->fault = 0;
        return;
    }

    if (v->use_diag) {
        int ng = 0;
        for (i = 0; i < 3; i++) if (!v->bad[i]) ng++;

        if (ng == 3) {
            for (i = 0; i < 3; i++) {
                v->dev_t[i] = (fabs(x[i] - m) > HIL_SPEED_DEV_RPM) ? v->dev_t[i] + dt : 0.0;
                if (v->dev_t[i] >= HIL_SPEED_DEV_TIME_S) v->bad[i] = 1;
            }
        } else if (ng == 2) {
            int a = -1, b = -1;
            for (i = 0; i < 3; i++) if (!v->bad[i]) { if (a < 0) a = i; else b = i; }
            v->dis_t = (fabs(x[a] - x[b]) > HIL_SPEED_DEV_RPM) ? v->dis_t + dt : 0.0;
            if (v->dis_t >= HIL_SPEED_DEV_TIME_S) v->disagree = 1;
        }

        if (v->use_flat && v->prev_valid && m > HIL_SPEED_FLAT_MIN_RPM) {
            for (i = 0; i < 3; i++) {
                if (v->bad[i]) continue;
                v->flat_t[i] = (x[i] == v->prev[i]) ? v->flat_t[i] + dt : 0.0;
                if (v->flat_t[i] >= HIL_SPEED_FLAT_TIME_S) v->bad[i] = 1;
            }
        }
        memcpy(v->prev, x, sizeof v->prev);
        v->prev_valid = 1;
    }

    {
        int ng = 0; double sum = 0.0;
        for (i = 0; i < 3; i++) if (!v->bad[i]) { ng++; sum += x[i]; }
        v->n_good = ng;
        if (ng == 3)      v->voted_rpm = m;
        else if (ng >= 1) v->voted_rpm = sum / ng;
        else              v->voted_rpm = m;
        v->fault = (ng < 2) || v->disagree;
    }
}

double tc_median4(const double t[4])
{
    double s[4]; int i, j;
    memcpy(s, t, sizeof s);
    for (i = 1; i < 4; i++) {
        double k = s[i];
        for (j = i - 1; j >= 0 && s[j] > k; j--) s[j + 1] = s[j];
        s[j + 1] = k;
    }
    return 0.5 * (s[1] + s[2]);
}

double tc_spread4(const double t[4])
{
    double lo = t[0], hi = t[0]; int i;
    for (i = 1; i < 4; i++) { if (t[i] < lo) lo = t[i]; if (t[i] > hi) hi = t[i]; }
    return hi - lo;
}
