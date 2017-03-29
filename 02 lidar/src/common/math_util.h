#ifndef _MATHUTIL_H
#define _MATHUTIL_H

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include "origin_vehicle.h"

#ifndef PI
#define PI 3.14159265358979323846264338
#endif

#define to_radians(x) ( (x) * (PI / 180.0 ))
#define to_degrees(x) ( (x) * (180.0 / PI ))

#define TWOPI_INV (0.5/PI)
#define TWOPI (2*PI)

static inline double sq(double v)
{
    return v*v;
}

static inline double sgn(double v)
{
    return (v>=0) ? 1 : -1;
}

// random number between [0, 1)
static inline float randf()
{
    return ((float) rand()) / (RAND_MAX + 1.0);
}

static inline float signed_randf()
{
    return randf()*2 - 1;
}

// return a random integer between [0, bound)
static inline int irand(int bound)
{
    int v = (int) (randf()*bound);
    assert(v >= 0);
    assert(v < bound);
    return v;
}

// valid only for v > 0
static inline double mod2pi_positive(double vin)
{
    double q = vin * TWOPI_INV + 0.5;
    int qi = (int) q;

    return vin - qi*TWOPI;
}

// Map v to [-PI, PI]
static inline double mod2pi(double vin)
{
    if (vin < 0)
        return -mod2pi_positive(-vin);
    else
        return mod2pi_positive(vin);
}

// Return vin such that it is within PI degrees of ref
static inline double mod2pi_ref(double ref, double vin)
{
    return ref + mod2pi(vin - ref);
}

static inline int theta_to_int(double theta, int max)
{
    theta = mod2pi_ref(PI, theta);
    int v = (int) (theta / ( 2 * PI ) * max);

    if (v == max)
        v = 0;

    assert (v >= 0 && v < max);

    return v;
}

static inline int imin(int a, int b)
{
    return (a < b) ? a : b;
}

static inline int imax(int a, int b)
{
    return (a > b) ? a : b;
}

static inline int64_t imin64(int64_t a, int64_t b)
{
    return (a < b) ? a : b;
}

static inline int64_t imax64(int64_t a, int64_t b)
{
    return (a > b) ? a : b;
}

static inline int iclamp(int v, int minv, int maxv)
{
    return imax(minv, imin(v, maxv));
}

static inline double fclamp(double v, double minv, double maxv)
{
    return fmax(minv, fmin(v, maxv));
}

static inline double fclamp_360(double v)
{
    if(v > 360.0)
        v -= 360.0;
    else if(v < 0.0)
        v += 360.0;

    return v;
}
// TODO
//static double calculate_vertical_direction(double current_head){
//    double vertical_direction;
//    vertical_direction = current_head + 90;
//
//    if(vertical_direction > 360)
//       vertical_direction -=360;
//    else if(vertical_direction < 0)
//       vertical_direction +=360;
//
//    return vertical_direction;
//}

#endif
