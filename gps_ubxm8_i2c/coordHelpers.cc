#include "coordHelpers.h"

#include <cmath>

const double a = 6378137.0;               // WGS-84 semi-major axis
const double e2 = 6.6943799901377997e-3;  // WGS-84 first eccentricity squared
const double a1 = 4.2697672707157535e+4;  // a1 = a*e2
const double a2 = 1.8230912546075455e+9;  // a2 = a1*a1
const double a3 = 1.4291722289812413e+2;  // a3 = a1*e2/2
const double a4 = 4.5577281365188637e+9;  // a4 = 2.5*a2
const double a5 = 4.2840589930055659e+4;  // a5 = a1+a3
const double a6 = 9.9330562000986220e-1;  // a6 = 1-e2

static double ecef[3];
// Convert Lat, Lon, Altitude to Earth-Centered-Earth-Fixed (ECEF)
// Input is a three element array containing lat, lon (rads) and alt (m)
// Returned array contains x, y, z in meters
double* geo_to_ecef(double lon, double lat, double alt) {
    double n = a / sqrt(1 - e2 * sin(lat) * sin(lat));
    ecef[0] = (n + alt) * cos(lat) * cos(lon);  // ECEF x
    ecef[1] = (n + alt) * cos(lat) * sin(lon);  // ECEF y
    ecef[2] = (n * (1 - e2) + alt) * sin(lat);  // ECEF z
    return (ecef);                              // Return x, y, z in ECEF
}
