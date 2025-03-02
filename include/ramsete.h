#ifndef RAMSETE_H
#define RAMSETE_H

#include "main.h"

// Declare variables as extern
extern double b;
extern double zeta;

// Function declarations
void initRamsete(double b_val, double zeta_val);
std::pair<double, double> transformError(double xError, double yError, double thetaActual);
double sinc(double x);
std::pair<double, double> calculate(
    const lemlib::Pose current, 
    const lemlib::Pose desired, 
    double vd, 
    double wd
);

#endif // RAMSETE_H