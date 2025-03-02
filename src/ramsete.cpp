#include "ramsete.h"
#include "main.h"


// Transformation function to convert global error to local error
std::pair<double, double> transformError(double xError, double yError, double thetaActual) {
    double transformedX = xError * cos(thetaActual) + yError * sin(thetaActual);
    double transformedY = -xError * sin(thetaActual) + yError * cos(thetaActual);
    return {transformedX, transformedY};
}

// Sinc function (sin(x) / x)
double sinc(double x) {
    if (std::abs(x) < 1e-6) return 1.0;
    return sin(x) / x;
}

// Calculate function
std::pair<double, double> calculate(
    const lemlib::Pose current,
    const lemlib::Pose desired,
    double vd,
    double wd
) {
    double xError = desired.x - current.x;
    double yError = desired.y - current.y;
    double thetaError = desired.theta - current.theta;

    thetaError = std::fmod(thetaError + 180, 360);
    if (thetaError < 0) thetaError += 360;
    thetaError -= 180;

    auto [ex, ey] = transformError(xError, yError, current.theta);
    double etheta = thetaError;

    if (std::abs(vd) < 1e-6 && std::abs(wd) < 1e-6) {
        constexpr double smallScalar = 0.01;
        vd = smallScalar * ex;
        wd = smallScalar * etheta;
    }

    double k = 2 * zeta * sqrt(wd * wd + b * vd * vd);
    double v = vd * cos(etheta) + k * ex;
    double w;

    if (std::abs(etheta) < 1e-6) {
        w = wd + k * etheta;
    } else {
        w = wd + k * etheta + (b * vd * sinc(etheta) * ey) / etheta;
    }

    double linearMotorVelocity = v / (M_PI * lemlib::Omniwheel::OLD_4);
    double leftVelocity = linearMotorVelocity + w;
    double rightVelocity = linearMotorVelocity - w;

    return {leftVelocity, rightVelocity};
}
