#include "base/joint_params.h"
#include "log.h"

using namespace bmpf;

/**
 * Получить строковое представление сочленения
 * @return строковое представление сочленения
 */
std::string JointParams::toString() const {
    char buf[256];
    sprintf(buf, "maxAcceleration=%.1f ", maxAcceleration);
    std::string msg = buf;
    sprintf(buf, "maxVelocity=%.1f ", maxVelocity);
    msg += buf;
    sprintf(buf, "maxAngle=%.3f ", maxAngle);
    msg += buf;
    sprintf(buf, "minAngle=%.1f ", minAngle);
    msg += buf;
    sprintf(buf, "jointNum=%lu ", jointNum);
    msg += buf;
    return msg;
}

/**
 * Получить случайный доступный угол
 * @return случайный доступный угол
 */
double JointParams::getRandomAngle() const {
    double f = (double) rand() / RAND_MAX;
    return minAngle + f * (maxAngle - minAngle);
}
