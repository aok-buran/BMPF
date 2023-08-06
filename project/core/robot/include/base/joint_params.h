#pragma once


#include <string>
#include <Eigen/Core>

namespace bmpf {
/**
 * Параметры сочленения робота
 */
    struct JointParams {
        /**
         * Максимальное по модулю ускорение
         */
        double maxAcceleration;
        /**
         * Максимальная по модулю скорость
         */
        double maxVelocity;
        /**
         * Максимальный угол
         */
        double maxAngle;
        /**
         * Минимальный угол
         */
        double minAngle;
        /**
         * Индекс сочленения
         */
        unsigned long jointNum;

        /**
         * Конструктор
         * @param maxAcceleration максимальное по модулю ускорение
         * @param maxVelocity максимальная по модулю скорость
         * @param maxAngle максимальный угол
         * @param minAngle минимальный угол
         * @param jointNum индекс сочленения
         */
        JointParams(double maxAcceleration, double maxVelocity, double maxAngle, double minAngle,
                    unsigned long jointNum) :
                maxAcceleration(maxAcceleration), maxVelocity(maxVelocity), maxAngle(maxAngle),
                minAngle(minAngle), jointNum(jointNum) {}

        /**
         * Получить строковое представление сочленения
         * @return строковое представление сочленения
         */
        std::string toString() const;

        /**
         * Получить случайный доступный угол
         * @return случайный доступный угол
         */
        double getRandomAngle() const;
    };

}
