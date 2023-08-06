#pragma once

#include "base/robot.h"

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <json/json.h>

namespace bmpf {
    /**
     * Робот с параметрами Денавита-Хартенберга (ДХ)
     */
    class DHRobot : public BaseRobot {
    public:
        DHRobot() = default;

        /**
         * @brief загрузить параметры робота из файла
         *
         * Загрузить параметры робота из файла, нужно заполнить следующие поля:
         * _jointParams, _links, _nonHierarchicalLinks, path
         * @param path путь к файлу описания робота
         */
        void loadFromFile(std::string path) override;

        /**
         * Получить матрицу преобразования по параметрам ДХ
         * @param theta параметр theta
         * @param d параметр d
         * @param a параметр a
         * @param alpha параметр alpha
         * @return
         */
        static Eigen::Matrix4d getDHMatrix(double theta, double d, double a, double alpha);
    };
}