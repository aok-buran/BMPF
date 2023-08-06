#pragma once

#include <Eigen/Core>
#include <utility>

#include "matrix_math.h"


namespace bmpf {
/**
 * Базовый класс для всех сочленений роботов
 */
    class Joint {
    public:
        /**
         * Конструктор
         * @param parentTransform матрица преобразования из СК родительского сочленения в СК текущего
         * @param linkTransform матрица преобразования из СК сочленения в СК следующего за ним звена
         * @param isFixed флаг, является ли сочленение фиксированным (без привода)
         * @param jointAngle текущий угол поворота звена
         * @param axis ось вращения
         * @param isVirtual флаг, что сочленение является виртуальным и не связано со звеном
         */
        Joint(Eigen::Matrix4d parentTransform, Eigen::Matrix4d linkTransform,
              bool isFixed, double jointAngle, Eigen::Vector3d axis, bool isVirtual)
                : parentTransform(std::move(parentTransform)), linkTransform(std::move(linkTransform)),
                  isFixed(isFixed), jointAngle(jointAngle), axis(std::move(axis)), isVirtual(isVirtual) {}

        /**
         * Получить матрицу преобразования сочленения
         * @return матрица преобразования сочленения
         */
        Eigen::Matrix4d getTransformMatrix() { return parentTransform * getRotMatrix4x4(axis, jointAngle); }

        /**
         * Получить первую производную матрицы преобразования сочленения
         * @return первая производную матрицы преобразования сочленения
         */
        Eigen::Matrix4d getDiffTransformMatrix() { return parentTransform * getDiffRotMatrix4x4(axis, jointAngle); }

        /**
         * Получить вторую производную матрицы преобразования сочленения
         * @return вторая производную матрицы преобразования сочленения
         */
        Eigen::Matrix4d getDiff2TransformMatrix() { return parentTransform * getDiff2RotMatrix4x4(axis, jointAngle); }

        /**
         * получить ось вращения сочленения
         * @return ось вращения сочленения
         */
        Eigen::Vector3d getAxis() const { return axis; }

        /**
         * Флаг, является ли сочленение фиксированным (без привода)
         */
        bool isFixed;
        /**
         * Текущий угол поворота звена
         */
        double jointAngle;
        /**
         * Ось вращения
         */
        Eigen::Vector3d axis;
        /**
         * Флаг, что сочленение является виртуальным и не связано со звеном
         */
        bool isVirtual;
        /**
         * Матрица преобразования из СК родительского сочленения в СК текущего
         */
        Eigen::Matrix4d parentTransform;
        /**
         * Матрица преобразования из СК сочленения в СК следующего за ним звена
         */
        Eigen::Matrix4d linkTransform;
    };

}