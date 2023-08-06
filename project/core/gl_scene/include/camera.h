#pragma once

#include <Eigen/Dense>
#include <GL/gl.h>
#include <GL/glut.h>

#include "matrix_math.h"
#include "log.h"

namespace bmpf {

    /**
     * @brief Класс камеры
     *
     * камера определяется тремя векторами: pos - положение камеры
     * dir - направление камеры, up - вектор "вверх"
     */
    class Camera {
    public:
        /**
         * Конструктор
         */
        Camera();

        /**
         * Конструктор камеры
         * @param _pos положение камеры
         * @param _dir направление камеры
         * @param _up вектор "вверх"
         */
        Camera(Eigen::Vector3d _pos, Eigen::Vector3d _dir, Eigen::Vector3d _up) :
                _pos(std::move(_pos)), _dir(std::move(_dir)), _up(std::move(_up)) {};

        /**
         * Получить вектор "вверх"
         * @return вектор "вверх"
         */
        Eigen::Vector3d getUp() { return _up; };

        /**
         * Получить положение камеры
         * @return положение камеры
         */
        Eigen::Vector3d getCenter() { return _pos; };

        /**
         * получить точку, на которую смотрит камера
         * @return точка, на которую смотрит камера
         */
        Eigen::Vector3d getEye() { return _dir + _pos; }

        /**
         * поворот камеры относительно оси OX экрана
         * @param alpha угол
         */
        void rotateX(double alpha);

        /**
         * поворот камеры относительно оси OY экрана
         * @param alpha угол
         */
        void rotateY(double alpha);

        /**
         * переместить камеру влево
         * @param d расстояние
         */
        void moveLeft(double d);

        /**
         * переместить камеру вправо
         * @param d расстояние
         */
        void moveRight(double d);

        /**
         * переместить камеру вперёд
         * @param d расстояние
         */
        void moveForward(double d) { _pos = _pos - _dir * d; }

        /**
         * переместить камеру назад
         * @param d расстояние
         */
        void moveBack(double d) { _pos = _pos + _dir * d; }

        /**
         * переместить камеру вниз
         * @param d расстояние
         */
        void moveDown(double d) { _pos(2) -= d; }

        /**
         * переместить камеру вверх
         * @param d расстояние
         */
        void moveUp(double d) { _pos(2) += d; }

        /**
         * применить параметры камеры к OpenGL
         */
        void applyLookAt();

    private:
        /**
         * вектор "вверх"
         */
        Eigen::Vector3d _up;
        /**
         * направление камеры
         */
        Eigen::Vector3d _dir;
        /**
         * положение камеры
         */
        Eigen::Vector3d _pos;
    };


}