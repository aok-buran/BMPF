#include "camera.h"

using namespace bmpf;

/**
 * Конструктор
 */
Camera::Camera() {
    _dir << 1, 0, 0;
    _pos << 0, 0, 0;
    _up << 0, 0, 1;
}

/**
 * поворот камеры относительно оси OX экрана
 * @param alpha угол
 */
void Camera::rotateX(double alpha) {
    // получаем матрицу поворота относительно вектора _up
    Eigen::Matrix3d R = getRotMatrix3x3(alpha, _up);
    // поворачиваем вектор _dir
    _dir = R * _dir;

    // получаем матрицу поворота относительно оси z
    Eigen::Matrix3d R_up = getRotMatrix3x3(alpha, Eigen::Vector3d(0, 0, 1));
    // поворачиваем вектор _up
    _up = R_up * _up;
}

/**
 * поворот камеры относительно оси OY экрана
 * @param alpha угол
 */
void Camera::rotateY(double alpha) {
    // если угол позволяет сделать поворот,
    // то получаем единичный вектор, перпендикулярный _dir и _up
    Eigen::Vector3d r = _up.cross(_dir).normalized();
    // получаем из него и угла alpha матрицу поворота
    Eigen::Matrix3d R = getRotMatrix3x3(alpha, r);
    // получаем новые значения векторов
    _up = R * _up;
    _dir = R * _dir;

}

/**
 * переместить камеру вправо
 * @param d расстояние
 */
void Camera::moveRight(double d) {
    Eigen::Vector3d r = _up.cross(_dir).normalized();
    _pos += r * d;
}

/**
 * переместить камеру влево
 * @param d расстояние
 */
void Camera::moveLeft(double d) {
    Eigen::Vector3d r = _up.cross(_dir).normalized();
    _pos -= r * d;
}

/**
 * применить параметры камеры к OpenGL
 */
void Camera::applyLookAt() {
    // получаем точку, на которую смотрит камера
    Eigen::Vector3d eye = getEye();
    // задаём параметры камеры OpenGL
    gluLookAt(
            eye(0), eye(1), eye(2),
            _pos(0), _pos(1), _pos(2),
            _up(0), _up(1), _up(2)
    );
}

