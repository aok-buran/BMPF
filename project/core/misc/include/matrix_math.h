#pragma once

#include <Eigen/Core>
#include <vector>

namespace bmpf {

    /**
     * получить матрицу трансформации по смещению
     * @param x смещение вдоль оси OX
     * @param y смещение вдоль оси OY
     * @param z смещение вдоль оси OZ
     * @return
     */
    static Eigen::Matrix4d getTranslationMatrix(double x, double y, double z) {
        Eigen::Matrix4d m;
        m << 1, 0, 0, x,
                0, 1, 0, y,
                0, 0, 1, z,
                0, 0, 0, 1;
        return m;    }

    /**
     * получить матрицу трансформации по смещению
     * @param translation вектор смещения
     * @return матрица трансформации
     */
    static Eigen::Matrix4d getTranslationMatrix(std::vector<double> translation) {
        return getTranslationMatrix(translation.at(0), translation.at(1), translation.at(2));
    }

    /**
     * получить матрицу поворота по углам Эйлера:
     * @param r крен
     * @param p тангаж
     * @param y рыскание
     * @return матрица поворота
     */
    static Eigen::Matrix4d getRotationMatrix(double r, double p, double y) {
        Eigen::Matrix4d m;
        double c1 = cos(r);
        double s1 = sin(r);
        double c2 = cos(p);
        double s2 = sin(p);
        double c3 = cos(y);
        double s3 = sin(y);
        m << c1 * c2, c1 * s2 * s3 - c3 * s1, s1 * s3 + c1 * c3 * s2, 0,
                c2 * s1, c1 * c3 + s1 * s2 * s3, c3 * s1 * s2 - c1 * s3, 0,
                -s2, c2 * s3, c2 * c3, 0,
                0, 0, 0, 1;
        return m;
    }

    /**
     * получить матрицу поворота по углам
     * @param rotation вектор (крен, тангаж и рыскание)
     * @return матрица поворота
     */
    static Eigen::Matrix4d getRotationMatrix(std::vector<double> rotation) {
        return getRotationMatrix(rotation.at(0), rotation.at(1), rotation.at(2));
    }

    /**
     * получить матрицу масштабирования
     * @param x масштабирование вдоль оси OX
     * @param y масштабирование вдоль оси OY
     * @param z масштабирование вдоль оси OZ
     * @return матрица масштабирования
     */
    static Eigen::Matrix4d getScaleMatrix(double x, double y, double z) {
        Eigen::Matrix4d m;
        m << x, 0, 0, 0,
                0, y, 0, 0,
                0, 0, z, 0,
                0, 0, 0, 1;
        return m;
    }

    /**
     * получить матрицу масштабирования
     * @param scale вектор масштабирования
     * @return матрица масштабирования
     */
    static Eigen::Matrix4d getScaleMatrix(std::vector<double> scale) {
        return getScaleMatrix(scale.at(0), scale.at(1), scale.at(2));
    }

    /**
     * получить операционный вектор (положение и ориентация в параметрах
    *  Родриго-Гамильтона) из матрицы преобразования
     * @param tf матрица преобразования
     * @return операционный вектор
     */
    static std::vector<double> getRGVector(Eigen::Matrix4d &tf) {
        double tr = tf.trace();

        double w = 1.0 / 2 * sqrt(tr);
        double x = 1.0 / (4 * w) * (tf(2, 1) - tf(1, 2));
        double y = 1.0 / (4 * w) * (tf(0, 2) - tf(2, 0));
        double z = 1.0 / (4 * w) * (tf(1, 0) - tf(0, 1));
        std::vector<double> pos{
                tf(0, 3), // STATE_POS_X
                tf(1, 3), // STATE_POS_Y
                tf(2, 3), // STATE_POS_Z
                w,        // STATE_ORIENT_W
                x,        // STATE_ORIENT_X
                y,        // STATE_ORIENT_Y
                z         // STATE_ORIENT_Z
        };
        return pos;
    }


    /**
     * Получить вектор положения из матрицы преобразования
     * @param tf матрица преобразования
     * @return вектор положения
     */
    static std::vector<double> getPosition(Eigen::Matrix4d tf) {
        std::vector<double> pos{
                tf(0, 3), // STATE_POS_X
                tf(1, 3), // STATE_POS_Y
                tf(2, 3), // STATE_POS_Z
        };
        return pos;
    }


    /**
     * получить ориентацию в параметрах Родриго-Гамильтона из матрицы преобразования
     * @param tf матрица преобразования
     * @return ориентация в параметрах Родриго-Гамильтона
     */
    static std::vector<double> getOrientation(Eigen::Matrix4d &tf) {
        double tr = tf.trace();

        double w = 1.0 / 2 * sqrt(tr);
        double x = 1.0 / (4 * w) * (tf(2, 1) - tf(1, 2));
        double y = 1.0 / (4 * w) * (tf(0, 2) - tf(2, 0));
        double z = 1.0 / (4 * w) * (tf(1, 0) - tf(0, 1));
        std::vector<double> pos{
                w,        // STATE_ORIENT_W
                x,        // STATE_ORIENT_X
                y,        // STATE_ORIENT_Y
                z         // STATE_ORIENT_Z
        };
        return pos;
    }

    /**
     * получить матрицу поворота вдоль оси на заданный угол
     * @param axis ось
     * @param theta угол
     * @return матрица поворота
     */
    static Eigen::Matrix4d getRotMatrix4x4(Eigen::Vector3d axis, double theta) {
        double x = axis(0);
        double y = axis(1);
        double z = axis(2);
        Eigen::Matrix4d rotM;

        rotM << cos(theta) + (1 - cos(theta)) * x * x,
                (1 - cos(theta)) * x * y - sin(theta) * z,
                (1 - cos(theta)) * x * z
                + sin(theta) * y,
                0,
                (1 - cos(theta)) * y * x + sin(theta) * z,
                cos(theta) + (1 - cos(theta)) * y * y,
                (1 - cos(theta)) * y * z
                - sin(theta) * x,
                0,
                (1 - cos(theta)) * z * x - sin(theta) * y,
                (1 - cos(theta)) * z * y + (sin(theta)) * x,
                cos(theta)
                + (1 - cos(theta)) *
                  z * z,
                0,
                0,
                0,
                0,
                1;
        return rotM;
    }

    /**
     * получить первую производную матрицы поворота вдоль оси на заданный угол
     * @param axis ось
     * @param theta угол
     * @return первая производная матрицы поворота
     */
    static Eigen::Matrix4d getDiffRotMatrix4x4(Eigen::Vector3d axis, double theta) {
        double x = axis(0);
        double y = axis(1);
        double z = axis(2);
        Eigen::Matrix4d rotM;

        rotM << sin(theta) * x * x - sin(theta),
                x * y * sin(theta) - z * cos(theta),
                y * cos(theta) + x * z * sin(theta),
                0,
                z * cos(theta) + x * y * sin(theta),
                sin(theta) * y * y - sin(theta),
                y * z * sin(theta) - x * cos(theta),
                0,
                x * z * sin(theta) - y * cos(theta),
                x * cos(theta) + y * z * sin(theta),
                sin(theta) * z * z - sin(theta),
                0,
                0,
                0,
                0,
                1;
        return rotM;
    }


    /**
     * получить вторую производную матрицы поворота вдоль оси на заданный угол
     * @param axis ось
     * @param theta угол
     * @return вторая производная матрицы поворота
     */
    static Eigen::Matrix4d getDiff2RotMatrix4x4(Eigen::Vector3d axis, double theta) {
        double x = axis(0);
        double y = axis(1);
        double z = axis(2);
        Eigen::Matrix4d rotM;

        rotM << 2 * x * sin(theta), y * sin(theta), z * sin(theta), 0,
                y * sin(theta), 0, -cos(theta), 0,
                z * sin(theta), cos(theta), 0, 0,
                0, 0, 0, 1;
        return rotM;
    }


    /**
     * получаем матрицу поворота 3х3
     * @param alpha  угол
     * @param v ось
     * @return
     */
    static Eigen::Matrix3d getRotMatrix3x3(double alpha, const Eigen::Vector3d &v) {
        double x = v(0);
        double y = v(1);
        double z = v(2);

        Eigen::Matrix3d R;
        R << cos(alpha) + (1 - cos(alpha)) * x * x,
                (1 - cos(alpha)) * x * y - (sin(alpha)) * z,
                (1 - cos(alpha)) * x * z + (sin(alpha)) * y,

                (1 - cos(alpha)) * y * x + (sin(alpha)) * z,
                cos(alpha) + (1 - cos(alpha)) * y * y,
                (1 - cos(alpha)) * y * z - (sin(alpha)) * x,

                (1 - cos(alpha)) * z * x - (sin(alpha)) * y,
                (1 - cos(alpha)) * z * y + (sin(alpha)) * x,
                cos(alpha) + (1 - cos(alpha)) * z * z;

        return R;
    }

}