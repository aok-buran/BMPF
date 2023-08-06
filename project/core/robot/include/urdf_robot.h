#pragma once

#include <base/robot.h>
#include <Eigen/Dense>
#include <utility>
#include <vector>
#include <string>

#include "urdf/model.h"
#include "base/joint.h"


namespace bmpf {
    /**
     * @brief Класс робота с URDF параметрами
     * Работает только с неиерархичными объектами или если
     * иерархия не имеет ветвлений (классические роботы-манипуляторы)
     */
    class URDFRobot : public BaseRobot {
    public:

        /**
         * Конструктор
         */
        URDFRobot() = default;

        /**
         * @brief Загрузить параметры робота из файла
         *
         * Загрузить параметры робота из файла, нужно заполнить следующие поля:
         * _jointParams, _links, _nonHierarchicalLinks, path
         * @param path путь к файлу описания робота
         */
        void loadFromFile(std::string path) override;

    private:

        /**
         * Коэффициент преобразованаия максимального момента
         * в звене в максимальное ускорение
         */
        constexpr static const double EFFORT_TO_ACCELERATION = 0.01;

        /**
         * Заполнить по urdf параметры робота, jointList - список сочленений,
         * linkMap - словарь, ключу соответствет название звена,
         * значению - умный указатель на само звено
         * @param jointList
         * @param linkMap
         */
        void _applyURDF(std::vector<std::shared_ptr<urdf::Joint>> jointList,
                        std::map<std::string, urdf::LinkSharedPtr> linkMap);

        /**
         * Получить матрицу преобразования из положения urdf
         * @param pose положение
         * @return матрица преобразования
         */
        static Eigen::Matrix4d _getMatrixFromPose(urdf::Pose pose);

        /**
         * получить звено по urdf звену
         * @param urdfLink urdf звено
         * @param linkTransform матрица преобразования звена
         * @return звено
         */
        static std::shared_ptr<Link>
        _createLink(const std::shared_ptr<urdf::Link> &urdfLink, const Eigen::Matrix4d &linkTransform);

        /**
         * получить матрицу преобразования по urdf звену
         * @param urdfLink urdf звено
         * @return матрица преобразования
         */
        static Eigen::Matrix4d _createLinkTransform(const std::shared_ptr<urdf::Link> &urdfLink);
    };
}