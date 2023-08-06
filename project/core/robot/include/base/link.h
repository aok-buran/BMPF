#pragma once

#include <string>
#include <Eigen/Core>
#include <utility>
#include <vector>
#include <memory>

namespace bmpf {
    /**
     * Звено робота
     */
    class Link {
    public:
        /**
         * Путь к модели
         */
        std::string modelPath;
        /**
         * Название
         */
        std::string name;
        /**
         * Матрица преобразования из СК мира в СК звена для неирерархичных звеньев
         */
        std::shared_ptr<Eigen::Matrix4d> linkTransformMatrix;
        /**
         * Тензор инерции звена
         */
        Eigen::Matrix3d inertia;
        /**
         * Координаты центра масс
         */
        std::vector<double> massCenter;

        /**
         * Конструктор звена
         * @param model_path путь к модели
         * @param linkTransformMatrix  матрица преобразования из СК мира в СК звена для неирерархичных звеньев
         * @param name название
         * @param inertia тензор инерции звена
         * @param massCenter координаты центра масс
         */
        Link(std::string model_path, std::shared_ptr<Eigen::Matrix4d> linkTransformMatrix,
             std::string name, Eigen::Matrix3d inertia, std::vector<double> massCenter
        ) : modelPath(std::move(model_path)), linkTransformMatrix(std::move(linkTransformMatrix)),
            name(std::move(name)), inertia(std::move(inertia)), massCenter(std::move(massCenter)) {}

        /**
         * Получить строковое представление звена
         * @return Строковое представление звена
         */
        std::string toString() const;
    };
}
