#pragma once


#include <vector>
#include "optimize_path.h"


namespace bmpf {
/**
 * @brief прототип оптимизатора путей методом Ньютона
 * он подразмуевает такой же поочердёный проход по всем точкам пути,
 * кроме первой и последней. Но при этом новая точка смещается до тех
 * пор, пока не будет найдено оптимальное положение промежуточной
 * вершины. Он может быть полезен в случаях, когда коллизии
 * очень разнородны, либо шаг планирования очень мал.
 * Это связано с тем, что такие ситуации будут часто
 * не допускать прямого усреднения координат. Такие координаты
 * часто будут соответствовать коллизиям
 */
    class NewtonPathOptimizer : public PathOptimizer {
    public:

        /**
         * Оптимизировать путь
         * @param path путь
         * @return оптимизированный путь
         */
        std::vector<std::vector<double>> optimizePath(std::vector<std::vector<double>> path) override;

        /**
         * Конструктор
         * @param pr планировщик пути
         * @param optimizeMethod название оптимизационного метода
         */
        explicit NewtonPathOptimizer(std::shared_ptr<bmpf::PathFinder> pr, const std::string &optimizeMethod);

    };
}
