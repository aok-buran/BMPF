#pragma once


#include <vector>
#include <memory>
#include "base/path_finder.h"

namespace bmpf {
    /**
     * Оптимизатор пути
     */
    class PathOptimizer {
    public:
        /**
         * Конструктор
         * @param pf планировщик пути
         * @param optimizeMethod название оптимизационного метода
         */
        explicit PathOptimizer(std::shared_ptr<bmpf::PathFinder> &pf, const std::string &optimizeMethod);

        /**
         * Оптимизировать путь
         * @param path путь
         * @return оптимизированный путь
         */
        virtual std::vector<std::vector<double>> optimizePath(std::vector<std::vector<double>> path) = 0;

        /**
         * Получить планировщик пути
         * @return планировщик пути
         */
        std::shared_ptr<bmpf::PathFinder> getPathFinder() { return _pf; }

        /**
         * Получить название оптимизационного метода
         * @return
         */
        std::string getPathOptimizeMethod() { return _optimizeMethod; }

    protected:

        /**
         * Планировщик пути
         */
        std::shared_ptr<bmpf::PathFinder> _pf;
        /**
         * Название метода оптимизации
         */
        std::string _optimizeMethod;

    };

}