#include "optimize_path_newton.h"

using namespace bmpf;

/**
 * Оптимизировать путь
 * @param path путь
 * @return оптимизированный путь
 */
std::vector<std::vector<double>> NewtonPathOptimizer::optimizePath(std::vector<std::vector<double>> path) {
    return path;
}

/**
 * Конструктор
 * @param pr планировщик пути
 * @param optimizeMethod название оптимизационного метода
 */
NewtonPathOptimizer::NewtonPathOptimizer(
        std::shared_ptr<bmpf::PathFinder> pf, const std::string &optimizeMethod
) : PathOptimizer(pf, optimizeMethod) {

}
