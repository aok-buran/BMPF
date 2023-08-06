#include "optimize_path.h"

using namespace bmpf;

/**
 * Конструктор
 * @param pf планировщик пути
 * @param optimizeMethod название оптимизационного метода
 */
PathOptimizer::PathOptimizer(std::shared_ptr<bmpf::PathFinder> &pf, const std::string &optimizeMethod) {
    _pf = pf;
    _optimizeMethod = optimizeMethod;
}
