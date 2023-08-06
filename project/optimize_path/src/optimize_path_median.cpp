#include "optimize_path_median.h"

using namespace bmpf;

/**
 * Оптимизировать путь
 * @param path путь
 * @return оптимизированный путь
 */
std::vector<std::vector<double>> MedianPathOptimizer::optimizePath(std::vector<std::vector<double>> path) {
    auto scene = _pf->getScene();

    auto dividedPath = bmpf::PathFinder::splitPath(path, _divideCnt);

    for (int j = 0; j < _optimizeLoopCnt; j++) {
        for (int i = 1; i < dividedPath.size() - 1; i++) {
            auto middle = bmpf::sumStates(dividedPath.at(i - 1), bmpf::mulState(
                    bmpf::subtractStates(dividedPath.at(i + 1), dividedPath.at(i - 1)), 0.5));

            if (_pf->checkState(middle)) {
                if (_checkCnt > 0)
                    if (_pf->divideCheckPathSegment(dividedPath.at(i - 1), middle, _checkCnt) &&
                        _pf->divideCheckPathSegment(dividedPath.at(i + 1), middle, _checkCnt)) {
                        dividedPath.at(i) = middle;
                    }
            }
        }
    }

    return dividedPath;
}


/**
 * Конструктор
 * @param pr планировщик пути
 * @param optimizeMethod название метода оптимизации
 * @param checkCnt количество промежуточных проверок каждого этап
 * @param divideCnt количество делений каждого этапа (если divideCnt=0, то
 * проверка промежуточных состояний не выполняется)
 * @param optimizeLoopCnt
 */
MedianPathOptimizer::MedianPathOptimizer(
        std::shared_ptr<bmpf::PathFinder> pf, const std::string &optimizeMethod, int checkCnt,
        int divideCnt, int optimizeLoopCnt
) : PathOptimizer(pf, optimizeMethod), _checkCnt(checkCnt), _divideCnt(divideCnt),
    _optimizeLoopCnt(optimizeLoopCnt) {

}
