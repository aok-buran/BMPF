#pragma once


#include <vector>
#include "optimize_path.h"


namespace bmpf {
    /**
     * Медианный оптимизатор пути
     */
    class MedianPathOptimizer : public PathOptimizer {
    public:

        /**
         * Оптимизировать путь
         * @param path путь
         * @return оптимизированный путь
         */
        std::vector<std::vector<double>> optimizePath(std::vector<std::vector<double>> path) override;


        /**
         * Конструктор
         * @param pf планировщик пути
         * @param optimizeMethod название метода оптимизации
         * @param checkCnt количество промежуточных проверок каждого этап
         * @param divideCnt количество делений каждого этапа (если divideCnt=0, то
         * проверка промежуточных состояний не выполняется)
         * @param optimizeLoopCnt
         */
        explicit MedianPathOptimizer(
                std::shared_ptr<bmpf::PathFinder> pf, const std::string &optimizeMethod, int checkCnt,
                int divideCnt, int optimizeLoopCnt
        );

    private:
        /**
         * количество промежуточных проверок каждого этап
         */
        int _checkCnt;
        /**
         * количество делений каждого этапа (если divideCnt=0, то
         * проверка промежуточных состояний не выполняется)
         */
        int _divideCnt;
        /**
         * Количество циклов оптимизации
         */
        int _optimizeLoopCnt;
    };
}

