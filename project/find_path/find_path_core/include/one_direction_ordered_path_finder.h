#pragma once

#include <memory>
#include <unordered_map>
#include <set>
#include <utility>
#include <unordered_set>
#include <base/grid_path_finder.h>
#include <future>
#include "scene.h"
#include "one_direction_path_finder.h"


namespace bmpf {
    /**
     * @brief Упорядоченный планировщик со смещениями вдоль одной из координат
     *
     * Упорядоченный планировщик со смещениями вдоль одной из координат (поворот
     * на каждом шаге ровно одного звена робота). Реализовано это с помощью списков смещений,
     * которые перебираются при каждом вызове `_forEachNeighbor`
     * Упорядоченность подразумевает, что робот двигается в первую очередь вдоль осей,
     * которые имеют наибольшую ошибку
     *
     * Суть планирования
     * на сетке заключается в том, что каждой вещественной
     * координате состояния мы ставим в соответствие целочисленную
     * координату из заданного диапазона.
     * В данном планировщике предполагается, что каждая координата
     * состояния разбивается на равное число точек (_gridSize)
     *
     * Планировщик основан на алгоритме A*, при этом в качестве ошибки
     * используется две величины: ошибка по углам поворота робота и ошибка
     * по положениям звеньев. Им соответствуют коэффициенты kG и kD
     *
     * Получается, что в данном планировщике одно и тоже
     * пространство конфигураций описывается двумя пространствами:
     * вещественным пространством состояний и целочисленным пространством
     * координат. При этом размерности пространств совпадают.
     *
     * Методы, реализованные в этом классе, позволяют переходить от
     * вещественного пространства к целочисленному и наоброт.
     *
     * Логика работы планировщика следующая:
     * 1) подготовка к тактам поиска пути
     * 2) выполнение тактов поиска пути с попутным заполнением тех или иных структур
     * 3) если путь найден из этих структур собирается путь и сохраняется в
     * переменную `_buildedPath`
     */
    class OneDirectionOrderedPathFinder : public OneDirectionPathFinder {
    public:
        /**
         * конструктор
         * @param scene сцена
         * @param showTrace флаг, нужно ли выводить информацию во время поиска пути
         * @param maxOpenSetSize максимальный размер открытого множества
         * @param gridSize размер сетки планирования
         * @param maxNodeCnt максимальное кол-во нод в закрытом множестве
         * @param kG коэффициент разницы в углах поворота сочленений робота
         * @param kD коэффициент разницы в положениях звеньев робота
         * @param threadCnt количество потоков планировщика
         */
        OneDirectionOrderedPathFinder(const std::shared_ptr<bmpf::Scene> &scene,
                                      bool showTrace,
                                      unsigned int maxOpenSetSize,
                                      int gridSize,
                                      unsigned int maxNodeCnt,
                                      unsigned int kG = 1,
                                      unsigned int kD = 0,
                                      int threadCnt = 1)
                : OneDirectionPathFinder(scene,
                                         showTrace,
                                         maxOpenSetSize,
                                         gridSize,
                                         maxNodeCnt,
                                         kG, kD, threadCnt) {};


    protected:

        /**
         * список смещений по отклонениям от цели (сцена из одного робота)
         * @param deltas отклонения
         * @return список смещений по отклонениям от цели
         */
        std::vector<unsigned long> _getSizeOrderedOffsetIndexesSingleRobot(std::vector<int> &deltas);

        /**
         * список смещений по отклонениям от цели (сцена из нескольких роботов)
         * @param deltas
         * @return список смещений по отклонениям от цели
         */
        std::vector<unsigned long> _getOrderedOffsetIndexesMultiRobot(std::vector<int> &deltas);

        /**
         * получить упорядоченные смещения по отклонению от цели (главный метод)
         * @param deltas отклонения
         * @return список смещений по отклонениям от цели
         */
        std::vector<unsigned long> _getOrderedOffsetIndexes(std::vector<int> &deltas);

        /**
         * для всех соседей текущей ноды метод должен добавить только
         * подходящих в множество _openSet, если один из соседей
         * имеет целевые указания (т.е. найден путь), то возвращаем указатель на эту ноду,
         * в противном случае должен быть возвращён nullptr
         * @param currentNode текущая нода
         * @param endCoords целевые координаты
         * @return найденная нода или null
         */
        std::shared_ptr<PathNode> _forEachNeighbor(
                std::shared_ptr<PathNode> currentNode,
                std::vector<int> &endCoords
        ) override;

    };

}