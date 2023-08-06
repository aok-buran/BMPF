#pragma once

#include <memory>
#include <unordered_map>
#include <set>
#include <utility>
#include <unordered_set>
#include <scene.h>
#include "path_finder.h"
#include "log.h"
#include "state.h"

#include <iostream>

namespace bmpf {
    /**
     * @brief Базовый класс для всех планировщиков на сетке
     * Базовый класс для всех планировщиков на сетке. Суть планирования
     * на сетке заключается в том, что каждой вещественной
     * координате состояния мы ставим в соответствие целочисленную
     * координату из заданного диапазона.
     * В данном планировщике предполагается, что каждая координата
     * состояния разбивается на равное число точек (_gridSize)
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
     *
     */
    class GridPathFinder : public PathFinder {
    public:

        /**
         * Ошибка планирования: не удалось найти стартовую точку на
         * сетке планирования
         */
        static const int ERROR_CAN_NOT_FIND_FREE_START_POINT = 2;
        /**
         * Ошибка планирования: не удалось найти конечную точку на
         * сетке планирования
         */
        static const int ERROR_CAN_NOT_FIND_FREE_END_POINT = 3;

        /**
         * конструктор
         * @param scene сцена
         * @param showTrace флаг, нужно ли выводить информацию во время поиска пути
         * @param gridSize размер сетки планирования
         * @param threadCnt количество потоков планировщика
         */
        GridPathFinder(
                const std::shared_ptr<bmpf::Scene> &scene, bool showTrace, int gridSize, int threadCnt = 1
        );

        /**
         * преобразование целочисленных координат в вещественное состояние
         * @param coords координаты
         * @return состояние
         */
        std::vector<double> coordsToState(std::vector<int> &coords) const;

        /**
         * преобразование целочисленных координат в вещественное состояние
         * @param coords координаты
         * @param robotNum  номер робота, координаты нужно преобразовать
         * @return состояние
         */
        std::vector<double> coordsToState(std::vector<int> coords, unsigned long robotNum);

        /**
         * Проверка доступности целочисленных координат для всей сцены
         * @param coords координаты
         * @return флаг, доступны ли целочисленных координаты
         */
        bool checkCoords(std::vector<int> coords);

        /**
         * поиск пути из состояния `startCoords` в состояние `endCoords`,
         *
         * @param startCoords начальные координаты
         * @param endCoords конечные координаты
         * @param errorCode  в эту переменную записывается код ошибки
         * @return путь по сетке планирования
         */
        std::vector<std::vector<double>>
        findGridPath(std::vector<int> &startCoords, std::vector<int> &endCoords, int &errorCode);

        /**
         * Проверка доступности целочисленных координат для робота с индексом `robotNum`
         * @param coords координаты
         * @param robotNum номер робота
         * @return  флаг, доступны ли целочисленных координаты
         */
        bool checkCoords(const std::vector<int> &coords, unsigned int robotNum);

        /**
         * преобразует состояние в координаты на сетке планирования,
         * если свободных координат не найдено, возвращает пустой список {}
         * @param state состояние
         * @return координаты
         */
        virtual std::vector<int> stateToCoords(std::vector<double> state);

        /**
         * подготовка к планированию
         * @param startState начальное состояние
         * @param endState конечное состояние
         */
        void prepare(const std::vector<double> &startState, const std::vector<double> &endState) override;

        /**
         * построить путь, построенный путь должен быть сохранён в переменную _buildedPath
         * @param startCoords начальные координаты
         * @param endCoords конечные координаты
         */
        virtual void prepare(std::vector<int> &startCoords, std::vector<int> &endCoords);

        /**
         * @brief проверка задания
         * проверка задания, возвращает пустой вектор, если маршрут можно строить
         * штатным образом, в противном случае сохраняет в _buildedPath готовый маршрут
         * (из стартового и конечного положений) и возвращает его
         * @param startState начальное состояние
         * @param endState конечное состояние
         * @param opacity точность проверки
         * @return пустой вектор, если маршрут можно строить, вектор из стартового и конечного состояний,
         * если нельзя
         */
        std::vector<std::vector<double>> checkTask(
                const std::vector<double> &startState, const std::vector<double> &endState,
                double opacity
        );


    protected:

        /**
         * @brief поиск координат соседней свободной точки
         * Это - рекуррентный метод, он пробует прибавить -1, 0 и 1
         * к каждой из координат из интервала [pos, maxPos] включительно
         * @param coords координаты исходной точки
         * @param pos начало интервала перебора
         * @param maxPos конец интервала перебора
         * @param state состояние
         * @return пустой вектор, если точку не удалось найти, в противном случае - вектор
         * координат
         */
        virtual std::vector<int> _findFreePoint(
                std::vector<int> coords, unsigned long pos, unsigned long maxPos,
                const std::vector<double> &state
        );

        /**
         * цена шага сетки вдоль каждой из соответствующих координат
         */
        std::vector<double> _gridSteps;
        /**
         * сгруппированные по роботам цены шага сетки вдоль
         * каждой из соответствующих координат
         */
        std::vector<std::vector<double>> _groupedGridSteps;
        /**
         * количество ячеек сетки планирования вдоль каждой из координат
         */
        int _gridSize;
        /**
         * стартовые координаты планирования на сетке
         */
        std::vector<int> _startCoords;
        /**
         * конечные координаты планирования на сетке
         */
        std::vector<int> _endCoords;
        /**
         * состояние сцены, соответствующее стартовым координатам планирования на сетке
         */
        std::vector<double> _startStateFromCoords;
        /**
         * состояние сцены, соответствующее конечным координатам планирования на сетке
         */
        std::vector<double> _endStateFromCoords;
        /**
         * максимальное расстояние между состояниями, соответствующими соседним узлам сетки планирования
         */
        double _maxDist;
        /**
         * флаг, выполнялось ли последнее планирование по координатам
         */
        bool _coordsUsed = false;
        /**
         * построенный путь (по координатам сетки планирования)
         * для удобства сопоставления с путём в конфигурационном
         * пространстве первая и последняя точки маршрута дублируются
         */
        std::vector<std::vector<int>> _buildedGridPath;

    public:

        /**
         * получить построенный по координатам сетки планирования путь
         * @return построенный по координатам сетки планирования путь
         */
        const std::vector<std::vector<int>> &getBuildedGridPath() const { return _buildedGridPath; }

        /**
         * получить состояние сцены, соответствующее стартовым координатам планирования на сетке
         * @return состояние сцены, соответствующее стартовым координатам планирования на сетке
         */
        const std::vector<double> &getStartStateFromCoords() const { return _startStateFromCoords; }

        /**
         * получить состояние сцены, соответствующее конечным координатам планирования на сетке
         * @return состояние сцены, соответствующее конечным координатам планирования на сетке
         */
        const std::vector<double> &getEndStateFromCoords() const { return _endStateFromCoords; }

        /**
         * получить цену шага сетки вдоль каждой из соответствующих координат
         * @return цена шага сетки вдоль каждой из соответствующих координат
         */
        const std::vector<double> &getGridSteps() const { return _gridSteps; }

        /**
         * получить максимальное расстояние между состояниями, соответствующими соседним узлам сетки планирования
         * @return  максимальное расстояние между состояниями, соответствующими соседним узлам сетки планирования
         */
        const double &getMaxDist() const { return _maxDist; }

        /**
         * получить сгруппированные по роботам цены шага сетки вдоль
         * каждой из соответствующих координат
         * @return ь сгруппированные по роботам цены шага сетки
         */
        const std::vector<std::vector<double>> &getGroupedGridSteps() const { return _groupedGridSteps; }
    };


}
