#pragma once

#include <set>
#include "path_node.h"
#include "scene.h"
#include "grid_path_finder.h"


namespace bmpf {
    /**
     *
     * @brief Базовый класс для всех планировщиков на сетке с нодами
     * Базовый класс для всех планировщиков на сетке с нодами, у каждой из которых
     * указывается предок и координаты на сетке. Суть планирования
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
     *
     */
    class NodeGridPathFinder : public GridPathFinder {
    public:
        /**
         * Ошибка планирования: достигнут предельный размер открытого множества
         */
        static const int ERROR_REACHED_MAX_NODE_CNT = 4;

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
        NodeGridPathFinder(const std::shared_ptr<bmpf::Scene> &scene,
                           bool showTrace,
                           int gridSize,
                           unsigned int maxNodeCnt,
                           unsigned int kG = 1,
                           unsigned int kD = 0,
                           int threadCnt = 1
        ) : GridPathFinder(scene, showTrace, gridSize, threadCnt),
            _maxNodeCnt(maxNodeCnt), _kG(kG), _kD(kD) {}

        /**
         * возвращает координаты всех перебранных точек пространства планирования
         * (из закрытого множества)
         * @return координаты всех перебранных точек
         */
        std::vector<std::vector<double>> getAllProcessedStates();

        /**
         * такт поиска
         * @param state текущее состояние планировщика
         * @return возвращает true, если планирование закончено
         */
        bool findTick(std::vector<double> &state) override;

        /**
         * для всех соседей текущей ноды метод должен добавить только
         * подходящих в множество _openSet, если один из соседей
         * имеет целевые указания (т.е. найден путь), то возвращаем указатель на эту ноду,
         * в противном случае должен быть возвращён nullptr
         * @param currentNode текущая нода
         * @param endCoords целевые координаты
         * @return найденная нода или null
         */
        virtual std::shared_ptr<PathNode> _forEachNeighbor(
                std::shared_ptr<PathNode> currentNode, std::vector<int> &endCoords
        ) = 0;

        /**
         * вспомогательный метод, возвращающий указатель на новую ноду только,
         * если её можно создать
         * @param newCoords координаты
         * @param parentNode указатель на предка
         * @param sum значение метрики
         * @return указатель на новую ноду
         */
        std::shared_ptr<PathNode>
        tryToGetNeighborPtr(std::vector<int> newCoords, const std::shared_ptr<PathNode> &parentNode, double sum);

        /**
         * построить путь, построенный путь должен быть сохранён в переменную _buildedPath
         */
        void buildPath() override;

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
        void prepare(std::vector<int> &startState, std::vector<int> &endState) override;

        /**
         * получить состояние по первой ноде открытого множества
         * @return текущее состояние
         */
        std::vector<double> getCurrentState();

        /**
         * перейти к следующей ноде из открытого списка
         */
        void nextNode() { _openSet.erase(_openSet.begin()); }

        /**
         * получить расстояние между звеньями (по координатам планировщика)
         * @param a координаты первой точки
         * @param b координаты второй точки
         * @return
         */
        double _findLinkDistance(std::vector<int> &a, std::vector<int> &b);

    protected:

        /**
         * переместить ноду из открытого множества открытых в множество закрытых
         * @param node нода
         */
        void _moveNodeFromOpenedToClosed(const std::shared_ptr<PathNode> &node);

        /**
         * поиск координат в списке закрытых нод
         * @param coords координаты
         * @return флаг, содержатся ли координаты в закрытом списке
         */
        bool _findCoordsInClosedList(std::vector<int> &coords);

        /**
         * поиск координат в списке открытых нод
         * @param coords
         * @return флаг, содержатся ли координаты в открытом списке
         */
        bool _findCoordsInOpenedList(std::vector<int> &coords);

        /**
         * Получить метрику ноды
         * @param curCoords текущие координаты
         * @param endCoords целевые координаты
         * @return метрика ноды
         */
        double _getPathNodeWeight(std::vector<int> curCoords, std::vector<int> &endCoords);

        /**
         * коэффициент разницы в углах поворота сочленений робота
         */
        unsigned int _kG;
        /**
         * коэффициент разницы в положениях звеньев робота
         */
        unsigned int _kD;
        /**
         * Последняя нода планирования
         */
        std::shared_ptr<PathNode> _endNode;
        /**
         * список закрытых нод
         */
        std::vector<std::shared_ptr<PathNode>> _closedNodes;
        /**
         * неупорядоченное множество свёрток обработанных состояний (так быстрее)
         */
        std::unordered_set<long> _closedStateConvCodeSet;
        /**
         * Открытое множество нод для обработки (упорядочено по метрике)
         */
        std::set<PathNodePtr> _openSet;
        /**
         * максимальное количество нод
         */
        unsigned int _maxNodeCnt;

    };


}