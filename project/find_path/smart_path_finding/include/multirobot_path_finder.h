#pragma once


#include "base/path_finder.h"
#include "base/node_grid_path_finder.h"
#include "one_direction_sync_path_finder.h"
#include "one_direction_ordered_path_finder.h"

/**
 * Планировщик для сцены с несколькими роботами.
 * Каждый такт планирования он сначала запускает для каждого робота
 * в отдельности. Потом, если нет коллизий на общей сцене,
 * то переходит к следующему, если нет, то откатывает к предыдущему состоянию
 * всех роботов, кроме одного и повторяет планирование из
 * предыдущего состояния
 *
 *
 * Логика работы планировщика следующая:
 * 1) подготовка к тактам поиска пути
 * 2) выполнение тактов поиска пути с попутным заполнением тех или иных структур
 * 3) если путь найден из этих структур собирается путь и сохраняется в
 * переменную `_buildedPath`
 *
 */
class MultiRobotPathFinder : public bmpf::PathFinder {
public:

    /**
     * конструктор
     * @param scene сцена
     * @param showTrace флаг, нужно ли выводить информацию во время поиска пути
     * @param maxOpenSetSize максимальный размер закрытого множества
     * @param gridSize размер сетки планирования
     * @param maxNodeCnt максимальное кол-во нод в закрытом множестве
     * @param threadCnt количество потоков планировщика
     */
    MultiRobotPathFinder(
            const std::shared_ptr<bmpf::Scene> &scene, bool showTrace,
            unsigned int maxOpenSetSize, int gridSize, unsigned int maxNodeCnt, int threadCnt = 1
    ) : PathFinder(scene, showTrace, threadCnt),
        _maxOpenSetSize(maxOpenSetSize), _maxNodeCnt(maxNodeCnt), _gridSize(gridSize),
        _scaleGridSize(gridSize) {
        _ready = true;
    };

    /**
     * Деструктор
     */
    virtual ~MultiRobotPathFinder() {
        _singleRobotPathFinders.clear();
        _pfsNotReadyIndexes.clear();
    }

    /**
     * рисование сцены планировщика
     * @param state состояние
     * @param onlyRobot флаг, нужно ли рисовать только роботов
     */
    void paint(const std::vector<double> &state, bool onlyRobot) override;

    /**
     * такт поиска
     * @param state текущее состояние планировщика
     * @return возвращает true, если планирование закончено
     */
    bool findTick(std::vector<double> &state) override;

    /**
     * подготовка к планированию
     * @param startState начальное состояние
     * @param endState конечное состояние
     */
    void prepare(const std::vector<double> &startState, const std::vector<double> &endState) override;

    /**
     * построить путь, построенный путь должен быть сохранён в переменную _buildedPath
     */
    void buildPath() override;

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
     * планировщики отдельных роботов
     */
    std::vector<std::shared_ptr<bmpf::NodeGridPathFinder>> _singleRobotPathFinders;
    /**
     * максимальное кол-во нод
     */
    unsigned int _maxNodeCnt;
    /**
     * максимальный размер открытого множества
     */
    unsigned int _maxOpenSetSize;
    /**
     * количество ячеек сетки планирования вдоль каждой из координат
     */
    int _gridSize;
    /**
     * количество ячеек сетки планирования вдоль каждой из координат
     * с возможностью увеличения
     */
    int _scaleGridSize;
    /**
     * предыдущее состояние сцены
     */
    std::vector<double> _prevState;
    /**
     * индексы не готовых планировщиков
     */
    std::unordered_set<int> _pfsNotReadyIndexes;
    /**
     * планировщик для всей сцены
     */
    std::shared_ptr<bmpf::NodeGridPathFinder> _wholeScenePathFinder;
    /**
     * планировщик для всей сцены с возможностью уменьшения ячейки сетки планирования
     */
    std::shared_ptr<bmpf::NodeGridPathFinder> _scaleWholeScenePathFinder;
};
