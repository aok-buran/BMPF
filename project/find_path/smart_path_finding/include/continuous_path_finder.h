#pragma once


#include "base/path_finder.h"
#include "base/node_grid_path_finder.h"
#include "one_direction_sync_path_finder.h"
#include "one_direction_ordered_path_finder.h"
#include "multirobot_path_finder.h"

/**
 * Базовы1 класс для всех склеивающих планировщиков
 * по тому или иному условию он определяет участки пути
 * которые не соответствуют тем или иным требованиям
 * и планирует новую траекторию между началом
 * недоступного участков пути и концом, потом склеивает
 * полученные части пути
 *
 * Логика работы планировщика следующая:
 * 1) подготовка к тактам поиска пути
 * 2) выполнение тактов поиска пути с попутным заполнением тех или иных структур
 * 3) если путь найден из этих структур собирается путь и сохраняется в
 * переменную `_buildedPath`
 *
 */
class ContinuousPathFinder : public bmpf::PathFinder {
public:

    /**
     * конструктор
     * @param scene сцена
     * @param showTrace флаг, нужно ли выводить информацию во время поиска пути
     * @param maxOpenSetSize максимальный размер закрытого множества
     * @param gridSize размер сетки планирования
     * @param maxNodeCnt максимальное кол-во нод в закрытом множестве
     * @param checkCnt количество проверок каждого этапа пути
     * @param threadCnt количество потоков планировщика
     */
    ContinuousPathFinder(
            const std::shared_ptr<bmpf::Scene> &scene, bool showTrace,
            unsigned int maxOpenSetSize, int gridSize, unsigned int maxNodeCnt, int checkCnt,
            int threadCnt = 1
    ) : PathFinder(scene, showTrace, threadCnt),
        _checkCnt(checkCnt), _maxOpenSetSize(maxOpenSetSize), _maxNodeCnt(maxNodeCnt),
        _gridSize(gridSize) {
        _ready = true;
    };

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
     * кол-во шагов проверки этапа пути
     */
    int _checkCnt;
    /**
     * планировщик для всей сцены
     */
    std::shared_ptr<ContinuousPathFinder> _localPathFinder;
    /**
     * планировщик для всей сцены с возможностью уменьшения ячейки сетки планирования
     */
    std::shared_ptr<MultiRobotPathFinder> _globalPathFinder;
};
