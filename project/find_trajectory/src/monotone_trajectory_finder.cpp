#include "monotone_trajectory_finder.h"

using namespace bmpf;


/**
 * инициализация
 * @param scene сцена
 * @param intervalDuration интервал между временными метками
 * @param showTrace флаг, выводить ли лог
 * @param maxOpenSetSize максимальный размер открытого множества
 * @param gridSize размер сетки
 * @param maxNodeCnt максимальное кол-во нод в закрытом множестве
 * @param checkCnt количество проверок каждого этапа пути
 * @param threadCnt количество потоков планировщика
 */
void MonotoneTrajectoryFinder::init(
        const std::shared_ptr<bmpf::Scene> &scene, double intervalDuration, bool showTrace,
        unsigned int maxOpenSetSize, int gridSize, unsigned int maxNodeCnt, int checkCnt, int threadCnt
) {
    _scene = scene;
    _pf = std::make_shared<ContinuousPathFinder>(scene, showTrace, maxOpenSetSize, gridSize,
                                                 maxNodeCnt, checkCnt, threadCnt);
    _startIntervalDuration = intervalDuration;
}

/**
 * Подготовка траектории
 * @param startState стартовое положение
 * @param endState конечное
 * @param errorCode код ошибки
 */
void MonotoneTrajectoryFinder::prepareTrajectory(
        const std::vector<double> &startState, const std::vector<double> &endState, int &errorCode
) {
    _isReady = false;
    _interpolators.clear();

    _lastPath = _pf->findPath(startState, endState, errorCode);

    _nodeCnt = _lastPath.size();

    _timeIntervals = std::vector<double>(_nodeCnt, 0);
    for (int i = 0; i < _nodeCnt; i++)
        _timeIntervals[i] = _startIntervalDuration * i;

    _prepareTrajectory(startState, endState, _timeIntervals, errorCode);
}

/**
 * Подготовка траектории
 * @param startState стартовое положение
 * @param endState конечное
 * @param timeIntervals интервалы времени
 * @param errorCode код ошибки
 */
void MonotoneTrajectoryFinder::_prepareTrajectory(
        const std::vector<double> &startState, const std::vector<double> &endState,
        std::vector<double> &timeIntervals, int &errorCode
) {
    _isReady = false;
    std::vector<std::vector<double>> ys;

    _wholeDuration = (_lastPath.size() - 1) * _startIntervalDuration;

    for (int j = 0; j < _lastPath.front().size(); j++) {
        ys.emplace_back(std::vector<double>());
    }

    for (int i = 0; i < _lastPath.size(); i++) {
        _timeIntervals[i] = _startIntervalDuration * i;
        for (int j = 0; j < _lastPath.front().size(); j++) {
            ys[j].emplace_back(_lastPath.at(i).at(j));
        }
    }
    for (int j = 0; j < _lastPath.front().size(); j++) {
        auto interpolator = std::make_shared<MonotoneCubicInterpolation>(_timeIntervals.data(), ys.at(j).data(),
                                                                         _timeIntervals.size());
        _interpolators.emplace_back(interpolator);
    }

    _lastTrajectory.clear();
    for (int i = 0; i < _lastPath.size(); i++) {
        auto node = getTrajectoryNode(_timeIntervals[i]);
        _lastTrajectory.emplace_back(getTrajectoryNode(_timeIntervals[i]));
    }
    _isReady = true;
}

/**
 * @brief получить все ноды траекторий
 * получить все ноды траекторий (первая координата время, потом положения,
 * потом скорости, потом ускорения, размерность каждого узла можно узнать из сцены
 * scene->getJointCnt()
 * @param tm время
 * @return все ноды траекторий
 */
std::vector<double> MonotoneTrajectoryFinder::getTrajectoryNode(double tm) {
    if (_lastPath.empty())
        throw std::runtime_error("MonotoneTrajectoryFinder::getTrajectoryNode() ERROR: \nlastPath is empty");

    std::vector<double> trajectoryNode;

    trajectoryNode.push_back(tm);
    for (int j = 0; j < _lastPath.front().size(); j++) {
        trajectoryNode.push_back(
                _interpolators.at(j)->getInterpolatedValue(tm)
        );
    }
    for (int j = 0; j < _lastPath.front().size(); j++) {
        trajectoryNode.push_back(
                _interpolators.at(j)->getInterpolatedSpeed(tm)
        );
    }
    for (int j = 0; j < _lastPath.front().size(); j++) {
        trajectoryNode.push_back(
                _interpolators.at(j)->getInterpolatedAcceleration(tm)
        );
    }
    return trajectoryNode;
}

/**
 * получить все положения траекторий (первая координата время, потом положения)
 * @param tm время
 * @return все положения траекторий
 */
std::vector<double> MonotoneTrajectoryFinder::getTrajectoryPosition(double tm) {
    if (_lastPath.empty())
        throw std::runtime_error("MonotoneTrajectoryFinder::getTrajectoryNode() ERROR: \nlastPath is empty");
    std::vector<double> trajectoryValue;

    trajectoryValue.emplace_back(tm);
    for (int j = 0; j < _lastPath.front().size(); j++) {
        trajectoryValue.emplace_back(
                _interpolators.at(j)->getInterpolatedValue(tm)
        );
    }
    return trajectoryValue;
}

/**
 * получить все скорости траекторий (первая координата время, потом скорости)
 * @param tm время
 * @return все скорости траекторий
 */
std::vector<double> MonotoneTrajectoryFinder::getTrajectorySpeed(double tm) {
    if (_lastPath.empty())
        throw std::runtime_error("MonotoneTrajectoryFinder::getTrajectoryNode() ERROR: \nlastPath is empty");
    std::vector<double> trajectoryValue;

    trajectoryValue.emplace_back(tm);
    for (int j = 0; j < _lastPath.front().size(); j++) {
        trajectoryValue.push_back(
                _interpolators.at(j)->getInterpolatedSpeed(tm)
        );
    }
    return trajectoryValue;
}

/**
 * получить все ускорения траекторий (первая координата время, потом ускорения)
 * @param tm время
 * @return все ускорения траекторий
 */
std::vector<double> MonotoneTrajectoryFinder::getTrajectoryAcceleration(double tm) {
    if (_lastPath.empty())
        throw std::runtime_error("MonotoneTrajectoryFinder::getTrajectoryNode() ERROR: \nlastPath is empty");
    std::vector<double> trajectoryValue;

    trajectoryValue.emplace_back(tm);
    for (int j = 0; j < _lastPath.front().size(); j++) {
        trajectoryValue.push_back(
                _interpolators.at(j)->getInterpolatedAcceleration(tm)
        );
    }
    return trajectoryValue;
}
