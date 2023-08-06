#include "base/grid_path_finder.h"

using namespace bmpf;

/**
 * конструктор
 * @param scene сцена
 * @param showTrace флаг, нужно ли выводить информацию во время поиска пути
 * @param gridSize размер сетки планирования
 * @param threadCnt количество потоков планировщика
 */
GridPathFinder::GridPathFinder(
        const std::shared_ptr<bmpf::Scene> &scene, bool showTrace, int gridSize, int threadCnt
) : PathFinder(scene, showTrace, threadCnt) {

    _gridSize = gridSize;
    _groupedGridSteps.clear();
    _gridSteps.clear();

    double sumStep = 0;

    for (const auto &sceneDescription: scene->getRobots()) {
        std::vector<double> localSteps;
        for (const auto &actuator: sceneDescription->getJointParamsList()) {
            double step = (actuator->maxAngle - actuator->minAngle) / gridSize;
            localSteps.emplace_back(step);
            _gridSteps.emplace_back(step);
            sumStep += step * step;
        }
        _groupedGridSteps.emplace_back(localSteps);
    }

    _maxDist = std::sqrt(sumStep) * gridSize;

    for (unsigned long i = 0; i < _scene->getJointCnt(); i++) {
        _startState.emplace_back(0.0);
        _endState.emplace_back(0.0);
    }

}

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
std::vector<std::vector<double>> GridPathFinder::checkTask(
        const std::vector<double> &startState, const std::vector<double> &endState,
        double opacity
) {
    _buildedPath.clear();
    _buildedGridPath.clear();

    auto startCoords = stateToCoords(startState);
    if (startCoords.empty())
        return _buildedPath;


    if (areStatesEqual(startState, endState, opacity)) {
        _buildedPath.push_back(startState);
        _buildedGridPath.push_back(startCoords);

        _pathLength = calculatePathLength(_buildedPath);
        return _buildedPath;
    }


    auto endCoords = stateToCoords(endState);
    if (endCoords.empty())
        return _buildedPath;


    if (areStatesEqual(startCoords, endCoords)) {
        _buildedPath.push_back(startState);
        _buildedPath.push_back(coordsToState(startCoords));
        _buildedPath.push_back(endState);
        _buildedGridPath.push_back(startCoords);
        _buildedGridPath.push_back(startCoords);
        _buildedGridPath.push_back(startCoords);

        _pathLength = calculatePathLength(_buildedPath);
        return _buildedPath;
    }

    return _buildedPath;
}

/**
 * подготовка к планированию
 * @param startState начальное состояние
 * @param endState конечное состояние
 */
void GridPathFinder::prepare(const std::vector<double> &startState, const std::vector<double> &endState) {
    if (_showTrace) {
        infoMsg("GridPathFinder::prepare");
        infoState("start state: ", startState);
        infoState("end state: ", endState);
    }


    _coordsUsed = false;

    _startState = startState;
    _endState = endState;

    _startCoords = stateToCoords(startState);
    _endCoords = stateToCoords(endState);

    if (_startCoords.empty()) {
        _errorCode = ERROR_CAN_NOT_FIND_FREE_START_POINT;
        return;
    }
    if (_endCoords.empty()) {
        _errorCode = ERROR_CAN_NOT_FIND_FREE_END_POINT;
        return;
    }

    if (!checkCoords(_startCoords))
        throw std::runtime_error("GridPathFinder::prepare() ERROR: \n _startState is disabled");

    if (!checkCoords(_endCoords))
        throw std::runtime_error("GridPathFinder::prepare() ERROR: \n _endState is disabled");

    if (areStatesEqual(_startCoords, _endCoords))
        throw std::runtime_error("GridPathFinder::prepare() ERROR: \n _startState and _endState are the same");



    if (_showTrace) {
        infoState("start coords: ", _startCoords);
        infoState("end coords: ", _endCoords);
    }


    _startStateFromCoords = coordsToState(_startCoords);
    _endStateFromCoords = coordsToState(_endCoords);

    if (_showTrace) {
        infoState("startStateFromCoords: ", _startStateFromCoords);
        infoState("endStateFromCoords: ", _endStateFromCoords);
        infoMsg("prepared");
    }
    _errorCode = NO_ERROR;


    _buildedPath.clear();
    _buildedGridPath.clear();
}

/**
 * построить путь, построенный путь должен быть сохранён в переменную _buildedPath
 * @param startCoords начальные координаты
 * @param endCoords конечные координаты
 */
void GridPathFinder::prepare(std::vector<int> &startCoords, std::vector<int> &endCoords) {
    if (_showTrace) {
        infoMsg("GridPathFinder::prepare");
        infoState("start coords: ", startCoords);
        infoState("end coords: ", endCoords);
    }

    _coordsUsed = true;

    _startState = coordsToState(startCoords);
    _endState = coordsToState(endCoords);

    if (!checkState(_startState))
        throw std::runtime_error("GridPathFinder::prepare() ERROR: \n _startState is disabled");

    if (!checkState(_endState))
        throw std::runtime_error("GridPathFinder::prepare() ERROR: \n _endState is disabled");


    if (_showTrace) {
        infoState("start state: ", _startState);
        infoState("end state: ", _endState);
    }

    _startCoords = startCoords;
    _endCoords = endCoords;

    if (_startCoords.empty()) {
        _errorCode = ERROR_CAN_NOT_FIND_FREE_START_POINT;
        return;
    }
    if (_endCoords.empty()) {
        _errorCode = ERROR_CAN_NOT_FIND_FREE_END_POINT;
        return;
    }

    if (areStatesEqual(_startCoords, _endCoords))
        throw std::runtime_error("GridPathFinder::prepare() ERROR: \n _startState and _endState are the same");


    if (_showTrace) {
        infoState("start coords: ", _startCoords);
        infoState("end coords: ", _endCoords);
    }

    _errorCode = NO_ERROR;
    _buildedPath.clear();
    _buildedGridPath.clear();
}


/**
 * поиск пути из состояния `startCoords` в состояние `endCoords`,
 *
 * @param startCoords начальные координаты
 * @param endCoords конечные координаты
 * @param errorCode  в эту переменную записывается код ошибки
 * @return путь по сетке планирования
 */
std::vector<std::vector<double>>
GridPathFinder::findGridPath(std::vector<int> &startCoords, std::vector<int> &endCoords, int &errorCode) {

    if (areStatesEqual(startCoords, endCoords))
        return {coordsToState(startCoords)};

    _startTime = std::chrono::high_resolution_clock::now();

    prepare(startCoords, endCoords);

    if (_errorCode != NO_ERROR) {
        errorCode = _errorCode;
        return {};
    }

    std::vector<double> actualState;

    // если очередной такт поиска пути не последний
    while (!findTick(actualState)) {};

    if (_errorCode != NO_ERROR)
        return {};

    // строим путь
    buildPath();

    auto endTime = std::chrono::high_resolution_clock::now();
    _calculationTimeInSeconds =
            (double) std::chrono::duration_cast<std::chrono::milliseconds>(endTime - _startTime).count() / 1000;

    errorCode = _errorCode;
    return _buildedPath;
}

/**
 * преобразование целочисленных координат в вещественное состояние
 * @param coords координаты
 * @return состояние
 */
std::vector<double> GridPathFinder::coordsToState(std::vector<int> &coords) const {
    std::vector<double> state;
    for (unsigned int i = 0; i < coords.size(); i++)
        state.push_back(coords.at(i) * _gridSteps.at(i) + _scene->getJointParamsList().at(i)->minAngle);
    return state;
}

/**
 * преобразование целочисленных координат в вещественное состояние
 * @param coords координаты
 * @param robotNum  номер робота, координаты нужно преобразовать
 * @return состояние
 */
std::vector<double> GridPathFinder::coordsToState(std::vector<int> coords, unsigned long robotNum) {
    std::vector<double> state;
    for (unsigned int i = 0; i < 6; i++)
        state.push_back(coords.at(i) * _gridSteps.at(i) + _scene->getJointParamsList().at(i)->minAngle);
    return state;
}

/**
 * Проверка доступности целочисленных координат для всей сцены
 * @param coords координаты
 * @return флаг, доступны ли целочисленных координаты
 */
bool GridPathFinder::checkCoords(std::vector<int> coords) {
    if (coords.size() != _scene->getJointCnt()) {
        char buf[1024];
        sprintf(buf,
                "GridPathFinder::checkCoords() ERROR: \n coords size is %zu, joint count is %zu"
                "\nthey must be equal",
                coords.size(), _scene->getJointCnt()
        );
        throw std::invalid_argument(buf);
    }

    for (int coord: coords)
        if (coord < 0 || coord >= _gridSize)
            return false;


    return checkState(coordsToState(coords));
}

/**
 * Проверка доступности целочисленных координат для робота с индексом `robotNum`
 * @param coords координаты
 * @param robotNum номер робота
 * @return  флаг, доступны ли целочисленных координаты
 */
bool GridPathFinder::checkCoords(const std::vector<int> &coords, unsigned int robotNum) {
    for (int coord: coords)
        if (coord < 0 || coord >= (int) _gridSize)
            return false;

    std::vector<double> state = coordsToState(coords, robotNum);

    return _scene->isStateEnabled(state, robotNum);
}

/**
 * преобразует состояние в координаты на сетке планирования,
 * если свободных координат не найдено, возвращает пустой список {}
 * @param state состояние
 * @return координаты
 */
std::vector<int> GridPathFinder::stateToCoords(std::vector<double> state) {
    // стартовые значения координат
    std::vector<int> coords;
    for (unsigned int i = 0; i < state.size(); i++)
        coords.push_back(
                (int) ((state.at(i) - _scene->getJointParamsList().at(i)->minAngle) / _gridSteps.at(i)));

    // если координаты, полученные взятием целой части подходят, то просто возвращаем их
    if (checkCoords(coords)) return coords;

    // массив флагов, готов ли тот или иной робот
    std::vector<bool> robotReady;
    for (unsigned long i = 0; i < _scene->getActiveRobotCnt(); i++)
        robotReady.emplace_back(false);

    // количество роботов, для которых определённы целочисленные координаты
    int readyRobotCnt = 0;
    // количество готовых роботов на прошлом такте
    int prevReadyRobotCnt = -1;
    // пока получается устанавливать новые координаты хотя бы
    // для одного робота, это нужно, т.к. положение одного робота
    // может привести к коллизии с другим, поэтому мы циклически
    // проходим по всем необработанным роботам и пытаемся их обработать
    while (prevReadyRobotCnt != readyRobotCnt) {
        prevReadyRobotCnt = readyRobotCnt;
        // перебираем роботов на сцене
        for (unsigned long i = 0; i < _scene->getActiveRobotCnt(); i++)
            // если робот ещё не готов
            if (!robotReady.at(i)) {
                // пытаемся найти соседние свободные координаты
                auto newCoords = _findFreePoint(
                        coords, _scene->getJointIndexRanges().at(i).first,
                        _scene->getJointIndexRanges().at(i).second, state
                );
                // если свободных координат не удалось найти, переходим к следующему роботу
                if (newCoords.empty()) continue;
                // если координаты для робота установлены
                readyRobotCnt++;
                robotReady.at(i) = true;
                // сохраняем новые значения координат для обработанного робота
                auto jRange = _scene->getJointIndexRanges().at(i);
                for (long j = jRange.first; j <= jRange.second; j++)
                    coords.at(j) = newCoords.at(j);

            }
    }
    // если все роботы обработаны
    if (readyRobotCnt == _scene->getActiveRobotCnt())
        // возвращаем составленные координаты
        return coords;

    return {};
}


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
std::vector<int> GridPathFinder::_findFreePoint(
        std::vector<int> coords, unsigned long pos, unsigned long maxPos, const std::vector<double> &state
) {

    if (pos > maxPos) return {};

    // рекуррентно для каждой координаты перебираем три варианта: вычесть из неё 1,
    // прибавить 1 или ничего не делать; для каждого варианта запускается новый такт рекурсии
    // по следующей координате
    std::vector<int> tmpCoords = coords;
    for (int i = -1; i <= 1; i++) {
        tmpCoords.at(pos) = coords.at(pos) + i;
        if (checkCoords(tmpCoords))
            return tmpCoords;
        else {
            if (pos < coords.size() - 1) {
                auto newCoords = _findFreePoint(tmpCoords, pos + 1, maxPos, state);
                if (!newCoords.empty())
                    return newCoords;
            }
        }
    }

    return {};
}

