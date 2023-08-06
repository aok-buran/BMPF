#include "base/node_grid_path_finder.h"

using namespace bmpf;

/**
 * переместить ноду из открытого множества открытых в множество закрытых
 * @param node нода
 */
void NodeGridPathFinder::_moveNodeFromOpenedToClosed(const std::shared_ptr<PathNode> &node) {
    // получаем свёртку состояния
    long code = (long) convState(node->coords, _gridSize);
    // добавляем его в список обработанных нод
    _closedStateConvCodeSet.insert(code);
    _closedNodes.push_back(node);

    // удаляем ноду из множества открытых (и все её дублёры)
    std::multiset<PathNodePtr>::iterator it;
    for (it = _openSet.begin(); it != _openSet.end() && !areStatesEqual(it->ptr->coords, node->coords); it++);
    if (it != _openSet.end())
        _openSet.erase(it);

}

/**
 * поиск координат в списке закрытых нод
 * @param coords координаты
 * @return флаг, содержатся ли координаты в закрытом списке
 */
bool NodeGridPathFinder::_findCoordsInClosedList(std::vector<int> &coords) {
    long code = (long) convState(coords, _gridSize);
    return _closedStateConvCodeSet.find(code) != _closedStateConvCodeSet.end();
}

/**
 * поиск координат в списке открытых нод
 * @param coords
 * @return флаг, содержатся ли координаты в открытом списке
 */
bool NodeGridPathFinder::_findCoordsInOpenedList(std::vector<int> &coords) {
    for (const PathNodePtr &node: _openSet)
        if (areStatesEqual(node.ptr->coords, coords))
            return true;
    return false;
}

/**
 * построить путь, построенный путь должен быть сохранён в переменную _buildedPath
 */
void NodeGridPathFinder::buildPath() {
    if (!_buildedPath.empty()) {
        _errorCode = NO_ERROR;
        return;
    }

    if (!_endNode) {
        _errorCode = ERROR_CAN_NOT_FIND_PATH;
        return;
    }


    // move between nodes in reverse order: from end node to start node
    while (_endNode) {
        // always insert new node at the begin of list
        std::vector<double> state = coordsToState(_endNode->coords);
        _buildedGridPath.insert(_buildedGridPath.begin(), _endNode->coords);
        _buildedPath.insert(_buildedPath.begin(), state);
        _endNode = _endNode->parent;
    }


    _buildedGridPath.insert(_buildedGridPath.begin(), _startCoords);

    if (!_coordsUsed) {
        // add real start and end points
        //_buildedPath.insert(_buildedPath.begin(), _startStateFromCoords);
        _buildedPath.insert(_buildedPath.begin(), _startState);
        // _buildedPath.emplace_back(_endStateFromCoords);
        _buildedPath.emplace_back(_endState);
    }
    _buildedGridPath.emplace_back(_endCoords);

    _pathLength = calculatePathLength(_buildedPath);
}

/**
 * вспомогательный метод, возвращающий указатель на новую ноду только,
 * если её можно создать
 * @param newCoords координаты
 * @param parentNode указатель на предка
 * @param sum значение метрики
 * @return указатель на новую ноду
 */
std::shared_ptr<PathNode> NodeGridPathFinder::tryToGetNeighborPtr(
        std::vector<int> newCoords, const std::shared_ptr<PathNode> &parentNode, double sum
) {
    if (checkCoords(newCoords)) {
        // если нода уже есть
        if (_findCoordsInClosedList(newCoords) || _findCoordsInOpenedList(newCoords)) {
            return nullptr;
        }
        return std::make_shared<PathNode>(newCoords, parentNode, sum);
    } else {
        return nullptr;
    }
}

/**
 * получить состояние по первой ноде открытого множества
 * @return текущее состояние
 */
std::vector<double> NodeGridPathFinder::getCurrentState() {
    // если открытое множество пустое
    if (_openSet.empty())
        // возвращаем состояние, соответствующее целевым координатам
        return coordsToState(_endCoords);

    std::shared_ptr<PathNode> currentNode = _openSet.begin()->ptr;
    return coordsToState(currentNode->coords);
}

/**
 * такт поиска
 * @param state текущее состояние планировщика
 * @return возвращает true, если планирование закончено
 */
bool NodeGridPathFinder::findTick(std::vector<double> &state) {

    if (_openSet.empty()) {
        _errorCode = ERROR_CAN_NOT_FIND_PATH;
        return true;
    }

    std::shared_ptr<PathNode> currentNode = _openSet.begin()->ptr;

    if (areStatesEqual(currentNode->coords, _endCoords)) {
        _errorCode = NO_ERROR;
        return true;
    }
    state = coordsToState(currentNode->coords);

    if (!checkCoords(currentNode->coords)) {
        throw std::runtime_error("NodeGridPathFinder::findTick() ERROR: coords are disabled");
    }

    _moveNodeFromOpenedToClosed(currentNode);
    std::vector<int> deltaCoords = subtractStates(_endCoords, currentNode->coords);

    if (_closedStateConvCodeSet.size() > _maxNodeCnt) {
        errMsg("closedSet is full");
        _errorCode = ERROR_REACHED_MAX_NODE_CNT;
        return true;
    }
    if (_showTrace) {
        bmpf::infoMsg(currentNode->toString(),
                      " openSet:", _openSet.size(),
                      " closedSet: ", _closedStateConvCodeSet.size());
        bmpf::infoState("delta node", deltaCoords);
        bmpf::infoState("actual state", coordsToState(currentNode->coords));
    }

    auto endNode = _forEachNeighbor(currentNode, _endCoords);

    if (endNode) {
        _endNode = endNode;
        return true;
    }
    return false;
}

/**
 * возвращает координаты всех перебранных точек пространства планирования
 * (из закрытого множества)
 * @return координаты всех перебранных точек
 */
std::vector<std::vector<double>> NodeGridPathFinder::getAllProcessedStates() {
    std::vector<std::vector<double>> findingPoses;
    for (auto &node: _closedNodes)
        findingPoses.emplace_back(coordsToState(node->coords));

    return findingPoses;
}

/**
 * подготовка к планированию
 * @param startState начальное состояние
 * @param endState конечное состояние
 */
void NodeGridPathFinder::prepare(
        const std::vector<double> &startState, const std::vector<double> &endState
) {
    GridPathFinder::prepare(startState, endState);

    _endNode = nullptr;

    if (_errorCode != NO_ERROR) {
        return;
    }

    _closedStateConvCodeSet.clear();
    _closedNodes.clear();
    _openSet.clear();

    _startState = startState;
    _endState = endState;

    std::shared_ptr<PathNode> pathNodePtr = std::make_shared<PathNode>(
            _startCoords, std::shared_ptr<PathNode>(),
            _getPathNodeWeight(_startCoords, _endCoords)
    );
    _openSet.insert(PathNodePtr(pathNodePtr));
}

/**
 * построить путь, построенный путь должен быть сохранён в переменную _buildedPath
 * @param startCoords начальные координаты
 * @param endCoords конечные координаты
 */
void NodeGridPathFinder::prepare(std::vector<int> &startCoords, std::vector<int> &endCoords) {
    GridPathFinder::prepare(startCoords, endCoords);

    _endNode = nullptr;

    if (_errorCode != NO_ERROR) {
        return;
    }

    _closedStateConvCodeSet.clear();
    _closedNodes.clear();
    _openSet.clear();

    std::shared_ptr<PathNode> pathNodePtr = std::make_shared<PathNode>(
            _startCoords, std::shared_ptr<PathNode>(),
            _getPathNodeWeight(_startCoords, _endCoords)
    );
    _openSet.insert(PathNodePtr(pathNodePtr));
}

/**
 * Получить метрику ноды
 * @param curCoords текущие координаты
 * @param endCoords целевые координаты
 * @return метрика ноды
 */
double NodeGridPathFinder::_getPathNodeWeight(std::vector<int> curCoords, std::vector<int> &endCoords) {
    double sum = 0;
    if (_kD != 0) {
        sum += _findLinkDistance(curCoords, endCoords) * _kD;
    }

    if (_kG != 0) {
        double g = 0;
        for (unsigned int i = 0; i < curCoords.size(); i++) {
            g += std::abs(curCoords.at(i) - endCoords.at(i));
        }
        g = g * _kG;
        sum += g;
    }

    return sum;
}

/**
 * получить расстояние между звеньями (по координатам планировщика)
 * @param a координаты первой точки
 * @param b координаты второй точки
 * @return
 */
double NodeGridPathFinder::_findLinkDistance(std::vector<int> &a, std::vector<int> &b) {
    if (a.size() != b.size()) {
        char buf[1024];
        sprintf(buf,
                "GridPathFinder::checkCoords() ERROR: \n a size is %zu, b size is %zu"
                "\nthey must be equal",
                a.size(), b.size()
        );
        throw std::invalid_argument(buf);
    }

    std::vector<double> posesA = _scene->getAllLinkPositions(coordsToState(a));
    std::vector<double> posesB = _scene->getAllLinkPositions(coordsToState(b));
    if (posesA.size() != posesB.size()) {
        char buf[1024];
        sprintf(buf,
                "GridPathFinder::checkCoords() ERROR: \n posesA size is %zu, posesB size is %zu"
                "\nthey must be equal",
                posesA.size(), posesB.size()
        );
        throw std::invalid_argument(buf);
    }

    return getAbsDistance(posesA, posesB);
}
