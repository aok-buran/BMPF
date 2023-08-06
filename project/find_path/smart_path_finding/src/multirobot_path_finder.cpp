#include "multirobot_path_finder.h"

/**
 * рисование сцены планировщика
 * @param state состояние
 * @param onlyRobot флаг, нужно ли рисовать только роботов
 */
void MultiRobotPathFinder::paint(const std::vector<double> &state, bool onlyRobot) {
    if (_wholeScenePathFinder) {
        _wholeScenePathFinder->paint(state, onlyRobot);
    }
}

/**
 * такт поиска
 * @param state текущее состояние планировщика
 * @return возвращает true, если планирование закончено
 */
bool MultiRobotPathFinder::findTick(std::vector<double> &state) {
    if (_pfsNotReadyIndexes.empty())
        return true;

    std::vector<std::vector<double>> singleRobotStates;
    for (const auto &pf: _singleRobotPathFinders) {
        singleRobotStates.push_back(pf->getCurrentState());
    }

    state = _scene->concatenateStates(singleRobotStates);

    std::unordered_set<int> pfsNotReadyIndexes(_pfsNotReadyIndexes);
    for (int index: pfsNotReadyIndexes) {
        auto pf = _singleRobotPathFinders.at(index);
        auto robotState = _scene->getSingleObjectState(state, index);
        if (pf->findTick(robotState)) {
            //infoMsg("ready ", index);
            _pfsNotReadyIndexes.erase(index);
        }
    }

    _prevState = state;
    return false;
}

/**
 * подготовка к планированию
 * @param startState начальное состояние
 * @param endState конечное состояние
 */
void MultiRobotPathFinder::prepare(const std::vector<double> &startState, const std::vector<double> &endState) {
    _singleRobotPathFinders.clear();
    _pfsNotReadyIndexes.clear();

    std::vector<std::shared_ptr<bmpf::Scene>> scenes = _scene->getSingleRobotScenes();

    if (scenes.empty()) {
        throw std::runtime_error("MultiRobotPathFinder::prepare() ERROR:\nscenes list is empty");
    }

    for (int i = 0; i < scenes.size(); i++) {
        auto scene = scenes.at(i);
        auto pf = std::make_shared<bmpf::OneDirectionPathFinder>(
                scene, _showTrace, _maxOpenSetSize, _gridSize,
                _maxNodeCnt, 1, 0, _threadCnt
        );

        auto singleStartState = _scene->getSingleObjectState(startState, i);
        auto singleEndState = _scene->getSingleObjectState(endState, i);
        if (pf->checkTask(singleStartState, singleEndState, 0.001).empty()) {
            pf->prepare(singleStartState, singleEndState);
            _pfsNotReadyIndexes.insert(i);
        }
        _singleRobotPathFinders.push_back(pf);

    }

    // т.к. отклонения между точками незначительны, часто вообще нулевые
    // вдоль некоторых координат, то удобнее использовать упорядоченный планировщик
    _wholeScenePathFinder = std::make_shared<bmpf::OneDirectionOrderedPathFinder>(
            _scene, _showTrace, _maxOpenSetSize, _gridSize,
            _maxNodeCnt, 1, 0, _threadCnt
    );

    _scaleGridSize = _gridSize;

    _scaleWholeScenePathFinder = std::make_shared<bmpf::OneDirectionOrderedPathFinder>(
            _scene, _showTrace, _maxOpenSetSize, _scaleGridSize,
            _maxNodeCnt, 1, 0, _threadCnt
    );
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
std::vector<std::vector<double>> MultiRobotPathFinder::checkTask(
        const std::vector<double> &startState, const std::vector<double> &endState,
        double opacity
) {
    // т.к. отклонения между точками незначительны, часто вообще нулевые
    // вдоль некоторых координат, то удобнее использовать упорядоченный планировщик
    _wholeScenePathFinder = std::make_shared<bmpf::OneDirectionOrderedPathFinder>(
            _scene, _showTrace, _maxOpenSetSize, _gridSize,
            _maxNodeCnt, 1, 0, _threadCnt
    );

    _buildedPath = _wholeScenePathFinder->checkTask(startState, endState, opacity);

    return _buildedPath;
}


/**
 * построить путь, построенный путь должен быть сохранён в переменную _buildedPath
 */
void MultiRobotPathFinder::buildPath() {
    std::vector<std::vector<std::vector<double>>> singleRobotPaths;
    std::vector<std::vector<std::vector<int>>> singleRobotGridPaths;

    int maxSize = -1;
    int maxGridSize = -1;


    // строим пути для каждого робота в единый список
    for (const auto &pf: _singleRobotPathFinders) {
        pf->buildPath();
        auto singleRobotPath = pf->getBuildedPath();
        auto singleRobotGridPath = pf->getBuildedGridPath();

        if (maxSize == -1 || maxSize < singleRobotPath.size())
            maxSize = (int) singleRobotPath.size();
        singleRobotPaths.push_back(singleRobotPath);

        if (maxGridSize == -1 || maxGridSize < singleRobotGridPath.size())
            maxGridSize = (int) singleRobotGridPath.size();
        singleRobotGridPaths.push_back(singleRobotGridPath);
    }

    if (maxSize == -1) {
        throw std::runtime_error(
                "MultiRobotPathFinder::buildPath() ERROR:\n maxSize is -1, it means, that all paths are empty");
    }

    // собираем единый путь, длина итогового пути будет равна
    // самому длинному из единичных роботов, недостающие участки
    // достраиваются за счёт копирования последнего состояния
    // пути единичного робота
    std::vector<std::vector<double>> notCheckedPath;
    std::vector<std::vector<int>> notCheckedGridPath;

    for (int i = 0; i < maxSize; i++) {
        std::vector<std::vector<double>> preUnited;
        for (auto singleRobotPath: singleRobotPaths) {
            if (singleRobotPath.size() <= i) {
                preUnited.emplace_back(singleRobotPath.back());
            } else {
                preUnited.emplace_back(singleRobotPath.at(i));
            }
        }
        notCheckedPath.emplace_back(_scene->concatenateStates(preUnited));
    }
    for (int i = 0; i < maxGridSize; i++) {
        std::vector<std::vector<int>> preGridUnited;

        for (auto singleRobotPath: singleRobotGridPaths) {
            if (singleRobotPath.size() <= i) {
                preGridUnited.emplace_back(singleRobotPath.back());
            } else {
                preGridUnited.emplace_back(singleRobotPath.at(i));
            }
        }
        notCheckedGridPath.emplace_back(_scene->concatenateCoords(preGridUnited));
    }

    // находим диапазоны индексов состояний пути всей сцены, в которых
    // есть коллизии (добавление выполняется парами: индекс начала диапазона,
    // индекс его окончания)
    std::vector<int> collisionRanges;
    bool prevIndexCollision = false;
    for (int i = 0; i < notCheckedPath.size(); i++) {
        // infoMsg(i, " ", checkState(notCheckedPath.at(i)));
        if (checkState(notCheckedPath.at(i))) {
            if (prevIndexCollision) {
                collisionRanges.emplace_back(i - 1);
            }
            prevIndexCollision = false;
        } else {
            if (!prevIndexCollision) {
                collisionRanges.emplace_back(i);
            }
            prevIndexCollision = true;
        }
    }
    if (prevIndexCollision)
        collisionRanges.emplace_back(notCheckedPath.size() - 1);


    if (collisionRanges.size() % 2 != 0) {
        throw std::runtime_error("MultiRobotPathFinder::buildPath() ERROR:\n collisionRanges size is not even");
    }


    // если все состояния пути не имеют коллизий, сразу
    // сохраняем путь и выходим
    if (collisionRanges.empty()) {
        _buildedPath = notCheckedPath;
        return;
    }

    std::vector<std::vector<double>> checkedPath;


    // перебираем диапазоны коллизий
    int prevCollisionEnd = 0;


    for (int i = 0; i < collisionRanges.size() / 2; i++) {

        int collisionStartIndex = collisionRanges.at(i * 2);
        int collisionEndIndex = collisionRanges.at(i * 2 + 1);

        std::vector<std::vector<double>> localPath;

        // ищем путь между ближайшими состояниями, свободными от коллизий
        int errCode;
        do {

            // т.к. отклонения между точками незначительны, часто вообще нулевые
            // вдоль некоторых координат, то удобнее использовать упорядоченный планировщик
            _wholeScenePathFinder = std::make_shared<bmpf::OneDirectionOrderedPathFinder>(
                    _scene, _showTrace, _maxOpenSetSize, _gridSize,
                    _maxNodeCnt, 1, 0, _threadCnt
            );
            errCode = NO_ERROR;

            if (collisionStartIndex == 1 || collisionEndIndex == notCheckedPath.size() - 2) {
                do {
                    localPath = _scaleWholeScenePathFinder->findPath(
                            notCheckedPath.at(collisionStartIndex - 1),
                            notCheckedPath.at(collisionEndIndex + 1),
                            errCode
                    );
                    if (errCode == NO_ERROR)
                        break;

                    _scaleGridSize = (int) ((_scaleGridSize) * 1.3);
                    _scaleWholeScenePathFinder = std::make_shared<bmpf::OneDirectionOrderedPathFinder>(
                            _scene, _showTrace, _maxOpenSetSize, _scaleGridSize,
                            _maxNodeCnt, 1, 0, _threadCnt
                    );
                } while (errCode != NO_ERROR);
            } else {
                localPath = _wholeScenePathFinder->findGridPath(
                        notCheckedGridPath.at(collisionStartIndex - 1),
                        notCheckedGridPath.at(collisionEndIndex + 1),
                        errCode
                );
            }


            if (errCode == bmpf::GridPathFinder::ERROR_CAN_NOT_FIND_FREE_START_POINT) {
                if (collisionStartIndex > 1) {
                    collisionStartIndex--;
                } else {
                    _errorCode = bmpf::GridPathFinder::ERROR_CAN_NOT_FIND_FREE_START_POINT;
                    return;
                }
            } else if (errCode == bmpf::GridPathFinder::ERROR_CAN_NOT_FIND_FREE_END_POINT) {
                //  errMsg("endPoint problem");
                if (collisionEndIndex < notCheckedPath.size() - 2) {
                    collisionEndIndex++;
                } else {
                    _errorCode = bmpf::GridPathFinder::ERROR_CAN_NOT_FIND_FREE_END_POINT;
                    return;
                }
            }

        } while (errCode != NO_ERROR);


        // добавляем все состояния пути до первого состояния с коллизией исключительно
        for (int j = prevCollisionEnd; j < collisionStartIndex; j++) {
            checkedPath.emplace_back(notCheckedPath.at(j));
        }

        checkedPath.insert(checkedPath.end(), localPath.begin(), localPath.end());
        // переходим к следующей коллизии
        prevCollisionEnd = collisionEndIndex + 1;
    }

    for (int j = prevCollisionEnd; j < notCheckedPath.size(); j++)
        checkedPath.emplace_back(notCheckedPath.at(j));

    _buildedPath = checkedPath;

}

