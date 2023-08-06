#include "continuous_path_finder.h"

/**
 * рисование сцены планировщика
 * @param state состояние
 * @param onlyRobot флаг, нужно ли рисовать только роботов
 */
void ContinuousPathFinder::paint(const std::vector<double> &state, bool onlyRobot) {
    if (_globalPathFinder) {
        _globalPathFinder->paint(state, onlyRobot);
    }
}

/**
 * такт поиска
 * @param state текущее состояние планировщика
 * @return возвращает true, если планирование закончено
 */
bool ContinuousPathFinder::findTick(std::vector<double> &state) {
    return _globalPathFinder->findTick(state);
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
std::vector<std::vector<double>> ContinuousPathFinder::checkTask(
        const std::vector<double> &startState, const std::vector<double> &endState,
        double opacity
) {
    // т.к. отклонения между точками незначительны, часто вообще нулевые
    // вдоль некоторых координат, то удобнее использовать упорядоченный планировщик
    _globalPathFinder = std::make_shared<MultiRobotPathFinder>(
            _scene, _showTrace, _maxOpenSetSize, _gridSize,
            _maxNodeCnt, _threadCnt
    );
    _buildedPath = _globalPathFinder->checkTask(startState, endState, opacity);

    return _buildedPath;
}

/**
 * подготовка к планированию
 * @param startState начальное состояние
 * @param endState конечное состояние
 */
void ContinuousPathFinder::prepare(const std::vector<double> &startState, const std::vector<double> &endState) {
    _globalPathFinder = std::make_shared<MultiRobotPathFinder>(
            _scene, _showTrace, _maxOpenSetSize, _gridSize,
            _maxNodeCnt, _threadCnt
    );
    _globalPathFinder->prepare(startState, endState);
}


/**
 * построить путь, построенный путь должен быть сохранён в переменную _buildedPath
 */
void ContinuousPathFinder::buildPath() {
    _globalPathFinder->buildPath();

    // собираем единый путь, длина итогового пути будет равна
    // самому длинному из единичных роботов, недостающие участки
    // достраиваются за счёт копирования последнего состояния
    // пути единичного робота
    std::vector<std::vector<double>> notCheckedPath = _globalPathFinder->getBuildedPath();

    notCheckedPath.erase(notCheckedPath.begin() + 1);
    notCheckedPath.erase(notCheckedPath.end() - 2);

    // infoMsg("test");
    // находим диапазоны индексов состояний пути всей сцены, в которых
    // есть коллизии (добавление выполняется парами: индекс начала диапазона,
    // индекс его окончания)
    std::vector<std::pair<int, int>> collisionRanges;
    int cStartIndex = -1;
    for (int i = 0; i < notCheckedPath.size() - 1; i++) {
        auto prevState = notCheckedPath.at(i);
        auto curState = notCheckedPath.at(i + 1);
        if (!divideCheckPathSegment(prevState, curState, _checkCnt)) {
            if (cStartIndex == -1)
                cStartIndex = i;
        } else {
            if (cStartIndex != -1)
                collisionRanges.emplace_back(std::make_pair(cStartIndex, i));

            cStartIndex = -1;
        }
    }
    if (cStartIndex != -1)
        collisionRanges.emplace_back(std::make_pair(cStartIndex, notCheckedPath.size() - 1));

    // если все состояния пути не имеют коллизий, сразу
    // сохраняем путь и выходим
    if (collisionRanges.empty()) {
        _buildedPath = notCheckedPath;
        return;
    }

    std::vector<std::vector<double>> checkedPath;

    // перебираем диапазоны коллизий
    int prevCollisionEnd = 0;
    //  infoMsg("test4");
    for (auto &collisionRange: collisionRanges) {

        int collisionStartIndex = collisionRange.first;
        int collisionEndIndex = collisionRange.second;

        std::vector<std::vector<double>> localPath;

        // ищем путь между ближайшими состояниями, свободными от коллизий
        int errCode;
        do {
            // т.к. отклонения между точками незначительны, часто вообще нулевые
            // вдоль некоторых координат, то удобнее использовать упорядоченный планировщик
            _localPathFinder = std::make_shared<ContinuousPathFinder>(
                    _scene, _showTrace, _maxOpenSetSize, _gridSize,
                    _maxNodeCnt, _threadCnt
            );
            errCode = NO_ERROR;

            localPath = _localPathFinder->checkTask(
                    notCheckedPath.at(std::max(collisionStartIndex - 1, 0)),
                    notCheckedPath.at(collisionEndIndex + 1),
                    0.01
            );

            if (localPath.empty()) {

                localPath = _localPathFinder->findPath(
                        notCheckedPath.at(std::max(collisionStartIndex - 1, 0)),
                        notCheckedPath.at(collisionEndIndex + 1),
                        errCode
                );
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

