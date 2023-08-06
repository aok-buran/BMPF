#include "one_direction_path_finder.h"


using namespace bmpf;

/**
 * для всех соседей текущей ноды метод должен добавить только
 * подходящих в множество _openSet, если один из соседей
 * имеет целевые указания (т.е. найден путь), то возвращаем указатель на эту ноду,
 * в противном случае должен быть возвращён nullptr
 * @param currentNode текущая нода
 * @param endCoords целевые координаты
 * @return найденная нода или null
 */
std::shared_ptr<PathNode> OneDirectionPathFinder::_forEachNeighbor(
        std::shared_ptr<PathNode> currentNode, std::vector<int> &endCoords
) {
    // перебираем смещения
    for (const std::vector<int> &offset: _offsetList) {
        std::vector<int> newCoords = bmpf::sumStates(currentNode->coords, offset);

        double sum = _getPathNodeWeight(newCoords, endCoords);

        std::shared_ptr<PathNode> newNode = tryToGetNeighborPtr(newCoords, currentNode, sum);

        if (newNode) {
            //  infoMsg("new node");
            if (bmpf::areStatesEqual(newNode->coords, endCoords))
                return newNode;

            _openSet.insert(PathNodePtr(newNode));
            if (_maxOpenSetSize != 0 && _openSet.size() > _maxOpenSetSize)
                _openSet.erase(std::prev(_openSet.end()));

        }
    }
    return nullptr;
}

/**
 * инициализация смещений
 */
void OneDirectionPathFinder::_initOffsets() {
    _offsetList.clear();

    std::vector<int> tmpVec;
    for (unsigned int i = 0; i < _scene->getJointCnt(); i++)
        tmpVec.push_back(0);

    for (unsigned int i = 0; i < _scene->getJointCnt(); i++) {
        _offsetStandartIndexes.push_back(i * 2);
        _offsetStandartIndexes.push_back(i * 2 + 1);
        tmpVec.at(i) = 1;
        _offsetList.push_back(tmpVec);
        tmpVec.at(i) = -1;
        _offsetList.push_back(tmpVec);
        tmpVec.at(i) = 0;
    }

    for (const auto &sceneDescription: _scene->getRobots()) {
        std::vector<std::vector<int>> localOffsetList;
        std::vector<int> tmpVec2;
        for (unsigned int i = 0; i < sceneDescription->getJointCnt(); i++)
            tmpVec.push_back(0);

        for (unsigned int i = 0; i < sceneDescription->getJointCnt(); i++) {
            tmpVec.at(i) = 1;
            localOffsetList.push_back(tmpVec2);
            tmpVec.at(i) = -1;
            localOffsetList.push_back(tmpVec2);
            tmpVec.at(i) = 0;
        }
    }

}