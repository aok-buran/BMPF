#include "one_direction_ordered_path_finder.h"


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
std::shared_ptr<PathNode> OneDirectionOrderedPathFinder::_forEachNeighbor(
        std::shared_ptr<PathNode> currentNode,
        std::vector<int> &endCoords
) {
    std::vector<int> delta = bmpf::subtractStates(endCoords, currentNode->coords);

    std::vector<unsigned long> sizeOrderedOffsetIndexes;
    if (bmpf::isStateLimited(delta, 1))
        sizeOrderedOffsetIndexes = _offsetStandartIndexes;
    else
        sizeOrderedOffsetIndexes = _getOrderedOffsetIndexes(delta);

    unsigned long listSize = sizeOrderedOffsetIndexes.size();

    for (unsigned int i = 0; i < listSize; i++) {
        std::vector<int> &offset = _offsetList.at(sizeOrderedOffsetIndexes.at(i));

        std::vector<int> newCoords = bmpf::sumStates(currentNode->coords, offset);
        double sum = _getPathNodeWeight(newCoords, endCoords);

        std::shared_ptr<PathNode> newNode = tryToGetNeighborPtr(newCoords, currentNode, sum);

        if (newNode) {
            if (bmpf::areStatesEqual(newNode->coords, endCoords))
                return newNode;

            _openSet.insert(PathNodePtr(newNode));
            if (_maxOpenSetSize != 0 && _openSet.size() > _maxOpenSetSize)
                _openSet.erase(std::prev(_openSet.end()));

            continue;
        }

    }
    return nullptr;
}

/**
 * список смещений по отклонениям от цели (сцена из нескольких роботов)
 * @param deltas
 * @return список смещений по отклонениям от цели
 */
std::vector<unsigned long> OneDirectionOrderedPathFinder::_getOrderedOffsetIndexesMultiRobot(
        std::vector<int> &deltas
) {
    // специальная структура для сортировки смещений
    struct RobotStruct {
        RobotStruct(unsigned int index, std::vector<int> &coords) {
            this->index = index;
            this->coords = coords;
        }

        unsigned int index;
        std::vector<int> coords;

        bool operator<(const RobotStruct &obj) const {
            //infoMsg(this->coords.size(), " ", obj.coords.size());

            assert(this->coords.size() == obj.coords.size());
            int aMax = std::abs(*this->coords.begin());
            int bMax = std::abs(*obj.coords.begin());
            for (int coord: this->coords) {
                if (std::abs(coord) > aMax)
                    aMax = std::abs(coord);
            }
            for (int coord: obj.coords) {
                if (std::abs(coord) > bMax)
                    bMax = std::abs(coord);
            }
            return aMax > bMax;
        }
    };

    std::vector<RobotStruct> robotsStructs;

    long pos = 0;
    unsigned long j = 0;
    for (const auto &robot: _scene->getRobots()) {
        std::vector<int> tmp =
                std::vector<int>(deltas.begin() + pos, deltas.begin() + pos + (long) robot->getJointCnt());
        robotsStructs.emplace_back(j, tmp);
        pos += (long) robot->getJointCnt();
        j++;
    }

    unsigned long robotCnt = robotsStructs.size();

    std::sort(robotsStructs.begin(), robotsStructs.end());

    std::vector<unsigned long> result;
    for (unsigned long k = 0; k < robotCnt; k++) {
        for (int i = 0; i < 6; i++) {
            unsigned long statePos = robotsStructs.at(k).index * 6 + i;
            result.push_back(statePos * 2);
            result.push_back(statePos * 2 + 1);
        }
    }

    return result;
}
/**
 * список смещений по отклонениям от цели (сцена из одного робота)
 * @param deltas отклонения
 * @return список смещений по отклонениям от цели
 */
std::vector<unsigned long>
OneDirectionOrderedPathFinder::_getSizeOrderedOffsetIndexesSingleRobot(std::vector<int> &deltas) {
    struct OffsetStruct {
        OffsetStruct(unsigned int index, int offset) {
            this->index = index;
            this->offset = offset;
        }

        unsigned int index;
        int offset;

        bool operator<(const OffsetStruct &obj) const {
            return std::abs(this->offset) < std::abs(obj.offset);
        }
    };

    std::vector<OffsetStruct> offsetStructs;

    for (unsigned int i = 0; i < deltas.size(); i++) {
        offsetStructs.emplace_back(i, deltas.at(i));
    }

    std::sort(offsetStructs.begin(), offsetStructs.end());

    std::vector<unsigned long> zeroOffsets;
    std::vector<unsigned long> directOffsets;
    std::vector<unsigned long> inDirectOffsets;

    for (OffsetStruct offsetStruct: offsetStructs) {
        if (std::abs(offsetStruct.offset) >= 1) {
            if (offsetStruct.offset > 0) {
                directOffsets.push_back(offsetStruct.index * 2 + 1);
                inDirectOffsets.insert(inDirectOffsets.begin(), offsetStruct.index * 2);
            } else {
                directOffsets.push_back(offsetStruct.index * 2);
                inDirectOffsets.insert(inDirectOffsets.begin(), offsetStruct.index * 2 + 1);
            }
        } else {
            zeroOffsets.push_back(offsetStruct.index * 2);
            zeroOffsets.push_back(offsetStruct.index * 2 + 1);
        }
    }
    directOffsets.insert(directOffsets.end(), zeroOffsets.begin(), zeroOffsets.end());
    directOffsets.insert(directOffsets.end(), inDirectOffsets.begin(), inDirectOffsets.end());
    return directOffsets;
}

/**
 * получить упорядоченные смещения по отклонению от цели (главный метод)
 * @param deltas отклонения
 * @return список смещений по отклонениям от цели
 */
std::vector<unsigned long> OneDirectionOrderedPathFinder::_getOrderedOffsetIndexes(std::vector<int> &deltas) {
    //  если робот один
    if (!_scene->isSingleActiveRobot())
        return _getOrderedOffsetIndexesMultiRobot(deltas);
    else
        return _getSizeOrderedOffsetIndexesSingleRobot(deltas);

}


