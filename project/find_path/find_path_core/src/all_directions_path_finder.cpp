#include "all_directions_path_finder.h"


using namespace bmpf;

// для всех соседей текущей ноды метод должен добавить только
// подходящих в множество _openSet, если один из соседей
// имеет целевые указания, то возвращаем указатель на эту ноду,
// в противном случае должен быть возвращён nullptr
std::shared_ptr<PathNode> AllDirectionsPathFinder::_forEachNeighbor(
        std::shared_ptr<PathNode> currentNode,
        std::vector<int> &endCoords
) {
    std::vector<int> offset(endCoords.size(), 0);

    std::shared_ptr<PathNode> res = nullptr;

    loop(offset, 0,
         [&currentNode, this, &endCoords, &res](const std::vector<int> & of) {
             //  infoState(offset,"local offset");
             std::vector<int> newCoords = bmpf::sumStates(currentNode->coords, of);

             double sum = _getPathNodeWeight(newCoords, endCoords);

             std::shared_ptr<PathNode> newNode = tryToGetNeighborPtr(newCoords, currentNode, sum);

             if (newNode) {
                 //  infoMsg("new node");
                 if (bmpf::areStatesEqual(newNode->coords, endCoords)) {
                     res = newNode;
                 }
                 _openSet.insert(PathNodePtr(newNode));
                 if (_maxOpenSetSize != 0 && _openSet.size() > _maxOpenSetSize) {
                     _openSet.erase(std::prev(_openSet.end()));
                 }

             }

         });
    return res;
}
