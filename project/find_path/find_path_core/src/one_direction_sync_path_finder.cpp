#include "one_direction_sync_path_finder.h"
#include "log.h"
#include <thread>
#include <future>
#include <functional>
#include <utility>


using namespace bmpf;

/**
 * Проводит вычисления для конкретного потока
 * @param prm результат
 * @param newCoords новые координаты
 * @param currentNode текущая нода
 * @param pointer указатель на планировщик
 * @param sum метрика
 */
void createGetNeighborThread(
        std::promise<std::shared_ptr<PathNode>> prm,
        std::vector<int> newCoords,
        std::shared_ptr<PathNode> currentNode,
        OneDirectionSyncPathFinder *pointer, double sum
) {

    std::shared_ptr<PathNode> ptr = pointer->tryToGetNeighborPtr(
            std::move(newCoords),
            currentNode,
            sum
    );
    prm.set_value_at_thread_exit(ptr);
}

/**
 * для всех соседей текущей ноды метод должен добавить только
 * подходящих в множество _openSet, если один из соседей
 * имеет целевые указания (т.е. найден путь), то возвращаем указатель на эту ноду,
 * в противном случае должен быть возвращён nullptr
 * @param currentNode текущая нода
 * @param endCoords целевые координаты
 * @return найденная нода или null
 */
std::shared_ptr<PathNode>
OneDirectionSyncPathFinder::_forEachNeighbor(std::shared_ptr<PathNode> currentNode, std::vector<int> &endCoords) {
    // перебираем пакеты
    for (auto &group: _groupedOffsetList) {

        std::vector<std::future<std::shared_ptr<PathNode>>> futures;
        std::vector<std::promise<std::shared_ptr<PathNode>>> promises;
        std::vector<std::thread> threads;

        // для каждого смещения в пакете создаём поток с
        // ожиданием результата выполнения
        for (const auto &offset: group) {
            std::vector<int> newCoords = bmpf::sumStates(currentNode->coords, offset);
            // sum=g+h-c
            double sum = _getPathNodeWeight(newCoords, endCoords);

            std::shared_ptr<PathNode> newNode = tryToGetNeighborPtr(newCoords, currentNode, sum);

            std::promise<std::shared_ptr<PathNode>> promise;
            futures.push_back(promise.get_future());

            std::thread thread(
                    createGetNeighborThread, std::move(promise), std::move(newCoords), currentNode, this, sum
            );
            thread.detach();
            threads.push_back(std::move(thread));
        }

        // перебираем фюьчерсы, ожидая от каждого результата вычисления соседней ноды
        for (unsigned int i = 0; i < group.size(); i++) {
            std::shared_ptr<PathNode> newNode = futures.at(i).get();
            if (newNode) {
                if (bmpf::areStatesEqual(newNode->coords, endCoords)) {
                    return newNode;
                }

                _openSet.insert(PathNodePtr(newNode));
                if (_maxOpenSetSize != 0 && _openSet.size() > _maxOpenSetSize)
                    _openSet.erase(std::prev(_openSet.end()));
            }
        }
    }
    return nullptr;
}

