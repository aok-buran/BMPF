#pragma once


#include "path_finding_server.h"
#include "continuous_path_finder.h"
#include "base/path_finder.h"
#include "scene.h"

/**
 * Сервер непрерывного планирования
 * для использования необходимо переопределить  в потомке `createPathFinder()`
 */
class ContinuousPFServer : public PathFindingServer {
public:

    /**
     * конструктор
     * @param scene сцена
     * @param showTrace флаг, нужно ли выводить информацию во время поиска пути
     * @param maxOpenSetSize максимальный размер открытом множества
     * @param gridSize размер сетки планирования
     * @param maxNodeCnt максимальное кол-во нод в закрытом множестве
     * @param threadCnt количество потоков планировщика
     * @param checkCnt количество промежуточных проверок каждого этапа пути
     */
    ContinuousPFServer(const std::shared_ptr<bmpf::Scene> &scene, bool showTrace,
                       unsigned int maxOpenSetSize, int gridSize, unsigned int maxNodeCnt, int checkCnt,
                       int threadCnt = 1) {
        _scene = scene;
        _showTrace = showTrace;
        _maxOpenSetSize = maxOpenSetSize;
        _gridSize = gridSize;
        _maxNodeCnt = maxNodeCnt;
        _checkCnt = checkCnt;
        _threadCnt = threadCnt;
    }

    /**
     * создать планировщик
     * @param clientSocket fd клиента
     * @return новый планировщик
     */
    std::shared_ptr<bmpf::PathFinder> createPathFinder(int clientSocket) override {
        std::shared_ptr<bmpf::PathFinder> pf = std::make_shared<ContinuousPathFinder>(
                _scene, _showTrace, _maxOpenSetSize, _gridSize,
                _maxNodeCnt, _checkCnt, _threadCnt
        );

        return pf;
    }

private:
    /**
     * сцена
     */
    std::shared_ptr<bmpf::Scene> _scene;
    /**
     * флаг, нужно ли выводить информацию во время поиска пути
     */
    bool _showTrace;
    /**
     * максимальный размер закрытого множества
     */
    unsigned int _maxOpenSetSize;
    /**
     * размер сетки планирования
     */
    int _gridSize;
    /**
     *  максимальное кол-во нод в закрытом множестве
     */
    unsigned int _maxNodeCnt;
    /**
     * количество потоков планирования
     */
    int _threadCnt;
    /**
     * количество промежуточных проверок каждого этапа пути
     */
    int _checkCnt;
};