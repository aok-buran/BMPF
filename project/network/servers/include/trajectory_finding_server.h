#pragma once

#include "base_server.h"
#include "monotone_trajectory_finder.h"

/**
 * Сервер для планирования траекторий
 */
class TrajectoryFindingServer : public BaseServer {
public:

    /**
     * Проверка, готов ли планировщик
     */
    static const int COMMAND_IS_READY = 0;
    /**
     * Запуск поиска траектории
     */
    static const int COMMAND_START_FIND_TRAJECTORY = 1;
    /**
     * Получить положения траектории
     */
    static const int COMMAND_GET_POSITIONS = 2;
    /**
     * Получить скорости траектории
     */
    static const int COMMAND_GET_SPEED = 3;
    /**
     * Получить ускорения траектории
     */
    static const int COMMAND_GET_ACCELERATIONS = 4;

    /**
     * Обработка запроса клиента
     * @param clientSocket fd клиента
     * @param command код команды
     * @param jsonData данные команды
     */
    void processCommand(int clientSocket, int command, Json::Value jsonData) override;

    /**
     * Запуск планирования траектории
     * @param tps сервер планирования
     * @param tf планировщик
     * @param clientSocket fd клиента
     * @param start стартовое состояние
     * @param end конечное состояние
     */
    static void startFindTrajectory(
            TrajectoryFindingServer *tps, std::shared_ptr<bmpf::MonotoneTrajectoryFinder> tf,
            int clientSocket, std::vector<double> start, std::vector<double> end);

    /**
     * Обработка подключения нового клиента
     * @param clientSocket fd клиента
     */
    void onSocketConnected(int clientSocket) override;

    /**
     *
     * @param scene сцена
     * @param intervalDuration интервал между временными метками
     * @param showTrace флаг, выводить ли лог
     * @param maxOpenSetSize максимальный размер открытого множества
     * @param gridSize размер сетки планирования
     * @param maxNodeCnt максимальное кол-во нод в закрытом множестве
     * @param checkCnt количество проверок каждого этапа пути
     * @param threadCnt количество потоков планировщика
     */
    TrajectoryFindingServer(const std::shared_ptr<bmpf::Scene> &scene, double intervalDuration, bool showTrace,
                            unsigned int maxOpenSetSize, int gridSize, unsigned int maxNodeCnt, int checkCnt,
                            int threadCnt = 1) {
        _scene = scene;
        _intervalDuration = intervalDuration;
        _showTrace = showTrace;
        _maxOpenSetSize = maxOpenSetSize;
        _gridSize = gridSize;
        _maxNodeCnt = maxNodeCnt;
        _checkCnt = checkCnt;
        _threadCnt = threadCnt;
    }

protected:
    /**
     * Словарь флагов, готов ли планировщик для каждого fd сокета клиента
     */
    std::unordered_map<int, bool> trajectoryFindingReady;
    /**
     * Словарь планировщиков для каждого fd сокета клиента
     */
    std::unordered_map<int, std::shared_ptr<bmpf::MonotoneTrajectoryFinder>> tfs;
    /**
     * сцена
     */
    std::shared_ptr<bmpf::Scene> _scene;
    /**
     *  интервал между временными метками
     */
    double _intervalDuration;
    /**
     * флаг, выводить ли лог
     */
    bool _showTrace;
    /**
     * максимальный размер открытого множества
     */
    unsigned int _maxOpenSetSize;
    /**
     *  размер сетки планирования
     */
    int _gridSize;
    /**
     * максимальное кол-во нод в закрытом множестве
     */
    unsigned int _maxNodeCnt;
    /**
     * количество проверок каждого этапа пути
     */
    int _checkCnt;
    /**
     * количество потоков планировщика
     */
    int _threadCnt;
};