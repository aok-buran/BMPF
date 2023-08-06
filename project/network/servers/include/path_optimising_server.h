#pragma once

#include "base_server.h"
#include "optimize_path_median.h"

/**
 * @brief Сервер оптимизации пути
 */
class PathOptimisingServer : public BaseServer {
public:
    /**
     * Проверка, готов ли планировщик
     */
    static const int COMMAND_IS_READY = 0;
    /**
     * Запуск оптимизации
     */
    static const int COMMAND_START_OPTIMIZE_PATH = 1;
    /**
     * команда оптимизации пути
     */
    static const int COMMAND_OPTIMIZE_PATH = 2;

    /**
     * Обработка запроса клиента
     * @param clientSocket fd клиента
     * @param command код команды
     * @param jsonData данные команды
     */
    void processCommand(int clientSocket, int command, Json::Value jsonData) override;

    /**
     * Запуск оптимизации пути
     * @param pos сервер оптимизации
     * @param po оптимизатор пути
     * @param clientSocket fd клиента
     * @param path путь, который нужно потимизировать
     */
    static void startOptimisePath(
            PathOptimisingServer *pos, std::shared_ptr<bmpf::MedianPathOptimizer> po, int clientSocket,
            std::vector<std::vector<double>> path
    );

    /**
     * Обработка подключения нового клиента
     * @param clientSocket fd клиента
     */
    void onSocketConnected(int clientSocket) override;


    /**
     * конструктор
     * @param scene сцена
     * @param showTrace флаг, нужно ли выводить информацию во время поиска пути
     * @param maxOpenSetSize максимальный размер открытом множества
     * @param gridSize размер сетки планирования
     * @param maxNodeCnt максимальное кол-во нод в закрытом множестве
     * @param checkCnt количество промежуточных проверок каждого этапа пути
     * @param filterDivideCnt количество разбиений медианного фильтра
     * @param optimizeLoopCnt количество проходов оптимизации
     * @param threadCnt количество потоков планировщика
     */
    PathOptimisingServer(
            const std::shared_ptr<bmpf::Scene> &scene, bool showTrace,
            unsigned int maxOpenSetSize, int gridSize, unsigned int maxNodeCnt,
            int checkCnt,
            int filterDivideCnt, int optimizeLoopCnt, int threadCnt = 1
    ) {
        _scene = scene;
        _checkCnt = checkCnt;
        _showTrace = showTrace;
        _maxOpenSetSize = maxOpenSetSize;
        _gridSize = gridSize;
        _maxNodeCnt = maxNodeCnt;
        _threadCnt = threadCnt;
        _filterDivideCnt = filterDivideCnt;
        _optimizeLoopCnt = optimizeLoopCnt;
    }


protected:
    /**
     * Словарь путей для каждого fd сокета клиента
     */
    std::unordered_map<int, std::vector<std::vector<double>>> paths;
    /**
     * Словарь флагов, готов ли оптимизатор для каждого fd сокета клиента
     */
    std::unordered_map<int, bool> pathOptimisingReady;
    /**
     * Словарь оптимизаторов для каждого fd сокета клиента
     */
    std::unordered_map<int, std::shared_ptr<bmpf::MedianPathOptimizer>> pfs;
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
    /**
     * Количество разбиений медианного фильтра
     */
    int _filterDivideCnt;
    /**
     * Количество проходов оптимизации
     */
    int _optimizeLoopCnt;

};