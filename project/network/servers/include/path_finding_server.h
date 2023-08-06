#pragma once

#include "base_server.h"

/**
 * @brief Базовый класс для всех серверов планирования
 * для использования необходимо переопределить  в потомке `createPathFinder()`
 */
class PathFindingServer : public BaseServer {
public:
    /**
     * Проверка, готов ли планировщик
     */
    static const int COMMAND_IS_READY = 0;
    /**
     * Запуск планирования
     */
    static const int COMMAND_START_FIND_PATH = 1;
    /**
     * Запрос результата
     */
    static const int COMMAND_FIND_PATH_RESULT = 2;

    /**
     * Обработка запроса клиента
     * @param clientSocket fd клиента
     * @param command код команды
     * @param jsonData данные команды
     */
    void processCommand(int clientSocket, int command, Json::Value jsonData) override;

    /**
     * Начать планирование
     * @param pfs сервер планирования
     * @param pf планировщик пути
     * @param clientSocket fd клиента
     * @param start стартовое состояние
     * @param end конечное состояние
     */
    static void startFindPath(PathFindingServer *pfs, std::shared_ptr<bmpf::PathFinder> pf, int clientSocket,
                              std::vector<double> start, std::vector<double> end);

    /**
     * Обработка подключения нового клиента
     * @param clientSocket fd клиента
     */
    void onSocketConnected(int clientSocket) override;

    /**
     * Создать новый планировщик
     * @param clientSocket fd клиента
     * @return новый планировщик
     */
    virtual std::shared_ptr<bmpf::PathFinder> createPathFinder(int clientSocket) = 0;

protected:
    /**
     * Словарь путей для каждого fd сокета клиента
     */
    std::unordered_map<int, std::vector<std::vector<double>>> paths;
    /**
     * Словарь флагов, готов ли планировщик для каждого fd сокета клиента
     */
    std::unordered_map<int, bool> pathFindingReady;
    /**
     * Словарь планировщиков для каждого fd сокета клиента
     */
    std::unordered_map<int, std::shared_ptr<bmpf::PathFinder>> pfs;
};