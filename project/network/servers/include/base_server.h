#pragma once

#include <sys/socket.h>
#include <netinet/in.h>
#include "log.h"
#include "base/path_finder.h"
#include "continuous_path_finder.h"
#include <netinet/in.h>
#include "iostream"
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include "base/path_finder.h"
#include "continuous_path_finder.h"
#include <json/value.h>
#include <json/reader.h>
#include <semaphore.h>
#include <csignal>

/**
 * @brief Базовый класс для всех серверов
 * Для реализации сервера, необходимо переопределить две чисто виртуальные
 * функции: `processCommand()` и `onSocketConnected()`
 */
class BaseServer {
public:

    /**
     * без вызова этого метода отваливается север после отключения клиента
     */
    static void initLinuxServerSocket();

    /**
     * главный цикл обработки запросов
     */
    void mainLoop();

    /**
     * Инициализация сервера
     * @param port порт
     * @param maxClients максимальное число клиентов
     * @param delay  пауза в микросекундах между итерациями цикла
     */
    virtual void init(int port, int maxClients, int delay);

    /**
     * Обработка команды от клиента
     * @param clientSocket  fd клиента
     * @param command код команды
     * @param jsonData данные json
     */
    virtual void processCommand(int clientSocket, int command, Json::Value jsonData) = 0;

    /**
     * Обработка подключения нового клиента
     * @param clientSocket fd клиента
     */
    virtual void onSocketConnected(int clientSocket) = 0;

    /**
     * Обработка запроса клиента
     * @param clientSocket fd клиента
     */
    void processRequests(int clientSocket);

    /**
     * Остановить сервер
     */
    void stop() { _terminated = true; }

    /**
     * Непосредственное подключение сокета
     * @return fd нового клиента
     */
    int acceptNewSocket() {
        return accept(_serverFD, (struct sockaddr *) &_address, (socklen_t *) &_addrLen);
    }

    /**
     * получить fd сервера
     * @return fd сервера
     */
    int getServerFd() const { return _serverFD; }

    /**
     * Получить адрес сервера
     * @return объект адреса сервера
     */
    const sockaddr_in &getAddress() const { return _address; }

protected:
    /**
     * Семафор, синхронизирующий доступ
     */
    sem_t _sem;

private:

    /**
     * объект адреса сервера
     */
    struct sockaddr_in _address;
    /**
     * Порт
     */
    int _port;
    /**
     * Длина адреса
     */
    int _addrLen;
    /**
     * fd сервера
     */
    int _serverFD;
    /**
     * задержка в циклах обработки
     */
    int _delay;
    /**
     * флаг, остановлен ли сервер
     */
    bool _terminated;
    /**
     * буфер для накопления команд
     */
    std::string _commandBuf;

};