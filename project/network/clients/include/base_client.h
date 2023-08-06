#pragma once

#include <vector>

#include <arpa/inet.h>
#include <cstdio>
#include "iostream"
#include "base_client.h"
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <json/value.h>
#include <json/writer.h>
#include <json/reader.h>


/**
 * @brief Базовый класс для всех клиентов
 * необходимо реализовать метод `mainLoop()`
 */
class BaseClient {
public:

    /**
     * конструктор
     * @param savePath путь к файлу сохранения
     */
    explicit BaseClient(const char *savePath) {
        _savePath = savePath;
    }

    /**
     * главный цикл обработки запросов
     */
    virtual void mainLoop() = 0;

    /**
     * Инициализация клиента
     * @param port порт
     * @param addr адрес сервера
     * @param delay  пауза в микросекундах между итерациями цикла
     */
    virtual void init(int port, const char *addr, int delay);

    /**
     * получить fd клиента
     * @return  fd клиента
     */
    int getClientFd() const { return clientFD; }

protected:
    /**
     * буфер для накопления команд
     */
    std::string _commandBuf;
    /**
     * fd клиента
     */
    int clientFD{};
    /**
     * путь к файлу сохранения
     */
    const char *_savePath;
    /**
     * адрес сервера
     */
    const char *_addr{};
    /**
     * задержка
     */
    int _delay{};
    /**
     * порт
     */
    int _port{};
};