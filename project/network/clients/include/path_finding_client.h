#pragma once


#include "base_client.h"

/**
 * Клиент планирования пути
 */
class PathFindingClient : public BaseClient {
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
      * конструктор
      * @param savePath путь к файлу сохранения
      */
    explicit PathFindingClient(const char *savePath) : BaseClient(savePath) {}

    /**
     * Запрос, готов ли планировщик
     */
    void requestReady();

    /**
     * Запуск планирования
     */
    void startFindPath();

    /**
     * Запрос результата
     */
    void getResult();

    /**
     * главный цикл планирования
     */
    void mainLoop() override;

    /**
     * Инициализация клиента
     * @param port порт
     * @param addr адрес сервера
     * @param delay  пауза в микросекундах между итерациями цикла
     * @param start стартовое состояние
     * @param end конечное состояние
    */
    virtual void init(int port, const char *addr, std::vector<double> start, std::vector<double> end, int delay);

protected:
    /**
     * стартовое состояние
     */
    std::vector<double> _start;
    /**
     * конечное состояние
     */
    std::vector<double> _end;
};


