#pragma once


#include "base_client.h"

/**
 * Клиент оптимизатора пути
 */
class PathOptimizingClient : public BaseClient {
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
      * конструктор
      * @param savePath путь к файлу сохранения
      */
    explicit PathOptimizingClient(const char *savePath) : BaseClient(savePath) {}
    /**
     * Запрос, готов ли оптимизатор
     */
    void requestReady();
    /**
     * Запуск оптимизации
     */
    void startOptimizePath();
    /**
     * Запрос результата
     */
    void getResult();
    /**
     * главный цикл оптимизации
     */
    void mainLoop() override;

    /**
     * Инициализация клиента
     * @param port порт
     * @param addr адрес сервера
     * @param path путь, который нужно оптимизировать
     * @param delay  пауза в микросекундах между итерациями цикла
     */
    virtual void init(int port, const char *addr, std::vector<std::vector<double>> path, int delay);

protected:
    /**
     * Путь
     */
    std::vector<std::vector<double>> _path;
};


