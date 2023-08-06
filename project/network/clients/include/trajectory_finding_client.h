#pragma once


#include "base_client.h"

/**
 * Клиент планирощика траекторий
 */
class TrajectoryFindingClient : public BaseClient {
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
     * Конструктор
     * @param savePath  путь к файлу сохранения
     * @param lvl уровень производных: 0 - только значения, 1 - значения и скорости
     * 2 - значения, скорости и производные
     */
    TrajectoryFindingClient(const char *savePath, int lvl) : BaseClient(savePath), _lvl(lvl) {}

    /**
     * Запрос, готов ли планировщик
     */
    void requestReady();

    /**
     * Запуск планирования траектории
     */
    void startFindTrajectory();

    /**
     * запрос результата планирования
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
    /**
     * уровень производных: 0 - только значения, 1 - значения и скорости
     * 2 - значения, скорости и производные
     */
    int _lvl;
};


