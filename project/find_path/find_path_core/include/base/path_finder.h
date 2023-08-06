#pragma once

#include <memory>
#include <utility>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <scene.h>
#include <json/json.h>
#include <chrono>
#include "base/collider.h"
#include "log.h"
#include "solid_collider.h"
#include "solid_sync_collider.h"
#include "state.h"

namespace bmpf {
    /**
     * @brief Базовый класс для всех планировщиков
     *
     * Логика работы планировщика следующая:
     * 1) подготовка к тактам поиска пути
     * 2) выполнение тактов поиска пути с попутным заполнением тех или иных структур
     * 3) если путь найден из этих структур собирается путь и сохраняется в
     * переменную `_buildedPath`
     *
     */
    class PathFinder {
    public:

        /**
         * Нет ошибки
         */
        static const int NO_ERROR = -1;
        /**
         * Не удалось найти путь
         */
        static const int ERROR_CAN_NOT_FIND_PATH = 1;

        /**
         * конструктор
         * @param scene сцена
         * @param showTrace флаг, нужно ли выводить информацию во время поиска пути
         * @param threadCnt количество потоков планировщика
         */
        PathFinder(const std::shared_ptr<bmpf::Scene> &scene, bool showTrace, int threadCnt = 1);

        /**
         * Поиск пути
         * @param startState стартовое состояние
         * @param endState конечное состояние
         * @param errorCode в эту переменную записывается код ошибки
         * @return
         */
        std::vector<std::vector<double>>
        findPath(const std::vector<double> &startState, const std::vector<double> &endState, int &errorCode);

        /**
         * такт поиска
         * @param state текущее состояние планировщика
         * @return возвращает true, если планирование закончено
         */
        virtual bool findTick(std::vector<double> &state) = 0;

        /**
         * подготовка к планированию
         * @param startState начальное состояние
         * @param endState конечное состояние
         */
        virtual void prepare(const std::vector<double> &startState, const std::vector<double> &endState) = 0;

        /**
         * построить путь, построенный путь должен быть сохранён в переменную _buildedPath
         */
        virtual void buildPath() = 0;

        /**
         * обновить коллайдер по сцене
         */
        void updateCollider();

        /**
         * рисование сцены планировщика
         * @param state состояние
         * @param onlyRobot флаг, нужно ли рисовать только роботов
         */
        virtual void paint(const std::vector<double> &state, bool onlyRobot);

        /**
         * проверить отрезок с промежуточными точками на коллизии
         * @param prevPoint первая точка отрезка
         * @param nextPoint вторая точка отрезка
         * @param checkCnt количество промежуточных точек
         * @return флаг, является ли отрезок безколлизионным
         */
        bool divideCheckPathSegment(const std::vector<double> &prevPoint, std::vector<double> nextPoint, int checkCnt);

        /**
         * проверить путь с промежуточными точками на коллизии
         * @param path путь
         * @param checkCnt  количество промежуточных точек
         * @return флаг, является ли путь безколлизионным
         */
        int divideCheckPath(std::vector<std::vector<double>> path, int checkCnt);

        /**
         * вывести на консоль путь
         * @param path путь
         */
        static void infoPath(const std::vector<std::vector<double>> &path);

        /**
         * Получить json-представление пути
         * @param path путь
         * @return json-представление
         */
        static Json::Value getJSONPath(std::vector<std::vector<double>> path);
        /**
         * Получить путь из json-представления
         * @param json json-представление
         * @return путь
         */
        static std::vector<std::vector<double>> getPathFromJSON(const Json::Value& json);

        /**
         * сохранить путь в файл
         * @param path путь
         * @param filename путь к файлу
         */
        static void savePathToFile(std::vector<std::vector<double>> path, const std::string &filename);

        /**
         * Загрузить список путь из файла
         * @param filename путь к файлу с путями
         * @param scenePath путь к сцене (читается из файла, сохраняется в scenePath)
         * @return список путей
         */
        static std::vector<std::vector<double>> loadPathFromFile(const std::string &filename, std::string &scenePath);

        /**
         * Загрузить список путей из файла
         * @param filename путь к файлу с путями
         * @param scenePath путь к сцене (читается из файла, сохраняется в scenePath)
         * @return список путей
         */
        static std::vector<std::vector<std::vector<double>>>
        loadPathsFromFile(const std::string &filename, std::string &scenePath);

        /**
         * разбить путь на части (partCnt - кол-во частей)
         * @param path путь
         * @param partCnt количество разбиений каждого этапа
         * @return новый путь с промежуточными точками
         */
        static std::vector<std::vector<double>> splitPath(std::vector<std::vector<double>> path, unsigned long partCnt);


        /**
         * проверить опорные точки пути на коллизии
         * @param path путь
         * @param maxDist максимальное расстояние между опорными точками
         * @return флаг, является ли путь безколлизионным
         */
        bool simpleCheckPath(const std::vector<std::vector<double>> &path, double maxDist);

        /**
         * @brief проверяет доступность состояния
         * Проверяет доступность углов, после проверяет состояние на коллизии
         * @param state состояние
         * @return флаг, допустимо ли состояние
         */
        bool checkState(const std::vector<double> &state);

        /**
         * получить случайное разрешённое состояние
         * @return случайное разрешённое состояние
         */
        std::vector<double> getRandomState();

        /**
         * @brief получить состояние по времени
         * две соседние точки считаются разделёнными
         * единичным временным интервалом.
         * @param tm время
         * @return состояние, соответствующее полученной временной меткой
         */
        std::vector<double> getPathStateFromTM(double tm);

        /**
         * @brief получить состояние по времени
         * две соседние точки считаются разделёнными
         * единичным временным интервалом.
         * @param path путь
         * @param tm время
         * @return состояние, соответствующее полученной временной меткой
         */
        static std::vector<double> getPathStateFromTM(std::vector<std::vector<double>> &path, double tm);

        /**
         * проверка сцены планировщика на коллизии
         * @param state состояние
         * @return флаг, соответствует ли состояние коллизии
         */
        bool checkCollision(const std::vector<double> &state);

        /**
         * добавить объект на сцену
         * @param path путь к файлу с описанием
         */
        void addObjectToScene(std::string path);

        /**
         * Удалить объект
         * @param robotNum номер робота в списке роботов
         */
        void deleteObjectFromScene(long robotNum);

        /**
         * рассчитать общую протяжённость пути
         * @param path путь
         * @return общая протяжённость пути
         */
        static double calculatePathLength(std::vector<std::vector<double>> path);

    protected:

        /**
         * длина пути
         */
        double _pathLength{};
        /**
         * код ошибки
         */
        int _errorCode;
        /**
         * сцена
         */
        std::shared_ptr<bmpf::Scene> _scene;
        /**
         * флаг, нужно ли выводить информацию во время поиска пути
         */
        bool _showTrace;
        /**
         * коллайдер
         */
        std::shared_ptr<bmpf::Collider> _collider;
        /**
         * флаг, готов ли планировщик, в конструкторе выставляется в false;
         * бывает полезным, когда планировщику нужно подготовить
         * значительный объём ресурсов, например, коллайдер может
         * долго загружать модели
         */
        bool _ready;
        /**
         * количество потоков планировщика
         */
        bool _threadCnt;
        /**
         * конечное состояние
         */
        std::vector<double> _endState;
        /**
         * начальное состояние
         */
        std::vector<double> _startState;
        /**
         * построенный путь
         */
        std::vector<std::vector<double>> _buildedPath;
        /**
         * время начала работы планировщика
         */
        std::chrono::time_point<std::chrono::system_clock> _startTime;
        /**
         * затраченное время на планирование
         */
        double _calculationTimeInSeconds;

    public:

        /**
         * получить размер построенного пути
         * @return размер построенного пути
         */
        double getPathLength() const { return _pathLength; }

        /**
         * получить флаг, готов ли планировщик
         * @return флаг, готов ли планировщик
         */
        bool isReady() const { return _ready; }

        /**
         * получить количество потоков планировщика
         * @return количество потоков планировщика
         */
        bool getThreadCnt() const { return _threadCnt; }

        /**
         * задать флаг, готов ли планировщик
         * @param ready флаг, готов ли планировщик
         */
        void setReady(bool ready) { _ready = ready; }

        /**
         * получить начальное состояние
         * @return начальное состояние
         */
        std::vector<double> &getStartState() { return _startState; }

        /**
         * получить конечное состояние
         * @return конечное состояние
         */
        std::vector<double> &getEndState() { return _endState; }

        /**
         * получить код ошибки
         * @return код ошибки
         */
        int getErrorCode() const { return _errorCode; }

        /**
         * получить построенный путь
         * @return построенный путь
         */
        const std::vector<std::vector<double>> &getBuildedPath() const { return _buildedPath; }

        /**
         * получить сцену
         * @return сцена
         */
        const std::shared_ptr<bmpf::Scene> &getScene() const { return _scene; }

        /**
         * получить коллайдер
         * @return коллайдер
         */
        const std::shared_ptr<bmpf::Collider> &getCollider() const { return _collider; }

        /**
         * получить затраченное время на обработку
         * @return затраченное время на обработку
         */
        double getCalculationTimeInSeconds() const { return _calculationTimeInSeconds; }
    };


}