#pragma once

#include <base/path_finder.h>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <json/json.h>
#include <memory>
#include <climits>
#include <utility>
#include <log.h>
#include <base/path_finder.h>
#include <urdf_robot.h>
#include <solid_collider.h>
#include "one_direction_path_finder.h"
#include "one_direction_ordered_path_finder.h"
#include "state.h"
#include "all_directions_path_finder.h"
#include "base/path_finder.h"

/**
 * Генератор для сравнивания планировщиков
 * "all_directions" - AStar со смещением ровно вдоль всех осей
 * "one_direction" - AStar со смещением ровно вдоль одной оси
 * "ordered_one_direction" - AStar со смещением ровно вдоль одной оси,
 * смещается в первую очередь в сторону наибольшего отклонения
 * "multirobot" - режим с многими роботами
 * "continuous" - непрерывный планировщик
 */
class Generator {
public:
    /**
     * Конструктор
     * @param scenePath путь к файлу сцены,
     * @param algorithms список названий алгоритмов
     * @param gridSize размер решётки планирования
     * @param trace флаг, нужно ли выводить лог
     */
    Generator(
            const std::string &scenePath, const std::vector<std::string> &algorithms,
            unsigned int gridSize, bool trace
    );

    /**
     * построить статистику по найденным путям
     * @return статистика по найденным путям
     */
    std::string buildStat();

    /**
     * сгенерировать маршруты
     * @param testCnt количество тестов
     * @return Json запись, в которой хранится список путей, полученных от каждого планировщика
     */
    Json::Value generateRoutes(unsigned int testCnt);

    /**
     * генерирование тестов
     * @param routePath путь к сохранённым маршрутам
     * @param reportPath путь к сохранённым отчётам
     * @param testCnt количество тестов
     */
    void generate(const std::string &routePath, const std::string &reportPath, unsigned int testCnt);

    /**
     * тест поиска конкретного пути с помощью всех заданных планировщиков
     * @param start стартовое состояние
     * @param end конечное состояние
     * @return Json запись, в которой хранится список путей, полученных от каждого планировщика
     */
    Json::Value test(const std::vector<double> &start, const std::vector<double> &end);


private:
    /**
     * список планировщиков
     */
    std::vector<std::shared_ptr<bmpf::PathFinder>> _pathFinders;
    /**
     * список стартовых точек маршрутов
     */
    std::vector<std::vector<double>> _startPoints;
    /**
     * список конечных точек маршрутов
     */
    std::vector<std::vector<double>> _endPoints;
    /**
     * список списков затраченного времени (для каждого маршрута хранятся списки затраченного времени)
     */
    std::vector<std::vector<double>> _secondsList;
    /**
     * список списков флагов, получилось ли у того или иного планировщика найти путь для конкретного маршрута
     */
    std::vector<std::vector<bool>> _isValid;
    /**
     * список списков кодов ошибок для каждого теста и каждого планировщика
     */
    std::vector<std::vector<int>> _errorCodes;
};