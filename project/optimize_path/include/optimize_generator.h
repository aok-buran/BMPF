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
#include "optimize_path.h"

/**
 * Генератор для сравнивания оптимизаторов
 * "median" - медианный оптимизатор
 * "newton" - оптимизатор алгоритмом Ньютона
 */
class OptimizeGenerator {
public:
    /**
    * Конструктор
    * @param expPathsFileNames список путей к данным
    * @param algorithms список названий алгоритмовы
    */
    OptimizeGenerator(
            const std::vector<std::string> &expPathsFileNames, const std::vector<std::string> &algorithms
    );

    /**
     * построить статистику по оптимизированным путям
     * @return статистика по оптимизированным путям
     */
    std::string buildStat();

    /**
     * сгенерировать оптимизированне пути
     * @return JSON-представление оптимизированных маршрутов
     */
    Json::Value optimizeRoutes();

    /**
     * генерирование тестов
     * @param routePath путь к сохранённым маршрутам
     * @param reportPath путь к сохранённым отчётам
     */
    void generate(const std::string &routePath, const std::string &reportPath);

    /**
     * тест поиска конкретного пути разными планировщиками
     * @param pathOptimizer оптимизатор пути
     * @param path путь
     * @return JSON-представление оптимизированных маршрутов
     */
    Json::Value test(
            const std::shared_ptr<bmpf::PathOptimizer> &pathOptimizer, const std::vector<std::vector<double>> &path
    );


private:
    /**
     * список путей к данным
     */
    std::vector<std::string> _expPathsFileNames;
    /**
     * Планировщик пути
     */
    std::shared_ptr<bmpf::PathFinder> _pathFinder;
    /**
     * Список оптимизаторов пути
     */
    std::vector<std::shared_ptr<bmpf::PathOptimizer>> _pathOptimizers;
    /**
     * список затраченного времени (для каждого маршрута хранятся списки затраченного времени)
     */
    std::vector<double> _secondsList;
    /**
     * список длин стартовых маршрутов
     */
    std::vector<double> _initialPathLengths;
    /**
     *  список длин оптимизированных маршрутов
     */
    std::vector<double> _optimizedPathLengths;
    /**
     * список длин оптимизированных маршрутов
     */
    std::vector<std::string> _optimizeMethods;
    /**
     * Словарь для связи оптимизаторов и путей, с которыми им нужно работать
     */
    std::unordered_map<
            std::shared_ptr<bmpf::PathOptimizer>, std::vector<std::vector<std::vector<double>>>
    > pfPathsLists;
};