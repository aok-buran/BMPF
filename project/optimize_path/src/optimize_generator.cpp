#include "optimize_generator.h"
#include "optimize_path_median.h"
#include "optimize_path_newton.h"


/**
* Конструктор
* @param expPathsFileNames список путей к данным
* @param algorithms список названий алгоритмовы
*/
OptimizeGenerator::OptimizeGenerator(
        const std::vector<std::string> &expPathsFileNames, const std::vector<std::string> &algorithms
) {

    _expPathsFileNames = expPathsFileNames;

    for (const std::string &expPathsFileName: _expPathsFileNames) {
        for (const std::string &algorithm: algorithms) {

            std::string scenePath;
            auto paths = bmpf::PathFinder::loadPathsFromFile(expPathsFileName, scenePath);

            std::shared_ptr<bmpf::Scene> sceneWrapper = std::make_shared<bmpf::Scene>();
            sceneWrapper->loadFromFile(scenePath);

            _pathFinder =
                    std::make_shared<bmpf::OneDirectionPathFinder>(sceneWrapper, true, 100, 10, 4000, 1, 2);

            if (algorithm == "median") {
                for (int i = 1; i <= 7; i += 2)
                    for (int j = 1; j <= 7; j += 2) {
                        std::ostringstream stringStream;
                        stringStream << algorithm << "_" << i << "_" << j;

                        auto optimizer = std::make_shared<bmpf::MedianPathOptimizer>(
                                _pathFinder, stringStream.str(), 100, i, j
                        );
                        pfPathsLists.insert({optimizer, paths});
                        _pathOptimizers.push_back(optimizer);

                    }
            } else if (algorithm == "newton") {
                _pathOptimizers.push_back(std::make_shared<bmpf::NewtonPathOptimizer>(_pathFinder, algorithm));
            }

        }
    }

}

/**
 * генерирование тестов
 * @param routePath путь к сохранённым маршрутам
 * @param reportPath путь к сохранённым отчётам
 */
void OptimizeGenerator::generate(const std::string &routePath, const std::string &reportPath) {
    auto routes = optimizeRoutes();
    std::ofstream myfile;
    myfile.open(routePath);
    myfile << routes.toStyledString();
    myfile.close();

    if (!reportPath.empty()) {
        std::string stat = buildStat();
        myfile.open(reportPath);
        myfile << stat;
        myfile.close();
    }
}

/**
 * сгенерировать оптимизированне пути
 * @return JSON-представление оптимизированных маршрутов
 */
Json::Value OptimizeGenerator::optimizeRoutes() {

    _initialPathLengths.clear();
    _optimizedPathLengths.clear();
    _secondsList.clear();
    _optimizeMethods.clear();


    Json::Value json;

    int pathPos = 0;
    for (auto &_pathOptimizer: _pathOptimizers) {
        Json::Value jsonRow;
        auto paths = pfPathsLists.at(_pathOptimizer);
        for (auto &path: paths) {
            json[pathPos++] = test(_pathOptimizer, path);
        }
    }

    return json;
}

/**
 * тест поиска конкретного пути разными планировщиками
 * @param pathOptimizer оптимизатор пути
 * @param path путь
 * @return JSON-представление оптимизированных маршрутов
 */
Json::Value OptimizeGenerator::test(
        const std::shared_ptr<bmpf::PathOptimizer> &pathOptimizer, const std::vector<std::vector<double>> &path
) {
    bmpf::infoMsg("test path optimizing ", pathOptimizer->getPathOptimizeMethod());

    auto startTime = std::chrono::high_resolution_clock::now();

    auto optimizedPath = pathOptimizer->optimizePath(path);

    auto endTime = std::chrono::high_resolution_clock::now();
    double seconds = (double) std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() / 1000;

    bmpf::infoMsg("optimizing took ", seconds, "\n");


    _secondsList.emplace_back(seconds);


    _initialPathLengths.push_back(pathOptimizer->getPathFinder()->calculatePathLength(path));

    _optimizedPathLengths.push_back(
            pathOptimizer->getPathFinder()->calculatePathLength(optimizedPath));

    _optimizeMethods.push_back(pathOptimizer->getPathOptimizeMethod());

    return bmpf::PathFinder::getJSONPath(optimizedPath);

}


/**
 * построить статистику по оптимизированным путям
 * @return статистика по оптимизированным путям
 */
std::string OptimizeGenerator::buildStat() {
    Json::Value json;

    for (int i = 0; i < _pathOptimizers.size(); i++) {
        Json::Value record;
        record["initialPathLength"] = _initialPathLengths.at(i);
        record["optimizedPathLength"] = _optimizedPathLengths.at(i);
        record["time"] = _secondsList.at(i);
        record["method"] = _optimizeMethods.at(i);
        record["id"] = i;
        json[i] = record;
    }

    return json.toStyledString();

}