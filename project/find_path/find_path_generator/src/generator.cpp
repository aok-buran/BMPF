#include "generator.h"
#include "multirobot_path_finder.h"
#include "continuous_path_finder.h"
#include "base/path_finder.h"

/**
 * Конструктор
 * @param scenePath путь к файлу сцены,
 * @param algorithms список названий алгоритмов
 * @param gridSize размер решётки планирования
 * @param trace флаг, нужно ли выводить лог
 */
Generator::Generator(const std::string &scenePath, const std::vector<std::string> &algorithms,
                     unsigned int gridSize, bool trace) {

    std::shared_ptr<bmpf::BaseRobot> sceneDescription;

    std::shared_ptr<bmpf::Collider> collider;

    std::shared_ptr<bmpf::Scene> sceneWrapper = std::make_shared<bmpf::Scene>();
    sceneWrapper->loadFromFile(scenePath);

    for (const std::string &algorithm: algorithms) {
        if (algorithm == "one_direction")
            _pathFinders.push_back(std::make_shared<bmpf::OneDirectionPathFinder>(
                    sceneWrapper, trace, 1000, gridSize, 5000
            ));
        else if (algorithm == "ordered_one_direction")
            _pathFinders.push_back(std::make_shared<bmpf::OneDirectionOrderedPathFinder>(
                    sceneWrapper, trace, 1000, gridSize, 5000
            ));
        else if (algorithm == "all_directions")
            _pathFinders.push_back(std::make_shared<bmpf::AllDirectionsPathFinder>(
                    sceneWrapper, trace, 1000, gridSize, 5000
            ));
        else if (algorithm == "multirobot")
            _pathFinders.push_back(std::make_shared<MultiRobotPathFinder>(
                    sceneWrapper, trace, 1000, gridSize, 5000
            ));
        else if (algorithm == "continuous")
            _pathFinders.push_back(std::make_shared<ContinuousPathFinder>(
                    sceneWrapper, trace, 1000, gridSize, 5000, 0.01
            ));
    }
}

/**
 * построить статистику по найденным путям
 * @return статистика по найденным путям
 */
std::string Generator::buildStat() {

    Json::Value json;

    for (unsigned int i = 0; i < _startPoints.size(); i++) {
        Json::Value record;
        for (unsigned int j = 0; j < _startPoints.at(i).size(); j++)
            record["start"][j] = _startPoints.at(i).at(j);
        for (unsigned int j = 0; j < _endPoints.at(i).size(); j++)
            record["end"][j] = _endPoints.at(i).at(j);
        for (int j = 0; j < _secondsList.at(i).size(); j++) {
            record["time"][j] = _secondsList.at(i).at(j);
            record["isValid"][j] = _isValid.at(i).at(j) ? 1 : 0;
        }
        std::cout << std::endl;
        json[i] = record;
    }

    Json::Value result;

    char buf[PATH_MAX + 1]; /* not sure about the "+ 1" */
    char *res = realpath(_pathFinders.front()->getScene()->getScenePath().c_str(), buf);
    if (res) {
        result["scene"] = buf;
    } else {
        perror("realpath");
        exit(EXIT_FAILURE);
    }

    result["data"] = json;
    long validCnt = 0;
    long nonValidCnt = 0;
    for (auto &i: _isValid) {
        validCnt += std::count(i.begin(), i.end(), true);
        nonValidCnt += std::count(i.begin(), i.end(), false);
    }
    long expCnt = _isValid.front().size();

    Json::Value agregated;

    agregated["validCnt"] = (int) validCnt;
    agregated["nonValidCnt"] = (int) nonValidCnt;
    agregated["expCnt"] = (int) expCnt;

    result["agregated"] = agregated;

    return result.toStyledString();

}

/**
 * сгенерировать маршруты
 * @param testCnt количество тестов
 * @return Json запись, в которой хранится список путей, полученных от каждого планировщика
 */
Json::Value Generator::generateRoutes(unsigned int testCnt) {
    assert(_pathFinders.front());
    assert(_pathFinders.front()->getScene());


    Json::Value json;
    _startPoints.clear();
    _endPoints.clear();
    _secondsList.clear();
    _isValid.clear();

    for (int i = 0; i < testCnt; i++) {
        bmpf::infoMsg("generate routes ", i);
        std::vector<double> start = _pathFinders.front()->getRandomState();
        std::vector<double> end = _pathFinders.front()->getRandomState();
        json[i] = test(start, end);
    }

    Json::Value result;

    char buf[PATH_MAX + 1]; /* not sure about the "+ 1" */
    char *res = realpath(_pathFinders.front()->getScene()->getScenePath().c_str(), buf);
    if (res) {
        result["scene"] = buf;
    } else {
        perror("realpath");
        exit(EXIT_FAILURE);
    }

    result["data"] = json;

    return result;
}

/**
 * генерирование тестов
 * @param routePath путь к сохранённым маршрутам
 * @param reportPath путь к сохранённым отчётам
 * @param testCnt количество тестов
 */
void Generator::generate(const std::string &routePath, const std::string &reportPath, unsigned int testCnt) {
    auto result = generateRoutes(testCnt);
    std::ofstream myfile;
    myfile.open(routePath);
    myfile << result.toStyledString();
    myfile.close();

    if (!reportPath.empty()) {
        std::string stat = buildStat();
        myfile.open(reportPath);
        myfile << stat;
        myfile.close();

    }
}

/**
 * тест поиска конкретного пути с помощью всех заданных планировщиков
 * @param start стартовое состояние
 * @param end конечное состояние
 * @return Json запись, в которой хранится список путей, полученных от каждого планировщика
 */
Json::Value Generator::test(const std::vector<double> &start, const std::vector<double> &end) {
    _startPoints.emplace_back(start);
    _endPoints.emplace_back(end);

    bmpf::infoMsg("test path finding");
    Json::Value json;

    using namespace std::chrono;

    std::vector<double> secondsLst;
    std::vector<bool> validLst;
    std::vector<int> errorLst;


    for (int i = 0; i < _pathFinders.size(); i++) {
        int errorCode = bmpf::PathFinder::NO_ERROR;

        auto startTime = high_resolution_clock::now();

        std::vector<std::vector<double>> path =
                _pathFinders.at(i)->findPath(start, end, errorCode);

        auto endTime = high_resolution_clock::now();
        double seconds = (double) duration_cast<milliseconds>(endTime - startTime).count() / 1000;
        bmpf::infoMsg(" pf took ", seconds, " seconds, error code: ", _pathFinders.at(i)->getErrorCode());

        secondsLst.emplace_back(seconds);

        bool isPathValid;
        if (errorCode == bmpf::PathFinder::NO_ERROR) {
            isPathValid = _pathFinders.front()->simpleCheckPath(path, 100);
            json[i] = bmpf::PathFinder::getJSONPath(path);
            if (!isPathValid)
                errorCode = bmpf::PathFinder::ERROR_CAN_NOT_FIND_PATH;
        } else {
            isPathValid = false;
            json[i] = bmpf::PathFinder::getJSONPath(
                    std::vector<std::vector<double >>{
                            start, end
                    }
            );
        }
        validLst.emplace_back(isPathValid);
        if (!isPathValid) {
            bmpf::errMsg("path is not valid");
            bmpf::infoState("start state", start);
            bmpf::infoState("end state", end);
        }

        errorLst.emplace_back(errorCode);

    }

    _errorCodes.emplace_back(errorLst);
    _isValid.emplace_back(validLst);
    _secondsList.push_back(secondsLst);


    return json;
}
