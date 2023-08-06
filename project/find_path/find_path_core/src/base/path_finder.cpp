#include "base/path_finder.h"


using namespace bmpf;

/**
 * конструктор
 * @param scene сцена
 * @param showTrace флаг, нужно ли выводить информацию во время поиска пути
 * @param threadCnt количество потоков планировщика
 */
PathFinder::PathFinder(const std::shared_ptr<bmpf::Scene> &scene, bool showTrace, int threadCnt) {
    if (!scene) {
        throw std::runtime_error("PathFinder::PathFinder() ERROR: scene is null");
    }

    _ready = false;
    _threadCnt = threadCnt;
    if (threadCnt != 1)
        _collider = std::make_shared<SolidSyncCollider>(threadCnt);
    else
        _collider = std::make_shared<SolidCollider>();

    _collider->init(scene->getGroupedModelPaths(), false);

    _scene = scene;
    _showTrace = showTrace;

    for (unsigned long i = 0; i < _scene->getJointCnt(); i++) {
        _startState.emplace_back(0.0);
        _endState.emplace_back(0.0);
    }

    _calculationTimeInSeconds = -1;
    _errorCode = NO_ERROR;
}


/**
 * Поиск пути
 * @param startState стартовое состояние
 * @param endState конечное состояние
 * @param errorCode в эту переменную записывается код ошибки
 * @return построенный путь
 */
std::vector<std::vector<double>>
PathFinder::findPath(const std::vector<double> &startState, const std::vector<double> &endState, int &errorCode) {

    _startState = startState;
    _endState = endState;

    _startTime = std::chrono::high_resolution_clock::now();

    prepare(startState, endState);

    if (_errorCode != NO_ERROR) {
        errorCode = _errorCode;
        return {};
    }

    std::vector<double> actualState;

    // если очередной такт поиска пути не последний
    while (!findTick(actualState)) {};

    if (_errorCode != NO_ERROR)
        return {};

    // строим путь
    buildPath();

    auto endTime = std::chrono::high_resolution_clock::now();
    _calculationTimeInSeconds =
            (double) std::chrono::duration_cast<std::chrono::milliseconds>(endTime - _startTime).count() / 1000;

    errorCode = _errorCode;

    return _buildedPath;
}

/**
 * добавить объект на сцену
 * @param path путь к файлу с описанием
 */
void PathFinder::addObjectToScene(std::string path) {
    _scene->addObject(std::move(path));
    _collider->init(_scene->getGroupedModelPaths(), false);
}

/**
 * Удалить объект
 * @param robotNum номер робота в списке роботов
 */
void PathFinder::deleteObjectFromScene(long robotNum) {
    _scene->deleteRobot(robotNum);
    _collider->init(_scene->getGroupedModelPaths(), false);
}

/**
 * @brief проверяет доступность состояния
 * Проверяет доступность углов, после проверяет состояние на коллизии
 * @param state состояние
 * @return флаг, допустимо ли состояние
 */
bool PathFinder::checkState(const std::vector<double> &state) {
    if (!_scene->isStateEnabled(state))
        return false;
    return !_collider->isCollided(_scene->getTransformMatrices(state));
}

/**
 * обновить коллайдер по сцене
 */
void PathFinder::updateCollider() {
    _collider->init(_scene->getGroupedModelPaths(), false);
}

/**
 * проверить отрезок с промежуточными точками на коллизии
 * @param prevPoint первая точка отрезка
 * @param nextPoint вторая точка отрезка
 * @param checkCnt количество промежуточных точек
 * @return флаг, является ли отрезок безколлизионным
 */
bool PathFinder::divideCheckPathSegment(
        const std::vector<double> &prevPoint, std::vector<double> nextPoint, int checkCnt
) {
    double checkStep = 1.0 / checkCnt;

    std::vector<double> delta = mulState(subtractStates(std::move(nextPoint), prevPoint), checkStep);

    std::vector<double> currentPoint = prevPoint;
    for (int i = 0; i < checkCnt; i++) {
        if (!checkState(currentPoint))
            return false;

        currentPoint = sumStates(currentPoint, delta);
    }
    return checkState(currentPoint);
}

/**
 * проверить путь с промежуточными точками на коллизии
 * @param path путь
 * @param checkCnt  количество промежуточных точек
 * @return флаг, является ли путь безколлизионным
 */
int PathFinder::divideCheckPath(std::vector<std::vector<double>> path, int checkCnt) {
    for (int i = 1; i < path.size(); i++) {
        std::vector<double> prevPoint = path.at(i - 1);
        std::vector<double> nextPoint = path.at(i);
        if (!divideCheckPathSegment(prevPoint, nextPoint, checkCnt))
            return i - 1;
    }
    return NO_ERROR;
}

/**
 * проверить опорные точки пути на коллизии
 * @param path путь
 * @param maxDist максимальное расстояние между опорными точками
 * @return флаг, является ли путь безколлизионным
 */
bool PathFinder::simpleCheckPath(const std::vector<std::vector<double>> &path, double maxDist) {
    // проверка расстояний между соседними точками
    for (int i = 1; i < path.size(); i++) {
        std::vector<double> a = path.at(i - 1);
        std::vector<double> b = path.at(i);
        if (getSqrDistance(a, b) > maxDist)
            return false;
    }

    // проверка точек
    for (const auto &i: path)
        if (!checkState(i))
            return false;

    return true;
}

/**
 * получить случайное разрешённое состояние
 * @return случайное разрешённое состояние
 */
std::vector<double> PathFinder::getRandomState() {
    std::vector<double> state;
    do {
        state = _scene->getRandomState();
    } while (!checkState(state));
    return state;
}

/**
 * проверка сцены планировщика на коллизии
 * @param state состояние
 * @return флаг, соответствует ли состояние коллизии
 */
bool PathFinder::checkCollision(const std::vector<double> &state) {
    return _collider->isCollided(_scene->getTransformMatrices(state));
}

/**
 * рисование сцены планировщика
 * @param state состояние
 * @param onlyRobot флаг, нужно ли рисовать только роботов
 */
void PathFinder::paint(const std::vector<double> &state, bool onlyRobot) {
    _collider->paint(_scene->getTransformMatrices(state), onlyRobot);
}

/**
 * @brief получить состояние по времени
 * две соседние точки считаются разделёнными
 * единичным временным интервалом.
 * @param tm время
 * @return состояние, соответствующее полученной временной меткой
 */
std::vector<double> PathFinder::getPathStateFromTM(double tm) {
    if (tm < 0) {
        char buf[1024];
        sprintf(buf,
                "athFinder::getPathStateFromTM() ERROR: \n tm is %f, but if must be positive"
                "\nthey must be equal", tm
        );
        throw std::invalid_argument(buf);
    }

    return getPathStateFromTM(_buildedPath, tm);
}

/**
 * @brief получить состояние по времени
 * две соседние точки считаются разделёнными
 * единичным временным интервалом.
 * @param path путь
 * @param tm время
 * @return состояние, соответствующее полученной временной меткой
 */
std::vector<double> PathFinder::getPathStateFromTM(std::vector<std::vector<double>> &path, double tm) {
    if (tm < 0) {
        char buf[1024];
        sprintf(buf,
                "athFinder::getPathStateFromTM() ERROR: \n tm is %f, but if must be positive"
                "\nthey must be equal", tm
        );
        throw std::invalid_argument(buf);
    }

    auto pos = (unsigned long) tm;

    if (pos > path.size() - 2) {
        return path.back();
    }

    auto prevState = path.at(pos);
    auto nextState = path.at(pos + 1);

    auto delta = subtractStates(nextState, prevState);
    return sumStates(prevState, mulState(delta, tm - pos));
}

/**
 * рассчитать общую протяжённость пути
 * @param path путь
 * @return общая протяжённость пути
 */
double PathFinder::calculatePathLength(std::vector<std::vector<double>> path) {
    double pathLength = 0;
    for (int i = 0; i < path.size() - 1; i++) {
        pathLength += getStateDistance(path.at(i), path.at(i + 1));
    }
    return pathLength;
}

/**
 * сохранить путь в файл
 * @param path путь
 * @param filename путь к файлу
 */
void PathFinder::savePathToFile(std::vector<std::vector<double>> path, const std::string &filename) {
    std::string jsonRepresentation = getJSONPath(std::move(path)).toStyledString();

    std::ofstream ofs;
    ofs.open(filename.c_str(), std::ios::out | std::ios::binary);
    ofs.write(jsonRepresentation.c_str(), (int) jsonRepresentation.length());
    ofs.close();
}

/**
 * Загрузить список путей из файла
 * @param filename путь к файлу с путями
 * @param scenePath путь к сцене (читается из файла, сохраняется в scenePath)
 * @return список путей
 */
std::vector<std::vector<std::vector<double>>>
PathFinder::loadPathsFromFile(const std::string &filename, std::string &scenePath) {
    std::vector<std::vector<std::vector<double>>> pathStates;
    Json::Reader reader;
    Json::Value obj;

    std::ifstream ifs((filename).c_str(), std::ios_base::binary);

    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    reader.parse(content, obj);
    scenePath = obj["scene"].asString();

    const Json::Value &data = obj["data"];

    pathStates.clear();

    for (Json::Value path: data) {
        std::vector<std::vector<double>> oneExpPathStates;
        for (Json::Value record: path[0]["states"]) {
            std::vector<double> state;
            for (const Json::Value &coord: record["state"])
                state.emplace_back(coord.asDouble());

            oneExpPathStates.emplace_back(state);
        }
        pathStates.emplace_back(oneExpPathStates);
    }
    return pathStates;
}


/**
 * Загрузить список путь из файла
 * @param filename путь к файлу с путями
 * @param scenePath путь к сцене (читается из файла, сохраняется в scenePath)
 * @return список путей
 */
std::vector<std::vector<double>> PathFinder::loadPathFromFile(const std::string &filename, std::string &scenePath) {
    std::ifstream ifs(filename, std::ios_base::binary);

    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    Json::Reader reader;
    Json::Value obj;

    reader.parse(content, obj);

    scenePath = obj["scene"].asString();

    return getPathFromJSON(obj["states"]);
}


/**
 * Получить путь из json-представления
 * @param json json-представление
 * @return путь
 */
std::vector<std::vector<double>> PathFinder::getPathFromJSON(const Json::Value &json) {
    std::vector<std::vector<double>> path;
    for (const Json::Value &state: json) {
        std::vector<double> curState;
        for (const Json::Value &coord: state)
            curState.emplace_back(coord.asDouble());

        path.emplace_back(curState);
    }

    return path;
}


/**
 * Получить json-представление пути
 * @param path путь
 * @return json-представление
 */
Json::Value PathFinder::getJSONPath(std::vector<std::vector<double>> path) {
    Json::Value json;
    Json::Value pointArr;

    for (unsigned int i = 0; i < path.size(); i++) {
        Json::Value point;
        Json::Value state;
        for (unsigned int j = 0; j < path.at(i).size(); j++) {
            state[j] = path.at(i).at(j);
        }
        point["state"] = state;
        pointArr[i] = point;
    }
    json["states"] = pointArr;

    return json;
}

/**
 * вывести на консоль путь
 * @param path путь
 */
void PathFinder::infoPath(const std::vector<std::vector<double>> &path) {
    for (const auto &state: path)
        bmpf::infoState("path_finding node", state);
}

/**
 * разбить путь на части (partCnt - кол-во частей)
 * @param path путь
 * @param partCnt количество разбиений каждого этапа
 * @return новый путь с промежуточными точками
 */
std::vector<std::vector<double>>
PathFinder::splitPath(std::vector<std::vector<double>> path, unsigned long partCnt) {
    double step = 1.0 / partCnt;
    std::vector<std::vector<double>> resPath;

    for (unsigned int i = 1; i < path.size(); i++) {
        std::vector<double> prevPoint = path.at(i - 1);
        std::vector<double> nextPoint = path.at(i);
        std::vector<double> delta = bmpf::mulState(bmpf::subtractStates(nextPoint, prevPoint), step);

        for (unsigned int j = 0; j < partCnt; j++) {
            std::vector<double> s = bmpf::mulState(delta, j);
            resPath.push_back(bmpf::sumStates(prevPoint, s));
        }
        resPath.push_back(nextPoint);
    }
    resPath.push_back(*std::prev(path.end()));

    return resPath;
}

