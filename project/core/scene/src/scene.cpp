#include "scene.h"
#include <urdf_robot.h>
#include <solid_collider.h>
#include <fstream>
#include <utility>
#include <json/json.h>


using namespace bmpf;

/**
 * конструктор сцены от одного робота
 * @param robot робот
 */
Scene::Scene(const std::vector<std::shared_ptr<bmpf::BaseRobot>> &robot) {
    _objects.insert(std::end(_objects), std::begin(robot), std::end(robot));
    _initObjects();
}

/**
 * @brief посчитать количество активных роботов
 * посчитать количество активных роботов на сцене (объектов с переменными состояниями)
 * @return количество активных роботов на сцене
 */
unsigned long Scene::getActiveRobotCnt() const {
    unsigned long robotCnt = 0;
    for (const auto &object: _objects)
        if (object->getJointCnt() != 0)
            robotCnt++;
    return robotCnt;
}


/**
 * объединить состояния роботов (последовательно)
 * @param states список состояний
 * @return объединённое состояние
 */
std::vector<double> Scene::concatenateStates(std::initializer_list<std::vector<double>> states) {
    std::vector<double> res;
    for (auto st: states)
        res.insert(res.end(), st.begin(), st.end());
    return res;
}

/**
 * объединить состояния роботов (последовательно)
 * @param states список состояний
 * @return объединённое состояние
 */
std::vector<double> Scene::concatenateStates(const std::vector<std::vector<double>> &states) {
    std::vector<double> res;
    for (auto st: states)
        res.insert(res.end(), st.begin(), st.end());
    return res;
}

/**
 * объединить координаты роботов
 * @param coords координаты
 * @return объединённый список координат
 */
std::vector<int> Scene::concatenateCoords(const std::vector<std::vector<int>> &coords) {
    std::vector<int> res;
    for (auto st: coords)
        res.insert(res.end(), st.begin(), st.end());
    return res;
}

/**
 * Добавить робота на сцену
 * @param path путь к описанию робота
 * @return id добавленного робота в общем списке роботов
 */
unsigned long Scene::addObject(std::string path) {
    std::shared_ptr<bmpf::BaseRobot> robot = std::make_shared<bmpf::URDFRobot>();
    robot->loadFromFile(std::move(path));
    std::vector<std::shared_ptr<bmpf::BaseRobot>> lst = {robot};
    _objects.emplace_back(robot);
    if (_objects.back()->getJointCnt() != 0) {
        _areObjectsJointed.emplace_back(true);
        _jointedObjectIndexes.push_back(_objects.size() - 1);
    } else {
        _areObjectsJointed.emplace_back(false);
        _notJointedObjectIndexes.push_back(_objects.size() - 1);
    }

    _initObjects();
    return _objects.size() - 1;
}

/**
 * добавить робота на сцену
 * @param path путь к описанию робота,
 * @param transformVector вектор трансформации position: x,y,z; rotation: r,p,y; scale: x,y,z
 * @return id добавленного робота в общем списке роботов
 */
unsigned long Scene::addObject(std::string path, std::vector<double> &transformVector) {
    unsigned long objectNum = addObject(std::move(path));
    _objects.at(objectNum)->setWorldTransformVector(transformVector);
    return objectNum;
}

/**
 * заполняет по имеющимся на сцене роботам общие списки jointCnt, _jointIndexRanges, _jointParams и _links
 */
void Scene::_initObjects() {
    _jointCnt = 0;
    _jointIndexRanges.clear();
    _jointParams.clear();
    _links.clear();
    _singleRobotScenes.clear();

    for (int i = 0; i < _objects.size(); i++) {
        auto robot = _objects.at(i);
        std::vector<std::shared_ptr<bmpf::JointParams>> rjps = robot->getJointParamsList();
        std::vector<double> rjms = robot->getMaxSpeedModules();
        std::vector<double> rjma = robot->getMaxAccelerationModules();

        std::pair<long, long> range{_jointParams.size(), robot->getJointCnt() + _jointParams.size() - 1};
        _jointIndexRanges.emplace_back(range);

        _jointParams.insert(std::end(_jointParams), std::begin(rjps), std::end(rjps));
        _maxSpeedModules.insert(std::end(_maxSpeedModules), std::begin(rjms), std::end(rjms));
        _maxAccelerationModules.insert(std::end(_maxAccelerationModules), std::begin(rjma), std::end(rjma));

        std::vector<std::shared_ptr<bmpf::Link>> rls = robot->getLinks();
        _links.insert(std::end(_links), std::begin(rls), std::end(rls));

        if (!_jointedObjectIndexes.empty() && robot->getJointCnt() != 0) {
            std::vector<std::shared_ptr<bmpf::BaseRobot>> objectLst;
            objectLst.push_back(_objects.at(i));
            for (int index: _notJointedObjectIndexes) {
                objectLst.push_back(_objects.at(index));
            }
            _singleRobotScenes.push_back(std::make_shared<Scene>(objectLst));
        }
    }
    _jointCnt = _jointParams.size();
}

/**
 * удалить робота со сцены
 * @param robotNum номер робота в общем списке
 */
void Scene::deleteRobot(long robotNum) {
    _objects.erase(_objects.begin() + robotNum);
    _initObjects();
}

/**
 * сохранить в файл
 * @param path
 */
void Scene::saveToFile(const std::string &path) {
    // infoMsg("scene description save scene, path ", path);
    Json::Value json;

    json["name"] = "saved scene";

    Json::Value modelArr;

    for (unsigned int i = 0; i < _objects.size(); i++) {
        const auto &sceneDescription = _objects.at(i);
        for (unsigned int j = 0; j < 3; j++) {
            modelArr[i]["pos"][j] = sceneDescription->getWorldTranslation().at(j);
            modelArr[i]["rpy"][j] = sceneDescription->getWorldRotation().at(j);
            modelArr[i]["scale"][j] = sceneDescription->getWorldScale().at(j);
        }
        modelArr[i]["model"] = sceneDescription->getPath();
    }
    json["robots"] = modelArr;

    std::ofstream myfile;
    myfile.open(path);
    myfile << json.toStyledString();
    myfile.close();

}

/**
 * загрузить из файла
 * @param path путь к описанию сцены
 */
void Scene::loadFromFile(const std::string &path) {

    _path = path;
    _objects.clear();

    Json::Reader reader;
    Json::Value obj;

    std::ifstream ifs(path.c_str(), std::ios_base::binary);
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    if (content.empty()) {
        char buf[1024];
        sprintf(buf,
                "Scene::loadFromFile() ERROR: \n file content is empty"
        );
        throw std::runtime_error(buf);
    }

    reader.parse(content, obj);
    // infoMsg("Load scene: ", obj["name"].asString());
    // список роботов сцены
    const Json::Value &sceneData = obj["robots"];

    // получаем путь к папке с параметрами сцены
    std::string subPath;
    int p = (int) _path.find_last_of('/');
    if (p != std::string::npos)
        subPath = _path.substr(0, p + 1);
    else
        subPath = "";

    // заполняем пути к моделям
    for (auto sceneObject: sceneData) {
        std::string modelPath = sceneObject["model"].asString();
        std::shared_ptr<Eigen::Matrix4d> localMatrix = std::make_shared<Eigen::Matrix4d>();

        std::vector<double> pose{
                sceneObject["pos"][0].asDouble(), sceneObject["pos"][1].asDouble(), sceneObject["pos"][2].asDouble(),
                sceneObject["rpy"][0].asDouble(), sceneObject["rpy"][0].asDouble(), sceneObject["rpy"][0].asDouble(),
                sceneObject["scale"][0].asDouble(), sceneObject["scale"][0].asDouble(),
                sceneObject["scale"][0].asDouble()
        };

        addObject(subPath + modelPath, pose);
    }
}

/**
 * получить состояние робота
 * @param state состояние сцены
 * @param objectNum номер робота в списке роботов
 * @return состояние робота
 */
std::vector<double> Scene::getSingleObjectState(const std::vector<double> &state, unsigned long objectNum) {
    std::pair<long, long> range = _jointIndexRanges.at(objectNum);
    std::vector<double> singleObjectState(
            std::begin(state) + range.first, std::begin(state) + range.second + 1
    );
    return singleObjectState;
}

/**
 * @brief получить список матриц преобразований всех звеньев
 * получить список матриц преобразований всех звеньев по состоянию сцены
 * @param state состояние
 * @return список матриц преобразований
 */
std::vector<Eigen::Matrix4d> Scene::getTransformMatrices(const std::vector<double> &state) {
    if (state.empty()) {
        char buf[1024];
        sprintf(buf,
                "Scene::getTransformMatrices() ERROR: \n state is empty"
        );
        throw std::invalid_argument(buf);
    }

    std::vector<Eigen::Matrix4d> matrices;
    for (unsigned long i = 0; i < _objects.size(); i++) {
        std::vector<double> localState = getSingleObjectState(state, i);
        std::vector<Eigen::Matrix4d> localMatrices = _objects.at(i)->getTransformMatrices(localState);
        matrices.insert(std::end(matrices), std::begin(localMatrices), std::end(localMatrices));
    }
    return matrices;
}

/**
 * получить матрицы преобразования из СК базы робота в СК рабочего инструмента для каждого робота сцены
 * @param state  состояние сцены
 * @return матрицы преобразования
 */
std::vector<Eigen::Matrix4d> Scene::getEndEffectorTransformMatrices(const std::vector<double> &state) {
    std::vector<Eigen::Matrix4d> matrices;
    for (unsigned long i = 0; i < _objects.size(); i++)
        if (_objects.at(i)->getJointCnt() > 0) {
            std::vector<double> localState = getSingleObjectState(state, i);
            Eigen::Matrix4d m = _objects.at(i)->getEndEffectorTransformMatrix(localState);
            matrices.emplace_back(m);
        }
    return matrices;
}

/**
 * получить частные производные матриц преобразования из СК базы робота в СК рабочего инструмента
 * для каждого робота сцены по i-ой координате
 * @param state  состояние сцены
 * @param iVal индекс координаты, по которой берётся производная
 * @return частные производные матриц преобразования
 */
std::vector<Eigen::Matrix4d> Scene::getEndEffectorDiffTransformMatrices(const std::vector<double> &state, int iVal) {
    std::vector<Eigen::Matrix4d> matrices;
    for (unsigned long i = 0; i < _objects.size(); i++)
        if (_objects.at(i)->getJointCnt() > 0) {
            std::vector<double> localState = getSingleObjectState(state, i);
            Eigen::Matrix4d m = _objects.at(i)->getEndEffectorDiffTransformMatrix(localState, iVal);
            matrices.emplace_back(m);
        }
    return matrices;
}

/**
 * получить частные производные матриц преобразования из СК базы робота в СК рабочего инструмента для каждого робота сцены
 * по i-ой координате и j-ой координате
 * @param state  состояние сцены
 * @param iVal индекс координаты, по которой первый раз берётся производная
 * @param jVal индекс координаты, по которой второй раз берётся производная
 * @return частные производные матриц преобразования
 */
std::vector<Eigen::Matrix4d>
Scene::getEndEffectorDiff2TransformMatrices(const std::vector<double> &state, int iVal, int jVal) {
    std::vector<Eigen::Matrix4d> matrices;
    for (unsigned long i = 0; i < _objects.size(); i++)
        if (_objects.at(i)->getJointCnt() > 0) {
            std::vector<double> localState = getSingleObjectState(state, i);
            Eigen::Matrix4d m = _objects.at(i)->getEndEffectorDiff2TransformMatrix(localState, iVal, jVal);
            matrices.emplace_back(m);
        }
    return matrices;
}

/**
 * @brief получить список положений (x,y,z) рабочих инструментов всех роботов
 * получить список положений (x,y,z) рабочих инструментов всех роботов по состоянию сцены
 * @param state состояние сцены
 * @return список положений
 */
std::vector<double> Scene::getEndEffectorPositions(const std::vector<double> &state) {
    std::vector<double> positions;
    for (unsigned long i = 0; i < _objects.size(); i++)
        if (_objects.at(i)->getJointCnt() > 0) {
            std::vector<double> localState = getSingleObjectState(state, i);
            std::vector<double> pos = _objects.at(i)->getEndEffectorPos(localState);
            positions.insert(positions.end(), pos.begin(), pos.end());
        }
    return positions;
}


/**
 * Получить частную производную положений рабочих инструментов роботов по i-ой координате
 * @param state состояние сцены
 * @param iVal индекс координаты, по которой берётся производная
 * @return  частная производная положений рабочих инструментов
 */
std::vector<double> Scene::getEndEffectorDiffPositions(const std::vector<double> &state, int iVal) {
    std::vector<double> positions;
    for (unsigned long i = 0; i < _objects.size(); i++)
        if (_objects.at(i)->getJointCnt() > 0) {
            std::vector<double> localState = getSingleObjectState(state, i);
            std::vector<double> pos;
            if (_jointIndexRanges.at(i).first <= iVal && iVal <= _jointIndexRanges.at(i).second)
                pos = _objects.at(i)->getEndEffectorDiffPos(localState, iVal - _jointIndexRanges.at(i).first);
            else
                pos = _objects.at(i)->getEndEffectorPos(localState);

            positions.insert(positions.end(), pos.begin(), pos.end());
        }
    return positions;

}

/**
 * получить частную производную положений рабочих инструментов роботов по i-ой и j-ой координатам
 * @param state состояние сцены
 * @param iVal индекс координаты, по которой первый раз берётся производная
 * @param jVal индекс координаты, по которой второй раз берётся производная
 * @return  частная производная положений рабочих инструментов
 */
std::vector<double> Scene::getEndEffectorDiff2Positions(const std::vector<double> &state, int iVal, int jVal) {
    std::vector<double> positions;
    for (unsigned long i = 0; i < _objects.size(); i++)
        if (_objects.at(i)->getJointCnt() > 0) {
            std::vector<double> localState = getSingleObjectState(state, i);
            std::vector<double> pos;
            if (_jointIndexRanges.at(i).first <= iVal && iVal <= _jointIndexRanges.at(i).second) {
                if (_jointIndexRanges.at(i).first <= jVal && jVal <= _jointIndexRanges.at(i).second) {
                    pos = _objects.at(i)->getEndEffectorDiff2Pos(
                            localState, iVal - _jointIndexRanges.at(i).first, jVal - _jointIndexRanges.at(i).first
                    );
                } else
                    pos = _objects.at(i)->getEndEffectorDiffPos(localState, iVal - _jointIndexRanges.at(i).first);
            } else if (_jointIndexRanges.at(i).first <= jVal && jVal <= _jointIndexRanges.at(i).second) {
                pos = _objects.at(i)->getEndEffectorDiffPos(localState, jVal - _jointIndexRanges.at(i).first);
            }
            positions.insert(positions.end(), pos.begin(), pos.end());
        }
    return positions;

}


/**
 * получить список положений и ориентаций рабочего инструмента (заданы параметрами Родриго-Гамильтона)
 * @param state  состояние сцены
 * @return список положений и ориентаций
 */
std::vector<double> Scene::getEndEffectorRGVectors(const std::vector<double> &state) {
    std::vector<double> rgVectors;
    for (unsigned long i = 0; i < _objects.size(); i++)
        if (_objects.at(i)->getJointCnt() > 0) {
            std::vector<double> localState = getSingleObjectState(state, i);
            std::vector<double> pos = _objects.at(i)->getEndEffectorRGVector(localState);
            rgVectors.insert(rgVectors.end(), pos.begin(), pos.end());
        }
    return rgVectors;
}

/**
 * получить частную производную положения и ориентации рабочего инструмента
 * (заданы параметрами Родриго-Гамильтона) по i-ой координате
 * @param state  состояние сцены
 * @param iVal индекс координаты, по которой берётся производная
 * @return частная производная
 */
std::vector<double> Scene::getEndEffectorDiffRGVector(const std::vector<double> &state, int iVal) {
    std::vector<double> rgVectors;
    for (unsigned long i = 0; i < _objects.size(); i++)
        if (_objects.at(i)->getJointCnt() > 0) {
            std::vector<double> localState = getSingleObjectState(state, i);
            std::vector<double> pos = _objects.at(i)->getEndEffectorDiffRGVector(localState, iVal);
            rgVectors.insert(rgVectors.end(), pos.begin(), pos.end());
        }
    return rgVectors;
}

/**
 *  получить частную производную положения и ориентации рабочего инструмента
 * (заданы параметрами Родриго-Гамильтона) по i-ой и j-ой координатам
 * @param state  состояние сцены
 * @param iVal индекс координаты, по которой первый раз берётся производная
 * @param jVal индекс координаты, по которой второй раз берётся производная
 * @return частная производная
 */
std::vector<double> Scene::getEndEffectorDiff2RGVector(const std::vector<double> &state, int iVal, int jVal) {
    std::vector<double> rgVectors;
    for (unsigned long i = 0; i < _objects.size(); i++)
        if (_objects.at(i)->getJointCnt() > 0) {
            std::vector<double> localState = getSingleObjectState(state, i);
            std::vector<double> pos = _objects.at(i)->getEndEffectorDiff2RGVector(localState, iVal, jVal);
            rgVectors.insert(rgVectors.end(), pos.begin(), pos.end());
        }
    return rgVectors;
}

/**
 * получить список положений звеньев всех роботов по состоянию сцены (x,y,z)
 * @param state  состояние сцены
 * @return список положений звеньев
 */
std::vector<double> Scene::getAllLinkPositions(const std::vector<double> &state) {
    std::vector<double> poses;
    for (unsigned long i = 0; i < _objects.size(); i++)
        if (_objects.at(i)->getJointCnt() > 0) {
            std::vector<double> localState = getSingleObjectState(state, i);
            std::vector<double> pos = _objects.at(i)->getAllLinkPositions(localState);
            poses.insert(poses.end(), pos.begin(), pos.end());
        }
    return poses;
}

/**
 * получить список положений (x,y,z) и ориентаций (параметры Родриго-Гамильтона)
 * звеньев всех роботов по состоянию сцены
 * @param state состояние сцены
 * @return список положений и ориентаций
 */
std::vector<double> Scene::getAllLinkRGVectors(const std::vector<double> &state) {
    std::vector<double> poses;
    for (unsigned long i = 0; i < _objects.size(); i++)
        if (_objects.at(i)->getJointCnt() > 0) {
            std::vector<double> localState = getSingleObjectState(state, i);
            std::vector<double> pos = _objects.at(i)->getAllLinkRGVectors(localState);
            poses.insert(poses.end(), pos.begin(), pos.end());
        }
    return poses;
}

/**
 * проверка, допустимо ли состояние сцены для конкретного робота
 * (по допустимым диапазонам звеньев)
 * @param state состояние сцены
 * @param robotNum номер робота в списке роботов
 * @return  флаг, допустимо ли состояние сцены
 */
bool Scene::isStateEnabled(std::vector<double> state, unsigned long robotNum) {
    return _objects.at(robotNum)->isStateEnabled(std::move(state));
}

/**
 * проверка, допустимо ли состояние сцены (по допустимым диапазонам звеньев)
 * @param  state состояние сцены
 * @return флаг, допустимо ли состояние сцены
 */
bool Scene::isStateEnabled(const std::vector<double> &state) {
    if (state.size() != _jointParams.size()) {
        char buf[1024];
        sprintf(buf,
                "Scene::isStateEnabled() ERROR: \n state size is %zu, but _jointParams size is %zu",
                state.size(), _jointParams.size()
        );
        throw std::invalid_argument(buf);
    }

    for (long i = 0; i < _jointIndexRanges.size(); i++)
        if (!isStateEnabled(getSingleObjectState(state, i), i))
            return false;
    return true;
}

/**
 * получить случайное состояние сцены
 * @return случайное состояние сцены
 */
std::vector<double> Scene::getRandomState() {
    std::vector<double> state;
    do {
        std::vector<double> joints;
        state.clear();
        for (const auto &jointParams: getJointParamsList())
            state.push_back(jointParams->getRandomAngle());
    } while (!isStateEnabled(state));
    return state;
}

/**
 * получить смещения для каждого из роботов
 * @return смещения для каждого из роботов
 */
std::vector<std::vector<double>> Scene::getGroupedTranslation() const {
    std::vector<std::vector<double>> result;
    for (const auto &sceneDescription: _objects)
        result.emplace_back(sceneDescription->getWorldTranslation());
    return result;
}

/**
 * получить повороты для каждого из роботов
 * @return повороты для каждого из роботов
 */
std::vector<std::vector<double>> Scene::getGroupedRotation() const {
    std::vector<std::vector<double>> result;
    for (const auto &robot: _objects)
        result.emplace_back(robot->getWorldRotation());
    return result;
}

/**
 * получить масштабы для каждого из роботов
 * @return масштабы для каждого из роботов
 */
std::vector<std::vector<double>> Scene::getGroupedScale() const {
    std::vector<std::vector<double>> result;
    for (const auto &robot: _objects)
        result.emplace_back(robot->getWorldScale());
    return result;
}

/**
 * получить векторы преобразования для каждого из роботов
 * @return векторы преобразования для каждого из роботов
 */
std::vector<std::vector<double>> Scene::getGroupedTransformVector() const {
    std::vector<std::vector<double>> result;
    for (const auto &robot: _objects)
        result.emplace_back(robot->getWorldTransformVector());
    return result;
}

/**
 * получить пути к моделям для каждого из роботов
 * (группируются по роботам)
 * @return пути к моделям для каждого из роботов
 */
std::vector<std::vector<std::string>> Scene::getGroupedModelPaths() const {
    std::vector<std::vector<std::string>> result;
    for (const auto &robot: _objects)
        result.emplace_back(robot->getModelPaths());
    return result;
}

/**
 * задать смещения для каждого из роботов
 * @param groupedTranslation смещения для каждого из роботов
 */
void Scene::setGroupedTranslation(std::vector<std::vector<double>> groupedTranslation) {
    if (groupedTranslation.size() != _objects.size()) {
        char buf[1024];
        sprintf(buf,
                "Scene::setGroupedTranslation() ERROR: \n groupedTranslation size is %zu, but _objects size is %zu",
                groupedTranslation.size(), _objects.size()
        );
        throw std::invalid_argument(buf);
    }

    for (unsigned int i = 0; i < groupedTranslation.size(); i++)
        _objects.at(i)->setWorldTranslation(groupedTranslation.at(i));
}

/**
 * задать повороты для каждого из роботов
 * @param groupedRotation повороты для каждого из роботов
 */
void Scene::setGroupedRotation(std::vector<std::vector<double>> groupedRotation) {
    if (groupedRotation.size() != _objects.size()) {
        char buf[1024];
        sprintf(buf,
                "Scene::setGroupedRotation() ERROR: \n groupedRotation size is %zu, but _objects size is %zu",
                groupedRotation.size(), _objects.size()
        );
        throw std::invalid_argument(buf);
    }

    for (unsigned int i = 0; i < groupedRotation.size(); i++)
        _objects.at(i)->setWorldRotation(groupedRotation.at(i));
}

/**
 * задать масштабы для каждого из роботов
 * @param groupedScale масштабы для каждого из роботов
 */
void Scene::setGroupedScale(std::vector<std::vector<double>> groupedScale) {
    if (groupedScale.size() != _objects.size()) {
        char buf[1024];
        sprintf(buf,
                "Scene::setGroupedScale() ERROR: \n groupedScale size is %zu, but _objects size is %zu",
                groupedScale.size(), _objects.size()
        );
        throw std::invalid_argument(buf);
    }
    for (unsigned int i = 0; i < groupedScale.size(); i++)
        _objects.at(i)->setWorldScale(groupedScale.at(i));
}

/**
 * задать векторы преобразования для каждого из роботов
 * @param transformVectors векторы преобразования для каждого из роботов
 */
void Scene::setGroupedTransformVector(std::vector<std::vector<double>> transformVectors) {
    if (transformVectors.size() != _objects.size()) {
        char buf[1024];
        sprintf(buf,
                "Scene::setGroupedTransformVector() ERROR: \n transformVectors size is %zu, but _objects size is %zu",
                transformVectors.size(), _objects.size()
        );
        throw std::invalid_argument(buf);
    }
    for (unsigned int i = 0; i < transformVectors.size(); i++)
        _objects.at(i)->setWorldTransformVector(transformVectors.at(i));
}

