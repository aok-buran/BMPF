#include "base/robot.h"

using namespace bmpf;

/**
 * Конструктор
 */
BaseRobot::BaseRobot() {
    // заполняем трансформациию из СК мира в СК робота
    _worldTransformVector = std::vector<double>{0, 0, 0, 0, 0, 0, 1, 1, 1};
    // рассчитываем матрицу трансформации из Ск мира в СК робота
    _fillWorldTransformMatrix();
}

/**
 * проверка допустимости углов поворота сочленения (рассчитывается по ограничениям сочленений)
 * @param state состояние
 * @return флаг, допустим ли соответствующий набор углов поворота (конфигурация)
 */
bool BaseRobot::isStateEnabled(std::vector<double> state) {
    if (state.size() != _jointParams.size()) {
        char buf[1024];
        sprintf(buf,
                "BaseRobot::isStateEnabled() ERROR: \n state size is %zu, but _jointParams size is %zu",
                state.size(), _jointParams.size()
        );
        throw std::invalid_argument(buf);
    }

    for (unsigned int i = 0; i < state.size(); i++) {
        double v = state.at(i);
        double max = _jointParams.at(i)->maxAngle;
        double min = _jointParams.at(i)->minAngle;
        if (v - min < -0.00001 || v - max > 0.00001)
            return false;
    }
    return true;
}

/**
 * получить список моделей, используемых роботом
 * @return список моделей, используемых роботом
 */
std::vector<std::string> BaseRobot::getModelPaths() const {
    // получаем путь к папке с параметрами робота
    std::string subPath;
    int p = (int) _path.find_last_of('/');
    if (p != std::string::npos)
        subPath = _path.substr(0, p + 1);
    else
        subPath = "";

    std::vector<std::string> modelPaths;
    for (const auto &link: _links)
        modelPaths.emplace_back(subPath + link->modelPath);

    return modelPaths;
}

/**
 * перебор матриц преобразования из СК мира в СК соответствующего звена,
 * при состоянии робота равном state
 * @param state состояние
 * @param consumer обработчик для каждой матрицы
 */
template<typename F>
void BaseRobot::_forEachJoint(std::vector<double> state, const F &consumer) {
    Eigen::Matrix4d transformMatrix = *getWorldTransformMatrix();
    std::vector<Eigen::Matrix4d> matrices;
    unsigned int jointPos = 0;

    for (unsigned long i = 0; i < _joints.size(); i++) {
        auto jd = _joints.at(i);
        if (!jd->isFixed) {
            jd->jointAngle = state.at(jointPos);
            jointPos++;
        }
        transformMatrix = transformMatrix * jd->getTransformMatrix();
        if (!_joints.at(i)->isVirtual)
            consumer(i, _joints.at(i), transformMatrix * jd->linkTransform);
    }
}

/**
 * перебор матриц преобразования из СК мира в СК соответствующего звена,
 * при состоянии робота равном state (вместо i-ой матрицы преобразования берётся
 * её частная производная по i-ой координате)
 * @param state состояние
 * @param iVal индекс координаты, по которой берётся производная
 * @param consumer обработчик для каждой матрицы
 */
template<typename F>
void BaseRobot::_forEachDiffJoint(std::vector<double> state, int iVal, const F &consumer) {
    Eigen::Matrix4d transformMatrix = *getWorldTransformMatrix();
    std::vector<Eigen::Matrix4d> matrices;
    unsigned int jointPos = 0;

    for (unsigned long i = 0; i < _joints.size(); i++) {
        auto jd = _joints.at(i);
        if (!jd->isFixed) {
            jd->jointAngle = state.at(jointPos);
            jointPos++;
        }
        if (jointPos - 1 == iVal)
            transformMatrix = transformMatrix * jd->getDiffTransformMatrix();
        else
            transformMatrix = transformMatrix * jd->getTransformMatrix();
        if (!_joints.at(i)->isVirtual)
            consumer(i, _joints.at(i), transformMatrix * jd->linkTransform);
    }
}

/**
 * перебор матриц преобразования из СК мира в СК соответствующего звена,
 * при состоянии робота равном state (вместо i-ой и j-ой матриц преобразования берутся
 * её частные производные по i-ой и j-ой координатам соответственно и вторая производная
 * по i-ой координате, если i==j)
 * @param state состояние
 * @param iVal индекс координаты, по которой первый раз берётся производная
 * @param jVal индекс координаты, по которой второй раз берётся производная
 * @param consumer обработчик для каждой матрицы
 */
template<typename F>
void BaseRobot::_forEachDiff2Joint(std::vector<double> state, int iVal, int jVal, const F &consumer) {
    Eigen::Matrix4d transformMatrix = *getWorldTransformMatrix();
    std::vector<Eigen::Matrix4d> matrices;
    unsigned int jointPos = 0;

    for (unsigned long i = 0; i < _joints.size(); i++) {
        auto jd = _joints.at(i);
        if (!jd->isFixed) {
            jd->jointAngle = state.at(jointPos);
            jointPos++;
        }
        if (jVal != iVal && (jointPos - 1 == iVal || jointPos - 1 == jVal))
            transformMatrix = transformMatrix * jd->getDiffTransformMatrix();
        else if (jVal == iVal && jointPos - 1 == iVal)
            transformMatrix = transformMatrix * jd->getDiff2TransformMatrix();
        else
            transformMatrix = transformMatrix * jd->getTransformMatrix();

        if (!_joints.at(i)->isVirtual)
            consumer(i, _joints.at(i), transformMatrix * jd->linkTransform);
    }
}

/**
 * получить список матриц преобразований всех звеньев по состоянию
 * @param state состояние
 * @return список матриц преобразований
 */
std::vector<Eigen::Matrix4d> BaseRobot::getTransformMatrices(std::vector<double> state) {
    std::vector<Eigen::Matrix4d> matrices;

    _forEachJoint(std::move(state),
                  [&matrices](int jointNum, const std::shared_ptr<Joint> &joint, const Eigen::Matrix4d &tf) {
                      matrices.emplace_back(tf);
                  });

    for (const auto &link: _nonHierarchicalLinks)
        matrices.emplace_back((*getWorldTransformMatrix()) * (*link->linkTransformMatrix));

    return matrices;
}

/**
 * получить список осей вращения сочленений робота в СК мира по состоянию
 * @param state состояние
 * @return список осей вращения сочленений робота
 */
std::vector<Eigen::Vector3d> BaseRobot::getJointAxes(std::vector<double> state) {
    std::vector<Eigen::Vector3d> axes;

    _forEachJoint(std::move(state),
                  [&axes](int jointNum, const std::shared_ptr<Joint> &joint, const Eigen::Matrix4d &tf) {
                      Eigen::Vector3d jointAxis = joint->getAxis();
                      Eigen::Vector4d axis = tf * Eigen::Vector4d({jointAxis(0), jointAxis(1), jointAxis(2), 0});
                      axes.emplace_back(Eigen::Vector3d({axis(0), axis(1), axis(2)}));
                  });

    return axes;
}

/**
 * получить матрицу преобразования из СК базы робота в СК рабочего инструмента по состоянию
 * @param state  состояние
 * @return матрица преобразования
 */
Eigen::Matrix4d BaseRobot::getEndEffectorTransformMatrix(std::vector<double> state) {
    if (_jointParams.empty())
        return Eigen::Matrix4d::Identity();

    Eigen::Matrix4d transformMatrix = *getWorldTransformMatrix();

    _forEachJoint(std::move(state),
                  [&transformMatrix](int jointNum, const std::shared_ptr<Joint> &joint, const Eigen::Matrix4d &tf) {
                      transformMatrix = tf;
                  });
    return transformMatrix;
}


/**
 * Получить частную производную по i-ой координате матрицы преобразования
 * из СК базы робота в СК рабочего инструмента
 *
 * @param state состояние
 * @param iVal индекс координаты, по которой берётся производная
 * @return частная производная по i-ой координате
 */
Eigen::Matrix4d BaseRobot::getEndEffectorDiffTransformMatrix(std::vector<double> state, int iVal) {
    if (_jointParams.empty())
        return Eigen::Matrix4d::Identity();

    Eigen::Matrix4d transformMatrix = *getWorldTransformMatrix();

    _forEachDiffJoint(std::move(state), iVal,
                      [&transformMatrix](int jointNum, const std::shared_ptr<Joint> &joint, const Eigen::Matrix4d &tf) {
                          transformMatrix = tf;
                      });
    return transformMatrix;
}

/**
 * Получить частную производную по i-ой и j-ой координатам матрицы преобразования
 * из СК базы робота в СК рабочего инструмента
 *
 * @param state состояние
 * @param iVal индекс координаты, по которой первый раз берётся производная
 * @param jVal индекс координаты, по которой второй раз берётся производная
 * @return частная производная по i-ой и j-ой координатам
 */
Eigen::Matrix4d BaseRobot::getEndEffectorDiff2TransformMatrix(std::vector<double> state, int iVal, int jVal) {
    if (_jointParams.empty())
        return Eigen::Matrix4d::Identity();

    Eigen::Matrix4d transformMatrix = *getWorldTransformMatrix();

    _forEachDiff2Joint(std::move(state), iVal, jVal,
                       [&transformMatrix](int jointNum, const std::shared_ptr<Joint> &joint,
                                          const Eigen::Matrix4d &tf) {
                           transformMatrix = tf;
                       });
    return transformMatrix;
}


/**
 * задать смещение из СК мира в СК робота
 * @param translation смещение
 */
void BaseRobot::setWorldTranslation(const std::vector<double> &translation) {
    if(translation.size() != 3 ) {
        char buf[1024];
        sprintf(buf,
                "BaseRobot::setWorldTranslation() ERROR: \n translation size is %zu, but needs 3",
                translation.size()
        );
        throw std::invalid_argument(buf);
    }
    for (unsigned int i = 0; i < 3; i++)
        _worldTransformVector.at(i) = translation.at(i);
    // рассчитываем матрицу трансформации из СК мира в СК робота
    _fillWorldTransformMatrix();
}

/**
 * задать поворот из СК мира в СК робота
 * @param rotation поворот
 */
void BaseRobot::setWorldRotation(const std::vector<double> &rotation) {
    if(rotation.size() != 3 ) {
        char buf[1024];
        sprintf(buf,
                "BaseRobot::setWorldRotation() ERROR: \n rotation size is %zu, but needs 3",
                rotation.size()
        );
        throw std::invalid_argument(buf);
    }
    for (unsigned int i = 0; i < 3; i++)
        _worldTransformVector.at(i + 3) = rotation.at(i);
    // рассчитываем матрицу трансформации из СК мира в СК робота
    _fillWorldTransformMatrix();
}

/**
 * задать масштабирование из СК мира в СК робота
 * @param scale масштабирование
 */
void BaseRobot::setWorldScale(const std::vector<double> &scale) {
    if(scale.size() != 3 ) {
        char buf[1024];
        sprintf(buf,
                "BaseRobot::setWorldScale() ERROR: \n scale size is %zu, but needs 3",
                scale.size()
        );
        throw std::invalid_argument(buf);
    }
    for (unsigned int i = 0; i < 3; i++)
        _worldTransformVector.at(i + 6) = scale.at(i);
    // рассчитываем матрицу трансформации из СК мира в СК робота
    _fillWorldTransformMatrix();
}

/**
 * задать вектор перехода из СК мира в СК робота: x,y,z; rotation: r,p,y; scale: x,y,z
 * @param vec  вектор перехода
 */
void BaseRobot::setWorldTransformVector(const std::vector<double> &vec) {
    if(vec.size() != 9 ) {
        char buf[1024];
        sprintf(buf,
                "BaseRobot::setWorldTransformVector() ERROR: \n vector size is %zu, but needs 9",
                vec.size()
        );
        throw std::invalid_argument(buf);
    }

    _worldTransformVector = vec;
    // рассчитываем матрицу трансформации из СК мира в СК робота
    _fillWorldTransformMatrix();
}

/**
 * Заполнить матрицу перехода из СК мира в СК робота
 */
void BaseRobot::_fillWorldTransformMatrix() {
    _worldTransformMatrix = std::make_shared<Eigen::Matrix4d>(
            getRotationMatrix(getWorldRotation()) * getTranslationMatrix(getWorldTranslation()) *
            getScaleMatrix(getWorldScale())
    );
}

/**
 * получить положение рабочего инструмента робота
 * @param state состояние
 * @return положение рабочего инструмента
 */
std::vector<double> BaseRobot::getEndEffectorPos(std::vector<double> state) {
    auto tf = getEndEffectorTransformMatrix(std::move(state));
    return getPosition(tf);
}

/**
 * получить частную производную положения энд-эффектора робота по i-ой координате
 * @param state состояние
 * @param iVal индекс координаты, по которой берётся производная
 * @return
 */
std::vector<double> BaseRobot::getEndEffectorDiffPos(std::vector<double> state, int iVal) {
    auto tf = getEndEffectorDiffTransformMatrix(std::move(state), iVal);
    return getPosition(tf);
}

/**
 * получить частную производную положения энд-эффектора робота по i-ой и j-ой координатам
 * @param state состояние
 * @param iVal индекс координаты, по которой первый раз берётся производная
 * @param jVal индекс координаты, по которой второй раз берётся производная
 * @return
 */
std::vector<double> BaseRobot::getEndEffectorDiff2Pos(std::vector<double> state, int iVal, int jVal) {
    auto tf = getEndEffectorDiff2TransformMatrix(std::move(state), iVal, jVal);
    return getPosition(tf);
}

/**
 * получить положение и ориентацию(задана параметрами Родриго-Гамильтона)
 * @param state состояние
 * @return положение и ориентацию(задана параметрами Родриго-Гамильтона)
 */
std::vector<double> BaseRobot::getEndEffectorRGVector(std::vector<double> state) {
    auto tf = getEndEffectorTransformMatrix(std::move(state));
    return getRGVector(tf);
}

/**
 *  получить частную производную положения и ориентации(задана параметрами
 *  Родриго-Гамильтона) по i-ой координате
 * @param state состояние
 * @param iVal индекс координаты, по которой берётся производная
 * @return
 */
std::vector<double> BaseRobot::getEndEffectorDiffRGVector(std::vector<double> state, int iVal) {
    auto tf = getEndEffectorDiffTransformMatrix(std::move(state), iVal);
    return getRGVector(tf);
}

/**
 * получить частную производную положения и ориентации(задана параметрами
 * Родриго-Гамильтона) по i-ой и j-ой координатам
 * @param state состояние
 * @param iVal индекс координаты, по которой первый раз берётся производная
 * @param jVal индекс координаты, по которой второй раз берётся производная
 * @return
 */
std::vector<double> BaseRobot::getEndEffectorDiff2RGVector(std::vector<double> state, int iVal, int jVal) {
    auto tf = getEndEffectorDiff2TransformMatrix(std::move(state), iVal, jVal);
    return getRGVector(tf);
}

/**
 * получить положения всех звеньев
 * @param state  состояние
 * @return положения всех звеньев
 */
std::vector<double> BaseRobot::getAllLinkPositions(std::vector<double> state) {
    auto tfs = getTransformMatrices(std::move(state));
    std::vector<double> allPoses;
    for (auto &tf: tfs) {
        auto pos = getPosition(tf);
        allPoses.insert(allPoses.end(), pos.begin(), pos.end());
    }
    return allPoses;
}

/**
 * получить список положений(x,y,z) и ориентаций (параметры Родриго-Гамильтона) всех звеньев
 * @param state состояние
 * @return  положения всех звеньев
 */
std::vector<double> BaseRobot::getAllLinkRGVectors(std::vector<double> state) {
    auto tfs = getTransformMatrices(std::move(state));
    std::vector<double> allPoses;
    for (auto &tf: tfs) {
        auto pos = getRGVector(tf);
        allPoses.insert(allPoses.end(), pos.begin(), pos.end());
    }
    return allPoses;
}

/**
* получить список координат центров масс в СК соответствующих
* звеньев робота(x1, y1, z1, x2, y2, z2, ...)
* @return список координат центров масс
*/
std::vector<double> BaseRobot::getMassCenterLocalPositions() {
    std::vector<double> result;
    for (auto &link: _links) {
        result.insert(result.end(), link->massCenter.begin(), link->massCenter.end());
    }
    return result;
}

/**
 * получить список координат центров масс в СК базы
 * робота(x1, y1, z1, x2, y2, z2, ...)
 * @param state  состояние
 * @return  список координат центров масс
 */
std::vector<double> BaseRobot::getMassCenterAbsolutePositions(std::vector<double> state) {
    std::vector<double> cmposes;
    // получаем матрицы преобразования звеньев
    auto tfs = getTransformMatrices(std::move(state));
    // получаем координаты центров масс в СК соответствующего звена
    auto mcs = getMassCenterLocalPositions();
    // перебираем звенья и переводим координаты центров масс
    // из СК соответствующих звеньев в СК базы робота
    for (int i = 0; i < _links.size(); i++) {
        auto tf = tfs.at(i);
        auto cm = _links.at(i)->massCenter;
        double mass = cm.at(0);
        double data[4]{cm.at(1), cm.at(2), cm.at(3), 1};
        Eigen::Vector4d pos(data);
        pos = tf * pos;
        cmposes.emplace_back(mass);
        cmposes.emplace_back(pos(0));
        cmposes.emplace_back(pos(1));
        cmposes.emplace_back(pos(2));
    }
    return cmposes;
}

/**
 * получить список тензоров инерции
 * @return список тензоров инерции
 */
std::vector<Eigen::Matrix3d> BaseRobot::getInertias() {
    std::vector<Eigen::Matrix3d> inertias;
    for (auto &_link: _links)
        inertias.emplace_back(_link->inertia);
    return inertias;
}

/**
 * получить случайное состояние
 * @return случайное состояние
 */
std::vector<double> BaseRobot::getRandomState() {
    std::vector<double> state;
    for (auto &actuator: _jointParams)
        state.emplace_back(actuator->getRandomAngle());
    return state;
}