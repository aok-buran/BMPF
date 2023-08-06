#include <log.h>
#include <fstream>
#include <set>

#include "urdf_robot.h"
#include "matrix_math.h"



using namespace bmpf;

/**
 * @brief Загрузить параметры робота из файла
 *
 * Загрузить параметры робота из файла, нужно заполнить следующие поля:
 * _jointParams, _links, _nonHierarchicalLinks, path
 * @param path путь к файлу описания робота
 */
void URDFRobot::loadFromFile(std::string path) {
    _jointParams.clear();
    _links.clear();
    _joints.clear();
    _nonHierarchicalLinks.clear();
    _path = path;
    urdf::Model model;
    // если не получилось загрузить модель
    if (!model.initFile(path))
        throw std::invalid_argument("error reading urdf file " + path);

    //  model.joints_ - словарь, ключу соответствет название сочленения,
    // значению - умный указатель на само звено, заполняем по нему множество
    // указателей на сочленения, после обработки в нём
    // должно остаться последнее сочленение, которое не является родителем (энд-эффектор)
    std::set<std::shared_ptr<urdf::Joint>> jointSet;
    for (const auto &joint: model.joints_)
        jointSet.insert(joint.second);

    // создаём и заполняемсловарь, где каждая пара ключ-значение
    // соответствует указателю на родительское сочленение и на дочернее
    std::map<std::shared_ptr<urdf::Joint>, std::shared_ptr<urdf::Joint>> nextJoints;
    for (const auto &joint1: model.joints_)
        for (const auto &joint2: model.joints_) {
            auto p1 = joint1.second;
            auto p2 = joint2.second;
            if (p1 && p2 && p1 != p2 && p1->child_link_name == p2->parent_link_name) {
                // удаляем из множества сочленение-родитель
                jointSet.erase(p2);
                nextJoints.insert(std::pair<std::shared_ptr<urdf::Joint>, std::shared_ptr<urdf::Joint>>(p1, p2));
            }
        }

    // проверяем, что осталось всего одно сочленение (предполагается, что робот либо не имеет
    // ветвелений в иерархии, либо вообще не иерархичен)
    // должно остаться последнее сочленение, которое не является родителем
    if(jointSet.size() != 1 ) {
        char buf[1024];
        sprintf(buf,
                "URDFRobot::loadFromFile() ERROR: \n vector size is %zu, but needs 1\n It seems, that"
                "robot is not hierarchical",
                jointSet.size()
        );
        throw std::runtime_error(buf);
    }



    // получаем оставшееся значение
    auto currentJoint = *jointSet.begin();

    // и формируем список, в котором звенья идут от энд-эффектора к базе
    std::vector<std::shared_ptr<urdf::Joint>> orderedJointList;
    while (nextJoints.find(currentJoint) != nextJoints.end()) {
        orderedJointList.push_back(currentJoint);
        currentJoint = nextJoints.at(currentJoint);
    }
    // применяем полученные параметры к роботу
    _applyURDF(orderedJointList, model.links_);
}

/**
 * Заполнить по urdf параметры робота, jointList - список сочленений,
 * linkMap - словарь, ключу соответствет название звена,
 * значению - умный указатель на само звено
 * @param jointList
 * @param linkMap
 */
void URDFRobot::_applyURDF(
        std::vector<std::shared_ptr<urdf::Joint>> jointList, std::map<std::string, urdf::LinkSharedPtr> linkMap
) {
    // списое названий иерархичных звеньев
    std::set<std::string> hierarchicalLinkNames;

    for (unsigned int i = 0; i < jointList.size(); i++) {
        auto j = jointList.at(i);
        Eigen::Matrix4d parentTransform = _getMatrixFromPose(j->parent_to_joint_origin_transform);
        urdf::Vector3 axis = j->axis;

        bool isFixed = j->type == urdf::Joint::FIXED;
        // если сочленение не фиксированное
        if (!isFixed){
            _jointParams.emplace_back(std::make_shared<JointParams>(
                    j->limits->effort, j->limits->velocity,
                    j->limits->upper, j->limits->lower,
                    i
            ));
            _maxSpeedModules.emplace_back(j->limits->velocity);
            _maxAccelerationModules.emplace_back(j->limits->effort*EFFORT_TO_ACCELERATION);
        }



        hierarchicalLinkNames.insert(j->child_link_name);
        // получаем urdf звено
        auto urdfLink = linkMap.at(j->child_link_name);

        Eigen::Matrix4d linkTransform = _createLinkTransform(urdfLink);

        _links.emplace_back(_createLink(urdfLink, linkTransform));

        _joints.emplace_back(std::make_shared<Joint>(
                parentTransform, linkTransform, isFixed, 0.0,
                Eigen::Vector3d(axis.x, axis.y, axis.z), false
        ));
    }

    // перебираем звенья
    for (auto &entry: linkMap)
        // если звено ещё не найдено
        if (hierarchicalLinkNames.find(entry.first) == hierarchicalLinkNames.end())
            // если у звена определена геометрия
            if (entry.second->collision && entry.second->collision->geometry) {
                Eigen::Matrix4d linkTransform = _createLinkTransform(entry.second);
                auto link = _createLink(entry.second, linkTransform);
                _links.emplace_back(link);
                _nonHierarchicalLinks.emplace_back(link);
            }
}

/**
 * получить матрицу преобразования по urdf звену
 * @param urdfLink urdf звено
 * @return матрица преобразования
 */
Eigen::Matrix4d URDFRobot::_createLinkTransform(const std::shared_ptr<urdf::Link> &urdfLink) {
    Eigen::Matrix4d linkTransform = _getMatrixFromPose(urdfLink->collision->origin);
    auto scale = std::dynamic_pointer_cast<const urdf::Mesh>(urdfLink->collision->geometry)->scale;
    linkTransform = linkTransform * getScaleMatrix(scale.x, scale.y, scale.z);
    return linkTransform;
}

/**
 * получить звено по urdf звену
 * @param urdfLink urdf звено
 * @param linkTransform матрица преобразования звена
 * @return звено
 */
std::shared_ptr<Link>
URDFRobot::_createLink(const std::shared_ptr<urdf::Link> &urdfLink, const Eigen::Matrix4d &linkTransform) {
    auto mc = std::vector<double>{
            urdfLink->inertial->mass, urdfLink->inertial->origin.position.x,
            urdfLink->inertial->origin.position.y, urdfLink->inertial->origin.position.z,
    };

    double inertiaData[9] = {
            urdfLink->inertial->ixx, urdfLink->inertial->ixy, urdfLink->inertial->ixz,
            urdfLink->inertial->ixy, urdfLink->inertial->iyy, urdfLink->inertial->iyz,
            urdfLink->inertial->ixz, urdfLink->inertial->iyz, urdfLink->inertial->izz
    };

    auto inertia = Eigen::Matrix3d(inertiaData);

    return std::make_shared<Link>(
            std::dynamic_pointer_cast<const urdf::Mesh>(urdfLink->collision->geometry)->filename,
            std::make_shared<Eigen::Matrix4d>(linkTransform),
            urdfLink->name,
            inertia,
            mc
    );
}

/**
 * Получить матрицу преобразования из положения urdf
 * @param pose положение
 * @return матрица преобразования
 */
Eigen::Matrix4d URDFRobot::_getMatrixFromPose(urdf::Pose pose) {
    Eigen::Matrix4d m;
    urdf::Vector3 p = pose.position;
    urdf::Rotation r = pose.rotation;
    double a = r.w;
    double b = r.x;
    double c = r.y;
    double d = r.z;
    m << a * a + b * b - c * c - d * c, 2 * b * c - 2 * a * d, 2 * b * d + 2 * a * c, p.x,
            2 * b * c + 2 * a * d, a * a - b * b + c * c - d * d, 2 * c * d - 2 * a * b, p.y,
            2 * b * d - 2 * a * c, 2 * c * d + 2 * a * b, a * a - b * b - c * c + d * d, p.z,
            0, 0, 0, 1;
    return m;
}


