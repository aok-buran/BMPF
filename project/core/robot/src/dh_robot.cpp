#include "dh_robot.h"

#include <fstream>
#include <log.h>

using namespace bmpf;

/**
 * Получить матрицу преобразования по параметрам ДХ
 * @param theta параметр theta
 * @param d параметр d
 * @param a параметр a
 * @param alpha параметр alpha
 * @return
 */
Eigen::Matrix4d DHRobot::getDHMatrix(double theta, double d, double a, double alpha) {
    double thetaF = theta;
    double data[16] = {
            cos(thetaF), -sin(thetaF) * cos(alpha), sin(thetaF) * sin(alpha), a * cos(thetaF),
            sin(thetaF), cos(thetaF) * cos(alpha), -cos(thetaF) * sin(alpha), a * sin(thetaF),
            0, sin(alpha), cos(alpha), d,
            0, 0, 0, 1
    };
    return Eigen::Matrix4d(data).transpose();
}

/**
 * @brief загрузить параметры робота из файла
 *
 * Загрузить параметры робота из файла, нужно заполнить следующие поля:
 * _jointParams, _links, _nonHierarchicalLinks, path
 * @param path путь к файлу описания робота
 */
void DHRobot::loadFromFile(std::string path) {
    _jointParams.clear();
    _joints.clear();
    _links.clear();
    _nonHierarchicalLinks.clear();
    _path = path;

    Json::Reader reader;
    Json::Value obj;

    std::ifstream ifs(path.c_str(), std::ios_base::binary);
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    if(content.empty()) {
        char buf[1024];
        sprintf(buf,
                "DHRobot::loadFromFile() ERROR: \n file content is empty"
        );
        throw std::runtime_error(buf);
    }


    reader.parse(content, obj); // reader can also read strings
    infoMsg("Robot name: ", obj["name"].asString());

    int i = 0;

    Eigen::Matrix4d parentTransform = Eigen::Matrix4d::Identity();

    for (auto joint: obj["joints"]) {
        double theta = joint["theta"].asDouble() / 180 * 3.14;
        double d = joint["d"].asDouble();
        double alpha = joint["alpha"].asDouble() / 180 * 3.14;
        double a = joint["a"].asDouble();
        bool isFixed = joint["fixed"].asBool();
        std::string name = joint["name"].asString();
        std::string linkName = joint["link"].asString();



        _joints.emplace_back(std::make_shared<Joint>(
                parentTransform, Eigen::Matrix4d::Identity(), isFixed, 0.0,
                Eigen::Vector3d(0, 0, -1), linkName.empty()
        ));

        parentTransform = getDHMatrix(theta, d, a, alpha);

        // если сочленение не фиксированное
        if (!isFixed)
            _jointParams.emplace_back(std::make_shared<JointParams>(
                    100, 100,
                    -100, -100, i++
            ));
    }

    for (auto dh: obj["links"]) {
        Link l(dh["model"].asString(),
               std::make_shared<Eigen::Matrix4d>(Eigen::Matrix4d::Identity()),
               dh["name"].asString(), Eigen::Matrix3d::Identity(),
               {0.0, 0.0, 0.0, 0.0}
        );
        _links.emplace_back(std::make_shared<Link>(l));
    }

}


