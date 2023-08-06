#include <base/robot.h>
#include <urdf_robot.h>

int main() {
    std::shared_ptr<bmpf::BaseRobot> sc = std::make_shared<bmpf::URDFRobot>();
    sc->loadFromFile("../../../../config/urdf/kuka_six.urdf");

    bmpf::infoMsg(sc->getJointCnt());
    bmpf::infoMsg(sc->getLinkCnt());

    for (const auto &path: sc->getModelPaths()) {
        bmpf::infoMsg(path);
    }
    return 0;
}