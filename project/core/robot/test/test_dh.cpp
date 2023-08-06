#include <base/robot.h>
#include <urdf_robot.h>
#include <dh_robot.h>
#include "state.h"


const int TEST_CNT = 50;

int main() {

    srand(time(nullptr));

    std::shared_ptr<bmpf::BaseRobot> urdf = std::make_shared<bmpf::URDFRobot>();
    urdf->loadFromFile("../../../../config/urdf/kuka_six.urdf");

    std::shared_ptr<bmpf::BaseRobot> dh = std::make_shared<bmpf::DHRobot>();
    dh->loadFromFile("../../../../config/dh/kuka_six.json");

    for (int i = 0; i < TEST_CNT; i++) {
        auto state = urdf->getRandomState();
        bmpf::infoState("random state:", state);
        auto urdfPoses = urdf->getAllLinkPositions(state);
        auto dhPoses = dh->getAllLinkPositions(state);
        for (int j = 0; j < 21; j++)
            assert(std::abs(dhPoses[j] - urdfPoses[j]) < 0.01);
    }
    return 0;
}