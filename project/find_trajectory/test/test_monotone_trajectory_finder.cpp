#include "monotone_trajectory_finder.h"

std::shared_ptr<bmpf::Scene> scene;

std::shared_ptr<bmpf::MonotoneTrajectoryFinder> mtf;

void testPath(std::vector<double> &start, std::vector<double> &end) {
    bmpf::infoMsg("test begin");
    int errorCode = -1;
    mtf->prepareTrajectory(start, end, errorCode);

    auto tr = mtf->getLastTrajectory();
    auto path = mtf->getLastPath();

    for (int i = 0; i < tr.size(); i++) {
        std::vector<double> trNode(tr.at(i).begin() + 1, tr.at(i).begin() + 1 + scene->getJointCnt());
        if (!bmpf::areStatesEqual(path.at(i), trNode, 0.0001)) {
            bmpf::infoState("path", path.at(i));
            bmpf::infoState("trNode", trNode);
            assert(false);
        }
    }

    for (int i = 0; i < tr.size() - 1; i++) {
        // infoMsg(scene->getJointCnt());
        std::vector<double> leftPos(tr.at(i).begin() + 1, tr.at(i).begin() + 1 + scene->getJointCnt());
        std::vector<double> rightPos(tr.at(i + 1).begin() + 1, tr.at(i + 1).begin() + 1 + scene->getJointCnt());

        for (double j = tr.at(i).at(0); j <= tr.at(i + 1).at(0); j += 0.0001) {
            auto trajectoryPositions = mtf->getTrajectoryPosition(j);

            std::vector<double> state(trajectoryPositions.begin() + 1, trajectoryPositions.end());
            if (bmpf::isInRange(state, leftPos, rightPos) != -1) {
                bmpf::infoMsg(j, " ", bmpf::isInRange(state, leftPos, rightPos));
                bmpf::infoState("leftPos", leftPos);
                bmpf::infoState("state", state);
                bmpf::infoState("rightPos", rightPos);
                assert(false);
            }
        }
    }

}

void test1() {
    bmpf::infoMsg("test 1");
    std::vector<double> start
            {-2.967, -0.855, 0.314, -1.937, -1.676, -3.665};

    std::vector<double> end
            {1.187, -0.035, -1.131, 0.000, 0.419, 3.665};
    testPath(start, end);
}


void test2() {
    bmpf::infoMsg("test 2");

    std::vector<double> start
            {-1.696, 0.453, -1.582, -0.569, 0.827, -2.817};
    std::vector<double> end
            {0.759, -2.957, 0.393, 3.176, 0.857, -4.351};

    testPath(start, end);
}

void test3() {
    bmpf::infoMsg("test 3");
    std::vector<double> start
            {0.424, -1.120, -0.451, 0.686, 1.911, 2.587};
    std::vector<double> end
            {0.953, -0.871, 1.649, 0.838, 0.009, -3.227};

    testPath(start, end);
}

void test4() {
    bmpf::infoMsg("test 4");
    std::vector<double> start
            {2.383, -2.842, -1.350, 2.419, -0.023, 0.089};
    std::vector<double> end
            {0.216, -0.043, 1.438, 0.904, 1.970, 0.048};

    testPath(start, end);
}

void test5() {
    bmpf::infoMsg("test 5");
    std::vector<double> start
            {0.026, -0.979, 1.400, -0.713, 0.068, 2.742};
    std::vector<double> end
            {-1.344, -0.959, 0.970, -0.756, -1.359, -0.948};
    testPath(start, end);
}

void test6() {
    bmpf::infoMsg("test 6");
    std::vector<double> start
            {-2.249, -0.468, -0.594, -2.520, 0.834, 1.116};
    std::vector<double> end
            {-2.100, 0.545, -0.183, 0.608, 0.126, 5.099};
    testPath(start, end);
}

int main() {
    bmpf::infoMsg("test one direction path finder");

    scene = std::make_shared<bmpf::Scene>();
    scene->loadFromFile("../../../config/murdf/demo_scene.json");

    mtf = std::make_shared<bmpf::MonotoneTrajectoryFinder>();
    mtf->init(scene,0.1, false, 1000, 10, 3000, 100, 1);

    test1();
    test2();
    test3();
    test4();
    test5();
    test6();


    bmpf::infoMsg("complete");
    return 0;
}
