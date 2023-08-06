#include <scene.h>
#include <log.h>
#include "state.h"
#include "continuous_path_finder.h"
#include "monotone_trajectory_finder.h"

#include <base/path_finder.h>
#include <one_direction_path_finder.h>

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
//        infoMsg(scene->getJointCnt());
//        infoState("tr", tr.at(i));
        std::vector<double> leftPos(tr.at(i).begin() + 1, tr.at(i).begin() + 1 + scene->getJointCnt());
        std::vector<double> rightPos(tr.at(i + 1).begin() + 1, tr.at(i + 1).begin() + 1 + scene->getJointCnt());

       // infoState("tr", tr.at(i + 1));
        std::vector<double> leftPosPath(path.at(i));
        std::vector<double> rightPosPath(path.at(i + 1));

        for (double j = tr.at(i).at(0); j <= tr.at(i + 1).at(0); j += 0.0001) {
            auto trajectoryPositions = mtf->getTrajectoryPosition(j);
            //infoState("trajectoryPositions ", trajectoryPositions);

            std::vector<double> state(trajectoryPositions.begin() + 1, trajectoryPositions.end());
            if (bmpf::isInRange(state, leftPos, rightPos) != -1) {
                bmpf::infoMsg(j, " ", bmpf::isInRange(state, leftPos, rightPos));
                bmpf::infoState("leftPos", leftPos);
                //   infoState("leftPosPath", leftPosPath);
                bmpf::infoState("state", state);
                bmpf::infoState("rightPos", rightPos);
                //   infoState("rightPosPath", rightPos);
                assert(false);
            }
        }
    }

}

void test1() {
    bmpf::infoMsg("test 1");
    std::vector<double> start
            {-2.372, -2.251, 1.977, 0.031, 1.885, 5.093, -2.043, -0.717, -0.893, 0.307, 0.687, -0.148, 0.723, 0.667,
             -1.421,
             -2.498, 1.934, -4.705, -2.144, -2.477, 1.529, 0.919, 1.333, 2.003};
    std::vector<double> end
            {0.262, -3.238, 1.314, 2.603, -0.827, -3.604, -1.641, -0.440, 1.958, 1.606, 1.474, -4.645, -2.421, -0.583,
             0.134, -0.834, 2.049, -4.375, -2.353, -2.529, 0.148, -0.707, 0.145, -2.702};
    testPath(start, end);
}

void test2() {
    bmpf::infoMsg("test 2");

    std::vector<double> start
            {-1.696, 0.453, -1.582, -0.569, 0.827, -2.817, -2.769, 0.360, 1.462, 1.441, -1.827, 5.589, -2.054, -1.892,
             -0.302, -1.915, 1.601, 5.947, 1.238, -0.023, -0.341, 0.757, 0.534, 0.494};
    std::vector<double> end
            {0.759, -2.957, 0.393, 3.176, 0.857, -4.351, 0.192, -2.326, 0.592, -0.243, 0.344, -3.707, -0.772, -0.119,
             -1.855, -1.959, -1.745, -2.263, 1.309, -0.623, 0.860, -2.320, 1.961, 1.648};

    testPath(start, end);
}

void test3() {
    bmpf::infoMsg("test 3");
    std::vector<double> start
            {0.424, -1.120, -0.451, 0.686, 1.911, 2.587, -2.711, -1.546, 0.809, 1.582, -0.477, 3.787, -1.726, -2.643,
             -0.098, -0.535, 0.694, -1.908, 2.335, -2.895, -0.173, -0.286, -1.405, -6.011};
    std::vector<double> end
            {0.953, -0.871, 1.649, 0.838, 0.009, -3.227, -2.690, -3.014, 1.621, -0.725, 0.753, 2.779, -2.377, -0.351,
             -1.328, 1.305, -0.134, 0.552, 0.072, -0.539, 1.322, 2.754, -1.700, -1.526};

    testPath(start, end);
}

void test4() {
    bmpf::infoMsg("test 4");
    std::vector<double> start
            {2.383, -2.842, -1.350, 2.419, -0.023, 0.089, 0.914, -2.245, 1.017, 2.578, 0.177, -6.047, 0.975, 0.217,
             -1.405,
             -1.892, -0.042, -4.390, 1.751, -2.111, 1.679, 0.971, 1.904, 0.275};
    std::vector<double> end
            {0.216, -0.043, 1.438, 0.904, 1.970, 0.048, -1.262, -0.689, -0.150, -2.305, 0.711, 0.922, -0.511, -1.067,
             -1.322, 2.551, 0.294, 5.391, -1.561, -0.489, 0.030, 0.495, 1.869, -3.930};

    testPath(start, end);
}

void test5() {
    bmpf::infoMsg("test 5");
    std::vector<double> start
            {0.026, -0.979, 1.400, -0.713, 0.068, 2.742, -2.683, -0.880, -0.710, -3.194, -0.169, 0.833, -0.223, 0.709,
             -0.839, 2.566, -1.977, 1.014, 0.817, -1.958, -1.504, -2.931, 1.558, 1.262};
    std::vector<double> end
            {-1.344, -0.959, 0.970, -0.756, -1.359, -0.948, 1.536, 0.240, 0.207, -1.211, 1.532, 4.826, -1.527, -1.702,
             -1.003, -1.186, -1.529, -6.033, 0.363, -0.562, 1.739, 0.567, -1.162, -0.147};
    testPath(start, end);
}

void test6() {
    bmpf::infoMsg("test 6");
    std::vector<double> start
            {-2.249, -0.468, -0.594, -2.520, 0.834, 1.116, 2.965, 0.415, 0.332, 0.374, -1.819, 1.548, -0.222, 0.090,
             -0.377,
             1.824, -1.213, 0.679, -1.040, -3.031, 0.064, 0.572, 0.087, -1.996};
    std::vector<double> end
            {-2.100, 0.545, -0.183, 0.608, 0.126, 5.099, 2.556, -0.529, 0.692, 0.461, -1.953, -4.132, -0.066, -2.354,
             2.078,
             -2.577, 1.035, 2.205, -0.522, 0.173, 0.287, 0.857, -0.176, 4.473};
    testPath(start, end);
}

void test7() {
    bmpf::infoMsg("test 7");

    std::vector<double> start
            {2.891, 0.101, -1.610, -0.915, 0.241, 3.377, 0.982, -1.895, -1.725, 1.885, 1.530, -2.532, -2.037, -2.228,
             0.126,
             -1.387, -1.135, -4.978, -2.876, -0.069, 1.063, -2.898, -1.186, 5.017};

    std::vector<double> end
            {-1.169, -3.184, 0.133, -1.435, -1.304, 2.631, -0.411, -2.594, 0.547, 0.203, 0.143, -4.815, -1.140, -2.497,
             0.085, -0.745, 2.059, -2.227, 1.052, -2.708, 0.714, -2.336, -0.279, 3.811};

    testPath(start, end);
}


void test8() {

    bmpf::infoMsg("test 8");
    std::vector<double> start
            {2.891, 0.101, -1.610, -0.915, 0.241, 3.377, 0.982, -1.895, -1.725, 1.885, 1.530, -2.532, -2.037, -2.228,
             0.126,
             -1.387, -1.135, -4.978, -2.876, -0.069, 1.063, -2.898, -1.186, 5.017};
    std::vector<double> end
            {-1.169, -3.184, 0.133, -1.435, -1.304, 2.631, -0.411, -2.594, 0.547, 0.203, 0.143, -4.815, -1.140, -2.497,
             0.085, -0.745, 2.059, -2.227, 1.052, -2.708, 0.714, -2.336, -0.279, 3.811};
    testPath(start, end);
}


void test9() {
    bmpf::infoMsg("test 9");
    std::vector<double> start
            {2.891, 0.101, -1.610, -0.915, 0.241, 3.377, 0.982, -1.895, -1.725, 1.885, 1.530, -2.532, -2.037, -2.228,
             0.126,
             -1.387, -1.135, -4.978, -2.876, -0.069, 1.063, -2.898, -1.186, 5.017};

    std::vector<double> end
            {-1.169, -3.184, 0.133, -1.435, -1.304, 2.631, -0.411, -2.594, 0.547, 0.203, 0.143, -4.815, -1.140, -2.497,
             0.085, -0.745, 2.059, -2.227, 1.052, -2.708, 0.714, -2.336, -0.279, 3.811};
    testPath(start, end);
}


int main() {
    bmpf::infoMsg("test one direction path finder");

    scene = std::make_shared<bmpf::Scene>();
    scene->loadFromFile("../../../config/murdf/4robots.json");

    mtf = std::make_shared<bmpf::MonotoneTrajectoryFinder>();
    mtf->init(scene,0.1, false, 1000, 10, 3000, 100, 1);

    test1();
    test2();
    test3();
    test4();
    test5();
    test6();
    test7();
    test8();
    test9();

    bmpf::infoMsg("complete");
    return 0;
}
