#include <scene.h>
#include <log.h>
#include "state.h"

#include <base/path_finder.h>
#include <one_direction_path_finder.h>

std::shared_ptr<bmpf::Scene> scene;

std::shared_ptr<bmpf::GridPathFinder> pathFinder;

void testPath(std::vector<double> &start, std::vector<double> &end) {
    bmpf::infoMsg("test begin");
    int errorCode = -1;
    std::vector<std::vector<double>> path = pathFinder->findPath(start, end, errorCode);

    if (errorCode != bmpf::PathFinder::NO_ERROR)
        bmpf::errMsg("error code:", errorCode);

    assert(errorCode == bmpf::PathFinder::NO_ERROR);
    assert(bmpf::getStateDistance(start, path.front()) < 0.0001);
    assert(bmpf::getStateDistance(end, path.back()) < 0.0001);
    assert(!path.empty());

    bmpf::infoMsg("ready");

    assert(pathFinder->simpleCheckPath(path, 100));

    bmpf::infoMsg("path is valid");

    bmpf::infoMsg(pathFinder->getCalculationTimeInSeconds(), " seconds");

    assert (errorCode == bmpf::PathFinder::NO_ERROR);

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

int main() {
    bmpf::infoMsg("test one direction path finder");

    std::shared_ptr<bmpf::Scene> sceneWrapper = std::make_shared<bmpf::Scene>();
    sceneWrapper->loadFromFile("../../../../config/murdf/4robots.json");

    pathFinder = std::make_shared<bmpf::OneDirectionPathFinder>(
            sceneWrapper, false, 1000, 10, 3000, 5, 1
    );

    test1();
    test2();
    test3();
    test4();
    test5();
    test6();

    bmpf::infoMsg("complete");
    return 0;
}
