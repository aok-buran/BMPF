#include <scene.h>
#include <base/path_finder.h>
#include <base/grid_path_finder.h>
#include <one_direction_ordered_path_finder.h>


/**
 * Приложение для демонстрации поиска свободной точки
 * вблизи заданного состояния
 */
int main() {
    srand(time(NULL));

    bmpf::infoMsg("demo free point finding");

    std::shared_ptr<bmpf::Scene> scene = std::make_shared<bmpf::Scene>();
    scene->loadFromFile("../../../../config/murdf/4robots.json");


    std::shared_ptr<bmpf::GridPathFinder> pathFinder;

    pathFinder = std::make_shared<bmpf::OneDirectionOrderedPathFinder>(
            scene, true, 30, 10, 1000, 1, 2
    );

    int nonValidCnt = 0;

    for (unsigned int i = 0; i < 10; i++) {
        auto state = scene->getRandomState();
        auto coords = pathFinder->stateToCoords(state);
        if (coords.empty()) {
            bmpf::infoState("random state", state);
            bmpf::infoMsg("NON VALID");
            nonValidCnt++;
        } else
            bmpf::infoMsg("VALID");
    }

    bmpf::infoMsg("non valid: ", nonValidCnt);

    return 0;
}