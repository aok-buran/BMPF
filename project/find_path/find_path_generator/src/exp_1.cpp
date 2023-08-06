
#include <generator.h>

int main() {
    srand(time(nullptr));
    Generator generator("../../../../config/murdf/demo_scene2.json",
                        {"one_direction", "ordered_one_direction", "all_directions", "multirobot",
                         "continuous"},
                        100, false);

    generator.generate("../../../../out/paths/exp_1_5_paths.json",
                       "../../../../out/reports/exp_1_5_report.json", 3);
    return 0;
}
