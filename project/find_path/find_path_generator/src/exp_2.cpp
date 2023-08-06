
#include <generator.h>

int main() {
    srand(time(nullptr));
    Generator generator("../../../../config/murdf/4robots.json",
                        {"one_direction", "ordered_one_direction", "multirobot",
                         "continuous"},
                        15, false);

    generator.generate("../../../../out/paths/exp_2_8_paths.json",
                       "../../../../out/reports/exp_2_8_report.json", 5);
    return 0;
}
