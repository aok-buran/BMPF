

#include <ctime>
#include <cstdlib>
#include "optimize_generator.h"

int main() {
    srand(time(nullptr));
    OptimizeGenerator generator(
            {
                    "../../../out/paths/exp_1_paths.json",
                    "../../../out/paths/exp_1_2_paths.json",
                    "../../../out/paths/exp_1_3_paths.json",
                    "../../../out/paths/exp_1_4_paths.json",
            },
            {
                    "median",
                    // "newton"
            }
    );

    generator.generate("../../../out/optimize_paths/exp_1_paths.json",
                       "../../../out/optimize_reports/exp_1_reports.json");
    return 0;
}
