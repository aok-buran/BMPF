
#include <generator.h>
#include "optimize_generator.h"


int main() {
    srand(time(nullptr));
    OptimizeGenerator generator(
            {
                    "../../../out/paths/exp_2_paths.json",
//                    "../../../out/paths/exp_2_2_paths.json",
//                    "../../../out/paths/exp_2_3_paths.json",
//                    "../../../out/paths/exp_2_4_paths.json",
//                    "../../../out/paths/exp_2_5_paths.json",
//                    "../../../out/paths/exp_2_6_paths.json",
//                    "../../../out/paths/exp_2_7_paths.json",
            },
            {
                    "median",
            //        "newton"
            }
    );

    generator.generate("../../../out/optimize_paths/exp_2_paths.json",
                       "../../../out/optimize_reports/exp_2_reports.json");
    return 0;
}
