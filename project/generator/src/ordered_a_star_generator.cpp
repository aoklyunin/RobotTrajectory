
#include <generator.h>
#include <traces.h>

int main(int argc, char const **argv)
{
    srand(time(NULL));
    Generator generator("/opt/tra/path_finding/config/murdf/demo_scene.json",
                        "ordered_a_star",
                        true);
    generator.writeResult("../../../config_deploy/paths/demo_ordered_a_star_path.json",
                          "../../../config_deploy/reports/demo_ordered_a_star_report.json",
                          40);


    Generator generator2("/opt/tra/path_finding/config/murdf/demo_scene2.json",
                         "ordered_a_star",
                         true);

    generator2.writeResult("../../../config_deploy/paths/demo2_ordered_a_star_path.json",
                           "../../../config_deploy/reports/demo2_ordered_a_star_report.json",
                           40);

    Generator generator3("/opt/tra/path_finding/config/murdf/4robots.json",
                         "ordered_a_star",
                         true);

    generator3.writeResult("../../../config_deploy/paths/4robot_ordered_a_star_path.json",
                           "../../../config_deploy/reports/4robot_ordered_a_star_report.json",
                           20);


    info_msg("complete");
    return 0;
}
