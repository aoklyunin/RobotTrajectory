
#include <generator.h>
#include <traces.h>

int main(int argc, char const **argv)
{
    srand(time(NULL));
    Generator generator("/opt/tra/path_finding/config/murdf/demo_scene.json",
                        "a_star",
                        false);

    generator.writeResult("../../../config_deploy/paths/demo_a_star_path.json",
                          "../../../config_deploy/reports/demo_a_star_report.json",
                          40);

    Generator generator2("/opt/tra/path_finding/config/murdf/demo_scene2.json",
                         "a_star",
                         false);


    generator2.writeResult("../../../config_deploy/paths/demo2_a_star_path.json",
                           "../../../config_deploy/reports/demo2_a_star_report.json",
                           40);


    Generator generator3("/opt/tra/path_finding/config/murdf/4robots.json",
                         "a_star",
                         false);

    generator3.writeResult("../../../config_deploy/paths/4robot_a_star_path.json",
                           "../../../config_deploy/reports/4robot_a_star_report.json",
                           20);

    generator.showErrorPaths();
    generator2.showErrorPaths();
    generator3.showErrorPaths();

    // info_msg("complete");
    return 0;
}
