
#include <generator.h>
#include <traces.h>

int main(int argc, char const **argv)
{
    srand(time(NULL));
//    err_msg("demo");
//    Generator generator("/opt/tra/path_finding/config/murdf/demo_scene.json",
//                        "tra",
//                        true);
//
//    generator.writeResult("../../../config_deploy/paths/demo_ordered_tra_path.json",
//                          "../../../config_deploy/reports/demo_ordered_tra_report.json",
//                          10);
//
//    generator.showErrorPaths();
//
//    err_msg("demo2");
//    Generator generator2("/opt/tra/path_finding/config/murdf/demo_scene2.json",
//                         "tra",
//                         false);
//
//    generator2.writeResult("../../../config_deploy/paths/demo2_ordered_tra_path.json",
//                           "../../../config_deploy/reports/demo2_ordered_tra_report.json",
//                           10);
//
//    generator2.showErrorPaths();

    err_msg("4robots");

    Generator generator3("/opt/tra/path_finding/config/murdf/4robots.json",
                         "tra",
                         true);

    generator3.writeResult("../../../config_deploy/paths/4robot_ordered_tra_path.json",
                           "../../../config_deploy/reports/4robot_ordered_tra_report_tmp.json",
                           100);

    generator3.showErrorPaths();

    info_msg("complete");
    return 0;
}
