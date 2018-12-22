

#include <generator.h>


void test1()
{

    Generator generator("/opt/tra/path_finding/config/murdf/demo_scene.json",
                        "a_star",
                        true);

    // can not find free point near start
    generator.testPathFinding(
        std::vector<double>{1.295, -2.619, 2.021, -0.704, -1.750, -1.439},
        std::vector<double>{0.324, 0.273, 0.080, 0.339, 1.472, 3.083}
    );

    // can not find free point near end
    generator.testPathFinding(
        std::vector<double>{-0.586, 0.199, 1.875, -0.056, 1.767, -5.471},
        std::vector<double>{-1.911, -0.676, -0.011, -0.584, -1.966, -1.215}
    );

    // can not find free point near end
    generator.testPathFinding(
        std::vector<double>{2.536, -0.221, 0.020, -0.313, -0.982, -1.441},
        std::vector<double>{0.885, -3.257, -1.847, 3.062, -1.758, 2.208}
    );

    // can not find free point near end
    generator.testPathFinding(
        std::vector<double>{0.741, -2.905, -0.351, 0.380, -1.692, 3.481},
        std::vector<double>{0.580, -2.306, 1.804, 1.708, 1.496, -5.737}
    );


//    generator.testPathFinding(
//        std::vector<double>{-0.821 ,-0.315 ,-0.507 ,1.304 ,-0.486 ,0.242},
//        std::vector<double>{0.915 ,-1.382 ,0.487 ,-0.251 ,-1.007 ,-5.597}
//    );

    generator.showErrorPaths();
}

int main()
{

    info_msg("testAStar");

    test1();


    return 0;
}