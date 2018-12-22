

#include <scene_wrapper.h>
#include <traces.h>
#include <solid_collider.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


#include <generator.h>


std::string scenePath;

std::string savePath;

std::string reportPath;

std::string tp;

bool trace;

std::string algorithm;

unsigned int testCnt;

bool init(int argc, char const **argv)
{
    srand(time(NULL));

    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("scene,s", po::value<std::string>(), "scene file")
        ("path,p", po::value<std::string>(), "path file")
        ("description,d", po::value<std::string>(), "type of scene description file (urdf, murdf or dh)")
        ("report,r", po::value<std::string>(), "report file")
        ("count,c", po::value<int>(), "count of tests, default is 1")
        ("algorithm,a", po::value<std::string>(), "finding algoritm (a_star, ordered_a_star, speed_a_star)")
        ("trace,t", po::value<bool>(), "show trace of path finding algorithm");


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return false;
    }

    if (vm.count("scene")) {
        scenePath = vm["scene"].as<std::string>();
    }
    else {
        std::cout << "Scene was not set." << std::endl;
        return false;
    }

    if (vm.count("path")) {
        savePath = vm["path"].as<std::string>();
    }
    else {
        std::cout << "Path was not set." << std::endl;
        return false;
    }

    if (vm.count("description")) {
        tp = vm["description"].as<std::string>();
    }
    else {
        std::cout << "Type was not set." << std::endl;
        return false;
    }

    if (tp != "urdf" && tp != "murdf" && tp != "dh") {
        std::cout << "Wrong type, you can choose only one of these: urdf, murdf or dh" << std::endl;
        return false;
    }

    if (vm.count("algorithm")) {
        algorithm = vm["algorithm"].as<std::string>();
    }
    else {
        std::cout << "Wrong algorithm, you can choose only one of these: urdf, murdf or dh" << std::endl;
    }


    if (algorithm != "a_star" && algorithm != "speed_a_star") {
        algorithm = "speed_a_star";
    }

    if (vm.count("report")) {
        reportPath = vm["report"].as<std::string>();

    }

    if (vm.count("count")) {
        testCnt = vm["count"].as<int>();
    }
    else {
        testCnt = 1;
    }


    if (vm.count("trace")) {
        trace = vm["trace"].as<bool>();
    }
    else {
        trace = false;
    }


    return true;
}

int main(int argc, char const **argv)
{
    if (!init(argc, argv))
        return -1;

    info_msg("scene: ", scenePath);
    info_msg("save path: ", savePath);
    if (!reportPath.empty())
        info_msg("report path: ", reportPath);

    if (testCnt > 1)
        info_msg("test cnt: ", testCnt);

    auto path = boost::filesystem::path(scenePath);
    info_msg(path.leaf().string());

    Generator generator(scenePath, algorithm, trace);
    generator.writeResult(savePath, reportPath, testCnt);

    info_msg("complete");
    return 0;
}
