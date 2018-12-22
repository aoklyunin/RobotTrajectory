#pragma once

#include <path_finder.h>

#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <json/json.h>
#include <memory>


class Generator
{
public:
    Generator(std::string scenePath,
              std::string algorithm,
              bool trace
    );

    Generator(std::shared_ptr<PathFinder> pathFinder);

    std::string
    buildStat(std::vector<std::vector<double>> startPoints,
              std::vector<std::vector<double>> endPoints,
              std::vector<double> seconds,
              std::vector<bool> isValid);

    Json::Value generateRoutes(unsigned int testCnt);

    void writeResult(std::string routePath, std::string reportPath, unsigned int testCnt);

    Json::Value testPathFinding(std::vector<double> start,
                                std::vector<double> end);

    void showErrorPaths();

private:

    std::shared_ptr<PathFinder> _pathFinder;

    std::vector<std::vector<double>> _startPoints;
    std::vector<std::vector<double>> _endPoints;
    std::vector<double> _secondsList;
    std::vector<bool> _isValid;
    std::vector<int> _errorCodes;
};