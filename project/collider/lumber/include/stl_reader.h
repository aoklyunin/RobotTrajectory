#pragma once

#include "traces.h"

#include <SOLID_types.h>
#include <MT_Point3.h>

#include <string>
#include <vector>
#include <ios>
#include <fstream>
#include <iostream>


std::vector<float> getChords(char *facet)
{

    char f1[4] = {facet[0],
                  facet[1], facet[2], facet[3]};

    char f2[4] = {facet[4],
                  facet[5], facet[6], facet[7]};

    char f3[4] = {facet[8],
                  facet[9], facet[10], facet[11]};

    float x = *((float *) f1); // x
    float y = *((float *) f2); // y
    float z = *((float *) f3); // z
    std::vector<float> point{x, y, z};
    return point;
};

// returns traingle vertexies in order:
// x chord of normal vector,
// y chord of normal vector,
// z chord of normal vector,
// x chord of first vertex,
// y chord of first vertex,
// z chord of first vertex,
// x chord of second vertex,
// y chord of second vertex,
// z chord of second vertex,
// x chord of third vertex,
// y chord of third vertex,
// z chord of third vertex,
// if we have stl with 10 triangles return vector will have 90 elements
std::vector<float> read_stl(std::string fname)
{
 //   info_msg("loading file: ", fname);
    std::ifstream myFile(fname.c_str(), std::ios::in | std::ios::binary);

    char header_info[80] = "";
    char nTri[4] = {0};
    unsigned int nTriLong = 0;

    //read 80 byte header
    if (myFile) {
        myFile.read(header_info, 80);
        // std::cout << "header: " << header_info << std::endl;
    }
    else {
        err_msg("header error: ", fname);
    }

    //read 4-byte ulong
    if (myFile) {
        myFile.read(nTri, 4);
        nTriLong = *((unsigned int *) nTri);
        //std::cout << "n Tri: " << nTriLong << std::endl;
    }
    else {
        err_msg("error opening file");
    }

    std::vector<float> points;

    //now read in all the triangles
    for (int i = 0; i < nTriLong; i++) {
        char facet[50];
        if (myFile) {
            //read one 50-byte triangle
            myFile.read(facet, 50);
            //populate each point of the triangle
            //using v3::v3(char* bin);
            //facet + 12 skips the triangle's unit normal
            for (float chord:getChords(facet))
                points.push_back(chord);
            for (float chord:getChords(facet + 12))
                points.push_back(chord);
            for (float chord:getChords(facet + 24))
                points.push_back(chord);
            for (float chord:getChords(facet + 36))
                points.push_back(chord);
        }
    }
   // info_msg(points.size(), " points are loaded");
    return points;
}

