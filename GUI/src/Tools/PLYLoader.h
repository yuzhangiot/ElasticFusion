/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef TOOLS_PLYLOADER_H_
#define TOOLS_PLYLOADER_H_


#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <experimental/filesystem>
#include <sstream>
#include <fstream>
#include <cstring>
#include <thread>
#include <chrono>

#include "tinyply.h"


using namespace std::experimental::filesystem;
using namespace tinyply;

struct SimplePLY
{
    int nums;
    std::vector<Eigen::Vector4f> vertices;
    std::vector<Eigen::Vector4f> normals;
    std::vector<Eigen::Vector4f> colors;
};


class PLYLoader
{
    public:
        PLYLoader();

        ~PLYLoader();

        void readFile(path, SimplePLY&);

        std::vector<SimplePLY> readPath(std::string m_path);



    private:
        std::vector<SimplePLY> plyfiles;
};


#endif
