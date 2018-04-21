#ifndef TOOLS_OFFLOADER_H_
#define TOOLS_OFFLOADER_H_

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
#include <stdint.h>
#include <memory>
#include <map>

using namespace std::experimental::filesystem;

struct SimpleOFF
{
    int nums;
    std::vector<Eigen::Vector4f> vertices;
    // std::vector<Eigen::Vector4f> normals;
    std::vector<Eigen::Vector4f> colors;
    std::vector<Eigen::Vector4f> faces;
};

class OFFLoader
{
public:
	OFFLoader();

	~OFFLoader();

	void readFile(path, SimpleOFF&);

	std::vector<SimpleOFF> readPath(std::string m_path);

private:
    std::vector<SimpleOFF> offFiles;
	
};




























#endif