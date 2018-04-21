#include "OFFLoader.h"

using namespace std;

OFFLoader::OFFLoader() {

}



OFFLoader::~OFFLoader() {

}


void OFFLoader::readFile(path f_path, SimpleOFF& s_offFile) {
	//read file
	ifstream ss(f_path.string(), ios::binary);

	if(ss.fail()) {
		throw runtime_error("failed to open " + f_path.string());
	}

	// read header
	std::string line;

	std::getline(ss, line);
	std::istringstream ls_tag(line);
    std::string token;
    ls_tag >> token;

    if (token == "COFF") {
    	std::cout << "Header TAG is correct, continue reading" << std::endl;
    }

    std::getline(ss, line);
    std::istringstream ls_nums(line);
    std::string vertex_num, face_num;
    ls_nums >> vertex_num;
    ls_nums >> face_num;
    std::cout << "vertex num is: " << vertex_num << std::endl;
    std::cout << "face num is: " << face_num << std::endl;

    // read body and copy data to SimpleOFF
    s_offFile.nums = stoi(vertex_num);

    s_offFile.vertices.resize(s_offFile.nums);
    // s_offFile.normals.resize(vertex_num);
    s_offFile.colors.resize(s_offFile.nums);
    s_offFile.faces.resize(stoi(face_num));

    // vertices
    for(auto vertex_id = 0; vertex_id < std::stoi(vertex_num); ++vertex_id) {
    	std::getline(ss, line);
    	std::istringstream ls(line);
    	std::string v_x, v_y, v_z, c_r, c_g, c_b;
    	ls >> v_x;
    	ls >> v_y;
    	ls >> v_z;
    	ls >> c_r;
    	ls >> c_g;
    	ls >> c_b;

    	s_offFile.vertices[vertex_id][0] = std::stof(v_x);
    	s_offFile.vertices[vertex_id][1] = std::stof(v_y);
    	s_offFile.vertices[vertex_id][2] = std::stof(v_z);
    	s_offFile.vertices[vertex_id][3] = 1.f;

    	s_offFile.colors[vertex_id][0] = std::stof(c_r);
    	s_offFile.colors[vertex_id][1] = std::stof(c_g);
    	s_offFile.colors[vertex_id][2] = std::stof(c_b);
    	s_offFile.colors[vertex_id][3] = 1.f;
    }

    // faces
    for(auto face_id = 0; face_id < std::stoi(face_num); ++face_id) {
    	std::getline(ss, line);
    	std::istringstream ls(line);
    	std::string f_x, f_y, f_z;
    	ls >> f_x;
    	ls >> f_y;
    	ls >> f_z;

    	s_offFile.faces[face_id][0] = std::stof(f_x);
    	s_offFile.faces[face_id][1] = std::stof(f_y);
    	s_offFile.faces[face_id][2] = std::stof(f_z);
    	s_offFile.faces[face_id][3] = 1.f;
    }
}


std::vector<SimpleOFF> OFFLoader::readPath(std::string d_path) {
	std::vector<path> files;
    for(auto& f : recursive_directory_iterator(d_path)) {
    	files.push_back(f);
    }

    sort(files.begin(), files.end());

    for(auto& ff : files) {
    	SimpleOFF tmpFile;
    	readFile(ff, tmpFile);
    	offFiles.push_back(tmpFile);
    }

    return offFiles;
}