#include "PLYLoader.h"



    PLYLoader::PLYLoader(){

    }

    PLYLoader::~PLYLoader(){

    }

    void PLYLoader::readFile() {
        
    }

    void PLYLoader::readPath(std::string m_path) {
        std::vector<path> files;
        for(auto& f : recursive_directory_iterator(m_path)) {
            cout << f << endl;
        }

        // return plyfiles;
    }