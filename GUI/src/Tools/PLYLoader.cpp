#include "PLYLoader.h"

using namespace std;

PLYLoader::PLYLoader(){

}

PLYLoader::~PLYLoader(){

}

void PLYLoader::readFile(path f_path, PlyFile& plyFile) {
	try {
		ifstream ss(f_path.string(), ios::binary);

		if(ss.fail()) {
			throw runtime_error("failed to open " + f_path.string());
		}

		PlyFile file;

		file.parse_header(ss);

		cout << "====================================================\n";

		for(auto c : file.get_comments())
			cout << "Comment: " << c << endl;

		for(auto e : file.get_elements()) {
			cout << "element - " << e.name << " (" << e.size << " )" << endl;
			for(auto p : e.properties) {
				cout << "\tproperty - " << p.name << " (" << PropertyTable[p.propertyType].str << ")" << endl;
			}
		}

		cout << "====================================================\n";
	}
	catch (const exception& e) {
		cerr << "Caught tinyply exception: " << e.what() << endl;
	}
}

std::vector<PlyFile> PLYLoader::readPath(std::string d_path) {
    std::vector<path> files;
    for(auto& f : recursive_directory_iterator(d_path)) {
    	files.push_back(f);
    }

    sort(files.begin(), files.end());

    for(auto& ff : files) {
    	PlyFile tmpFile;
    	readFile(ff, tmpFile);
    	// if(success)
    	// 	plyfiles.push_back(tmpFile);
    }

    return plyfiles;
}