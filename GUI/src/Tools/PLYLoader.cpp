

#include "PLYLoader.h"

using namespace std;

PLYLoader::PLYLoader(){

}

PLYLoader::~PLYLoader(){

}

void PLYLoader::readFile(path f_path, SimplePLY& s_plyFile) {
	try {
		//read file
		ifstream ss(f_path.string(), ios::binary);

		if(ss.fail()) {
			throw runtime_error("failed to open " + f_path.string());
		}

		PlyFile file;

		// read header
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

		// read body
		std::shared_ptr<PlyData> vertices, normals, colors, faces, texcoords;

		try{ vertices = file.request_properties_from_element("vertex", {"x", "y", "z"}); }
		catch (const exception& e) { cerr << "tinyply exception: " << e.what() << endl; }

		try{ normals = file.request_properties_from_element("vertex", {"nx", "ny", "nz", "radius"}); }
		catch (const exception& e) { cerr << "tinyply exception: " << e.what() << endl; }

		try{ colors = file.request_properties_from_element("vertex", {"red", "green", "blue"}); }
		catch (const exception& e) { cerr << "tinyply exception: " << e.what() << endl; }

		try{ faces = file.request_properties_from_element("face", {"vertex_indices"}); }
		catch (const exception& e) { cerr << "tinyply exception: " << e.what() << endl; }

		try{ texcoords = file.request_properties_from_element("face", {"texcoord"}); }
		catch (const exception& e) { cerr << "tinyply exception: " << e.what() << endl; }

		file.read(ss);

		if(vertices) cout << "\tread " << vertices->count << " total vertices" << endl;
		if(normals) cout << "\tread " << normals->count << " total normals" << endl;
		if(colors) cout << "\tread " << colors->count << " total colors" << endl;
		if(faces) cout << "\tread  " << faces->count << " total faces" << endl;
		if(texcoords) cout << "\tread " << texcoords->count << " total texcoords" << endl;

		// copy data to Eigen
		s_plyFile.nums = vertices->count;

		const size_t numVerticesBytes = vertices->buffer.size_bytes();
		struct float3 { float x, y, z; };
		vector<float3> tmp_vertices(vertices->count);
		memcpy(tmp_vertices.data(), vertices->buffer.get(), numVerticesBytes);
		s_plyFile.vertices.resize(vertices->count);
		for(auto i = 0; i < vertices->count; ++i) {
			s_plyFile.vertices[i][0] = tmp_vertices[i].x;
			s_plyFile.vertices[i][1] = tmp_vertices[i].y;
			s_plyFile.vertices[i][2] = tmp_vertices[i].z;
			s_plyFile.vertices[i][3] = 1.0f;
		}

		const size_t numColorsBytes = colors->buffer.size_bytes();
		struct uchar3 { unsigned char x, y, z;};
		vector<uchar3> tmp_colors(colors->count);
		memcpy(tmp_colors.data(), colors->buffer.get(), numColorsBytes);
		s_plyFile.colors.resize(vertices->count);
		for(auto i = 0; i < colors->count; ++i) {
			s_plyFile.colors[i][0] = (float)tmp_colors[i].x / 255.f;
			s_plyFile.colors[i][1] = (float)tmp_colors[i].y / 255.f;
			s_plyFile.colors[i][2] = (float)tmp_colors[i].z / 255.f;
			s_plyFile.colors[i][3] = 1.0f;
			// if((float)tmp_colors[i].x != 0)
			// 	std::cout << "r: " << (float)tmp_colors[i].x << ", g: " << (float)tmp_colors[i].y << ", b: " << (float)tmp_colors[i].z << std::endl;
		}

		const size_t numNormalsBytes = normals->buffer.size_bytes();
		struct float4 { float x, y, z, w; };
		vector<float4> tmp_normals(normals->count);
		memcpy(tmp_normals.data(), normals->buffer.get(), numNormalsBytes);
		s_plyFile.normals.resize(normals->count);
		for(auto i = 0; i < vertices->count; ++i) {
			s_plyFile.normals[i][0] = tmp_normals[i].x;
			s_plyFile.normals[i][1] = tmp_normals[i].y;
			s_plyFile.normals[i][2] = tmp_normals[i].z;
			s_plyFile.normals[i][3] = tmp_normals[i].w;
		}

	}
	catch (const exception& e) {
		cerr << "Caught tinyply exception: " << e.what() << endl;
	}

	


}

std::vector<SimplePLY> PLYLoader::readPath(std::string d_path) {
    std::vector<path> files;
    for(auto& f : recursive_directory_iterator(d_path)) {
    	files.push_back(f);
    }

    sort(files.begin(), files.end());

    for(auto& ff : files) {
    	SimplePLY tmpFile;
    	readFile(ff, tmpFile);
    	plyfiles.push_back(tmpFile);
    }

    return plyfiles;
}