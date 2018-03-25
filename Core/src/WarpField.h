#ifndef WARPFIELD_H_
#define WARPFIELD_H_

#include "Utils/DualQuaternion.h"
#include "Utils/nanoflann.hpp"
#include "Utils/KnnPointCloud.hpp"

#define KNN_NEIGHBOURS 8

typedef nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<float, utils::PointCloud>,
            utils::PointCloud,
            3 /* dim */
    > kd_tree_t;

struct deformation_node
{
	Eigen::Vector3f vertex;
	utils::DualQuaternion<float> transform;
	float weight = 0;
};

class WarpField
{
public:
	WarpField();
	~WarpField();

private:
	std::vector<deformation_node>* nodes_;
	kd_tree_t* index_;
	Eigen::Affine3f warp_to_live_;
};












#endif