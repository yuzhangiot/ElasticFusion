#ifndef WARPFIELD_H_
#define WARPFIELD_H_

#include "Utils/DualQuaternion.h"
#include "Utils/nanoflann.hpp"
#include "Utils/KnnPointCloud.hpp"
#include "DynamicModel.h"

#define KNN_NEIGHBOURS 8

typedef nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<float, utils::PointCloud>,
            utils::PointCloud,
            3 /* dim */
    > kd_tree_t;

struct deformation_node
{
	Eigen::Vector4f vertex;
	utils::DualQuaternion<float> transform;
};

class WarpField
{
public:
	WarpField(float confidenceThreshold);
	~WarpField();

    void init(const std::vector<Eigen::Vector4f>& first_frame);
    void init(DynamicModel& dynamicModel);

    void buildKDTree();

private:
	std::vector<deformation_node>* nodes_;
	kd_tree_t* index_;
	Eigen::Affine3f warp_to_live_;

	float confidenceThreshold;
};












#endif