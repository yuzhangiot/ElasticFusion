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
	Eigen::Vector3f vertex;
	utils::DualQuaternion<float> transform;
	float weight = 0;
};

class WarpField
{
public:
	WarpField(float confidenceThreshold);
	~WarpField();

    void init(const std::vector<Eigen::Vector4f>& first_frame);
    void init(DynamicModel& dynamicModel);

    std::vector<Eigen::Vector4f> warp(std::vector<Eigen::Vector4f>& points, std::vector<Eigen::Vector3f>& normals) const;

    utils::DualQuaternion<float> DQB(const Eigen::Vector3f& vertex) const;

    void getWeightsAndUpdateKNN(const Eigen::Vector3f& vertex, float weights[KNN_NEIGHBOURS]) const;

    void KNN(Eigen::Vector3f point) const;

    void buildKDTree();

    float weighting(float squared_dist, float weight) const;

    const std::vector<deformation_node>* getNodes() const;
          std::vector<deformation_node>* getNodes();

    std::vector<size_t>* getRetIndex() const;

private:
	std::vector<deformation_node>* nodes_;
	kd_tree_t* index_;
	Eigen::Affine3f warp_to_live_;

	float confidenceThreshold;
};












#endif