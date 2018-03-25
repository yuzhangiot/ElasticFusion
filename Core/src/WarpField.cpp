#include "WarpField.h"

std::vector<utils::DualQuaternion<float>> neighbours;
utils::PointCloud cloud;
nanoflann::KNNResultSet<float> *resultSet_;
std::vector<float> out_dist_sqr_;
std::vector<size_t> ret_index_;

WarpField::WarpField() {
	nodes_ = new std::vector<deformation_node>();
	index_ = new kd_tree_t(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	ret_index_ = std::vector<size_t>(KNN_NEIGHBOURS);
    out_dist_sqr_ = std::vector<float>(KNN_NEIGHBOURS);
    resultSet_ = new nanoflann::KNNResultSet<float>(KNN_NEIGHBOURS);
    resultSet_->init(&ret_index_[0], &out_dist_sqr_[0]);
    neighbours = std::vector<utils::DualQuaternion<float>>(KNN_NEIGHBOURS);
    warp_to_live_ = Eigen::Affine3f();
}

WarpField::~WarpField() {
	// delete nodes_;
	delete resultSet_;
	delete index_;
}