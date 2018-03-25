#include "WarpField.h"

std::vector<utils::DualQuaternion<float>> neighbours;
utils::PointCloud cloud;
nanoflann::KNNResultSet<float> *resultSet_;
std::vector<float> out_dist_sqr_;
std::vector<size_t> ret_index_;

WarpField::WarpField(float confidenceThreshold)
:
 confidenceThreshold(confidenceThreshold)
{
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
	delete nodes_;
	delete resultSet_;
	delete index_;
}


/**
 *
 * @param first_frame
 * @param normals
 */
void WarpField::init(const std::vector<Eigen::Vector4f>& first_frame)
{
    nodes_->resize(first_frame.size());

//    FIXME: this is a test, remove
    for (size_t i = 0; i < first_frame.size(); i++)
    {
        auto point = first_frame[i];
        if (!std::isnan(point[0]))
        {
            nodes_->at(i).transform = utils::DualQuaternion<float>();
            nodes_->at(i).vertex = point;
        }
    }
    buildKDTree();
}

void WarpField::init(DynamicModel& dynamicModel) {
	auto point_num = dynamicModel.lastCount();
	nodes_->resize(point_num);
	int step = 50;

	Eigen::Vector4f * mapData = dynamicModel.downloadMap();

	for (int i = 0; i < point_num; i += step) {
		Eigen::Vector4f pos = mapData[(i * 3) + 0];

        // if(pos[3] > confidenceThreshold) {
        	nodes_->at(i).transform = utils::DualQuaternion<float>();
            nodes_->at(i).vertex = pos;
        // }
	}
	buildKDTree();
}

void WarpField::buildKDTree()
{
    //    Build kd-tree with current warp nodes.
    cloud.pts.resize(nodes_->size());
    for(size_t i = 0; i < nodes_->size(); i++)
        cloud.pts[i] = nodes_->at(i).vertex;
    index_->buildIndex();
}




