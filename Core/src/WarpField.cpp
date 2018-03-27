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
            nodes_->at(i).vertex = point.head<3>();
            nodes_->at(i).weight = point[3];
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
            nodes_->at(i).vertex = pos.head<3>();
            nodes_->at(i).weight = pos[3];
        // }
	}
	buildKDTree();
}

void WarpField::buildKDTree()
{
    //Build kd-tree with current warp nodes.
    cloud.pts.resize(nodes_->size());
    for(size_t i = 0; i < nodes_->size(); i++)
        cloud.pts[i] = nodes_->at(i).vertex;
    index_->buildIndex();
}

void WarpField::KNN(Eigen::Vector3f point) const
{
	//TODO : check if point's memory has been changed
    resultSet_->init(&ret_index_[0], &out_dist_sqr_[0]);
    float tmp[3];
    tmp[0] = point[0];
    tmp[1] = point[1];
    tmp[2] = point[2];
    index_->findNeighbors(*resultSet_, tmp, nanoflann::SearchParams(10));
}

float WarpField::weighting(float squared_dist, float weight) const
{
    return (float) exp(-squared_dist / (2 * weight * weight));
}

void WarpField::getWeightsAndUpdateKNN(const Eigen::Vector3f& vertex, float weights[KNN_NEIGHBOURS]) const
{
    KNN(vertex);
    for (size_t i = 0; i < KNN_NEIGHBOURS; i++)
        weights[i] = weighting(out_dist_sqr_[i], nodes_->at(ret_index_[i]).weight);
}

utils::DualQuaternion<float> WarpField::DQB(const Eigen::Vector3f& vertex) const
{
    float weights[KNN_NEIGHBOURS];
    getWeightsAndUpdateKNN(vertex, weights);
    utils::Quaternion<float> translation_sum(0,0,0,0);
    utils::Quaternion<float> rotation_sum(0,0,0,0);
    for (size_t i = 0; i < KNN_NEIGHBOURS; i++)
    {
        translation_sum += weights[i] * nodes_->at(ret_index_[i]).transform.getTranslation();
        rotation_sum += weights[i] * nodes_->at(ret_index_[i]).transform.getRotation();
    }
    rotation_sum.normalize();
    auto res = utils::DualQuaternion<float>(translation_sum, rotation_sum);
    return res;
}

void WarpField::warp(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals) const
{
    int i = 0;
    for (auto& point : points)
    {
        if(std::isnan(point[0]) || std::isnan(normals[i][0]))
            continue;
        utils::DualQuaternion<float> dqb = DQB(point);
        dqb.transform(point);
        point = warp_to_live_ * point;

        dqb.transform(normals[i]);
        normals[i] = warp_to_live_ * normals[i];
        i++;

    }
}

const std::vector<deformation_node>* WarpField::getNodes() const
{
    return nodes_;
}

/**
 * \brief
 * \return
 */
std::vector<deformation_node>* WarpField::getNodes()
{
    return nodes_;
}

std::vector<size_t>* WarpField::getRetIndex() const
{
    return &ret_index_;
}



