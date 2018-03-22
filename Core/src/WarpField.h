#ifndef WARPFIELD_H_
#define WARPFIELD_H_

#include "DualQuaternion.h"

struct deformation_node
{
	Eigen::Vector3f vertex;
	DualQuaternion<float> transform;
	float weight = 0;
};

class WarpField
{
public:
	WarpField();
	~WarpField();

private:
	std::vector<deformation_node>* nodes_;
	
	
};












#endif