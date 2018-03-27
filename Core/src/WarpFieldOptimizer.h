#ifndef WARP_FIELD_OPTIMIZER_H_
#define WARP_FIELD_OPTIMIZER_H_

#include "WarpField.h"
#include "Opts/CombinedSolver.h"


class WarpFieldOptimizer
{
public:
	WarpFieldOptimizer(WarpField *warp, CombinedSolver *solver);

	WarpFieldOptimizer(WarpField *warp, CombinedSolverParameters params);

	~WarpFieldOptimizer();

	void optimiseWarpData(const std::vector<Eigen::Vector3f> &canonical_vertices,
                              const std::vector<Eigen::Vector3f> &canonical_normals,
                              const std::vector<Eigen::Vector3f> &live_vertices,
                              const std::vector<Eigen::Vector3f> &live_normals);


private:
    WarpField *warp_;
    CombinedSolver *solver_;
	
};






































#endif