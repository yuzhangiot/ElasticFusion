#include "WarpFieldOptimizer.h"


WarpFieldOptimizer::WarpFieldOptimizer(WarpField *warp, CombinedSolver *solver)
:
 warp_(warp),
 solver_(solver)
{

}

WarpFieldOptimizer::WarpFieldOptimizer(WarpField *warp, CombinedSolverParameters params)
:
 warp_(warp)
{
	solver_ = new CombinedSolver(warp, params);
}

WarpFieldOptimizer::~WarpFieldOptimizer() {

}


void WarpFieldOptimizer::optimiseWarpData(const std::vector<Eigen::Vector3f> &canonical_vertices,
                                                   const std::vector<Eigen::Vector3f> &canonical_normals,
                                                   const std::vector<Eigen::Vector3f> &live_vertices,
                                                   const std::vector<Eigen::Vector3f> &live_normals)
{
    solver_->initializeProblemInstance(canonical_vertices,
                                       canonical_normals,
                                       live_vertices,
                                       live_normals);
    solver_->solveAll();
}