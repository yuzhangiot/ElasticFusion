#ifndef COMBINED_SOLVER_H_
#define COMBINED_SOLVER_H_

#include <cuda_runtime.h>
#include <cuda_profiler_api.h>
#include <cudaUtil.h>
#include <SolverIteration.h>
#include <CombinedSolverParameters.h>
#include <CombinedSolverBase.h>
#include <OptGraph.h>
#include <cuda_profiler_api.h>
#include "../Utils/macro_utils.hpp"
#include "../WarpField.h"

#include <Eigen/Core>
#include <Eigen/LU>


class CombinedSolver : public CombinedSolverBase
{
public:
	CombinedSolver(WarpField* warpField, CombinedSolverParameters params) {
		m_combinedSolverParameters = params;
        m_warp = warpField;
	}
	~CombinedSolver(){};

	void initializeProblemInstance(const std::vector<Eigen::Vector3f> &canonical_vertices,
                                   const std::vector<Eigen::Vector3f> &canonical_normals,
                                   const std::vector<Eigen::Vector3f> &live_vertices,
                                   const std::vector<Eigen::Vector3f> &live_normals)
    {
        m_canonicalVerticesEigen = canonical_vertices;
        m_canonicalNormalsEigen = canonical_normals;
        m_liveVerticesEigen = live_vertices;
        m_liveNormalsEigen = live_normals;


        unsigned int D = m_warp->getNodes()->size();
        unsigned int N = canonical_vertices.size();

        m_dims = { D, N };

        m_rotationDeform    = createEmptyOptImage({D}, OptImage::Type::FLOAT, 3, OptImage::GPU, true);
        m_translationDeform = createEmptyOptImage({D}, OptImage::Type::FLOAT, 3, OptImage::GPU, true);

        m_canonicalVerticesOpt = createEmptyOptImage({N}, OptImage::Type::FLOAT, 3, OptImage::GPU, true);
        m_liveVerticesOpt      = createEmptyOptImage({N}, OptImage::Type::FLOAT, 3, OptImage::GPU, true);

        m_canonicalNormalsOpt  = createEmptyOptImage({N}, OptImage::Type::FLOAT, 3, OptImage::GPU, true);
        m_liveNormalsOpt       = createEmptyOptImage({N}, OptImage::Type::FLOAT, 3, OptImage::GPU, true);

        m_weights              = createEmptyOptImage({N}, OptImage::Type::FLOAT, KNN_NEIGHBOURS, OptImage::GPU, true);

        resetGPUMemory();
        initializeConnectivity(m_canonicalVerticesEigen);

#ifdef SOLVER_PATH
        if(m_solverInfo.size() == 0)
        {
            std::string solver_file = std::string(TOSTRING(SOLVER_PATH)) + "dynamicfusion.t";
            addOptSolvers(m_dims, solver_file);
        }
#else
        std::cerr<<"Please define a path for your solvers."<<std::endl;
        exit(-1);
#endif
    }

    void resetGPUMemory()
    {
        uint N = (uint)m_canonicalVerticesEigen.size();
        std::vector<float3> h_canonical_vertices(N);
        std::vector<float3> h_canonical_normals(N);
        std::vector<float3> h_live_vertices(N);
        std::vector<float3> h_live_normals(N);

        for(int i = 0; i < (int)N; i++)
        {
//            FIXME: this code could look better
            if(std::isnan(m_canonicalVerticesEigen[i][0]) ||
               std::isnan(m_canonicalVerticesEigen[i][1]) ||
               std::isnan(m_canonicalVerticesEigen[i][2])) continue;

            if(std::isnan(m_canonicalNormalsEigen[i][0]) ||
               std::isnan(m_canonicalNormalsEigen[i][1]) ||
               std::isnan(m_canonicalNormalsEigen[i][2])) continue;

            if(std::isnan(m_liveVerticesEigen[i][0]) ||
               std::isnan(m_liveVerticesEigen[i][1]) ||
               std::isnan(m_liveVerticesEigen[i][2])) continue;

            if(std::isnan(m_liveNormalsEigen[i][0]) ||
               std::isnan(m_liveNormalsEigen[i][1]) ||
               std::isnan(m_liveNormalsEigen[i][2])) continue;


            h_canonical_vertices[i] = make_float3(m_canonicalVerticesEigen[i][0], m_canonicalVerticesEigen[i][1], m_canonicalVerticesEigen[i][2]);
            h_canonical_normals[i] = make_float3(m_canonicalNormalsEigen[i][0], m_canonicalNormalsEigen[i][1], m_canonicalNormalsEigen[i][2]);
            h_live_vertices[i] = make_float3(m_liveVerticesEigen[i][0], m_liveVerticesEigen[i][1], m_liveVerticesEigen[i][2]);
            h_live_normals[i] = make_float3(m_liveNormalsEigen[i][0], m_liveNormalsEigen[i][1], m_liveNormalsEigen[i][2]);
        }
        m_canonicalVerticesOpt->update(h_canonical_vertices);
        m_canonicalNormalsOpt->update(h_canonical_normals);
        m_liveVerticesOpt->update(h_live_vertices);
        m_liveNormalsOpt->update(h_live_normals);

        uint D = (uint)m_warp->getNodes()->size();
        std::vector<float3> h_translation(D);
        std::vector<float3> h_rotation(D);

        for(int i = 0; i < (int)m_warp->getNodes()->size(); i++)
        {
            float x,y,z;
            auto t = m_warp->getNodes()->at(i).transform;
            t.getTranslation(x,y,z);
            h_translation[i] = make_float3(x,y,z);

            t.getRotation().getRodrigues(x,y,z);
            h_rotation[i] = make_float3(x,y,z);
        }

        m_rotationDeform->update(h_rotation);
        m_translationDeform->update(h_translation);
    }

    void initializeConnectivity(const std::vector<Eigen::Vector3f> canonical_vertices)
    {
        unsigned int N = (unsigned int) canonical_vertices.size();

        std::vector<std::vector<int> > graph_vector(KNN_NEIGHBOURS + 1, std::vector<int>(N));
//        std::vector<float> weights(N * KNN_NEIGHBOURS);
        std::vector<float[KNN_NEIGHBOURS]> weights(N);
//FIXME: KNN doesn't need to be recomputed every time.
        for(int count = 0; count < (int)canonical_vertices.size(); count++)
        {
            graph_vector[0].push_back(count);
            m_warp->getWeightsAndUpdateKNN(canonical_vertices[count], weights[count]);
            for(int i = 1; i < (int)graph_vector.size(); i++)
                graph_vector[i].push_back((int)m_warp->getRetIndex()->at(i-1));
        }
        m_weights->update(weights);
        m_data_graph = std::make_shared<OptGraph>(graph_vector);

    }

    void copyResultToCPUFromFloat3()
    {
        unsigned int N = (unsigned int)m_warp->getNodes()->size();
        std::vector<float3> h_translation(N);
        m_translationDeform->copyTo(h_translation);

        for (unsigned int i = 0; i < N; i++)
            m_warp->getNodes()->at(i).transform.encodeTranslation(h_translation[i].x, h_translation[i].y, h_translation[i].z);
    }

    virtual void combinedSolveInit() override
    {
        m_functionTolerance = 1e-6f;
        m_paramTolerance = 1e-5f;

        m_problemParams.set("RotationDeform", m_rotationDeform);
        m_problemParams.set("TranslationDeform", m_translationDeform);

        m_problemParams.set("CanonicalVertices", m_canonicalVerticesOpt);
        m_problemParams.set("LiveVertices", m_liveVerticesOpt);

        m_problemParams.set("CanonicalNormals", m_canonicalNormalsOpt);
        m_problemParams.set("LiveNormals", m_liveNormalsOpt);

        m_problemParams.set("Weights", m_weights);

        m_problemParams.set("DataG", m_data_graph);
//        m_problemParams.set("RegG", m_reg_graph);

        m_solverParams.set("nIterations", &m_combinedSolverParameters.nonLinearIter);
        m_solverParams.set("lIterations", &m_combinedSolverParameters.linearIter);
        m_solverParams.set("function_tolerance", &m_functionTolerance);
//        m_solverParams.set("max_trust_region_radius", &m_trust_region_radius);
//        m_solverParams.set("q_tolerance", &m_paramTolerance);
    }

    virtual void preSingleSolve() override {
//        resetGPUMemory();
    }
    virtual void postSingleSolve() override {
        copyResultToCPUFromFloat3();
    }

    virtual void preNonlinearSolve(int) override {}

    virtual void postNonlinearSolve(int) override {}

    virtual void combinedSolveFinalize() override {
        reportFinalCosts("Robust Mesh Deformation", m_combinedSolverParameters, getCost("Opt(GN)"), getCost("Opt(LM)"), nan(""));
    }

    std::vector<Eigen::Vector3f> result()
    {
        return m_resultVertices;
    }

private:
	WarpField *m_warp;
	// Current index in solve
    std::vector<unsigned int> m_dims;

    std::shared_ptr<OptImage> m_rotationDeform;
    std::shared_ptr<OptImage> m_translationDeform;

    std::shared_ptr<OptImage> m_canonicalVerticesOpt;
    std::shared_ptr<OptImage> m_liveVerticesOpt;
    std::shared_ptr<OptImage> m_canonicalNormalsOpt;
    std::shared_ptr<OptImage> m_liveNormalsOpt;
    std::shared_ptr<OptImage> m_weights;
    std::shared_ptr<OptGraph> m_reg_graph;
    std::shared_ptr<OptGraph> m_data_graph;

    std::vector<Eigen::Vector3f> m_canonicalVerticesEigen;
    std::vector<Eigen::Vector3f> m_canonicalNormalsEigen;
    std::vector<Eigen::Vector3f> m_liveVerticesEigen;
    std::vector<Eigen::Vector3f> m_liveNormalsEigen;
    std::vector<Eigen::Vector3f> m_resultVertices;


    float m_functionTolerance;
    float m_paramTolerance;
    float m_trust_region_radius;
	
};





#endif