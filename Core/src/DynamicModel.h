#ifndef DYNAMIC_MODEL_H_
#define DYNAMIC_MODEL_H_

#include "Shaders/Shaders.h"
#include "Shaders/Uniform.h"
#include "Shaders/FeedbackBuffer.h"
#include "GPUTexture.h"
#include "Utils/Resolution.h"
#include "IndexMap.h"
#include "Utils/Stopwatch.h"
#include "Utils/Intrinsics.h"
#include "Utils/RGBDOdometry.h"
#include <pangolin/gl/gl.h>
#include <Eigen/LU>

#include "Cuda/cudafuncs.cuh"

#include "Defines.h"

class DynamicModel
{
public:
	DynamicModel();
	virtual ~DynamicModel();

	void initialise(const FeedbackBuffer & rawFeedback,
                        const FeedbackBuffer & filteredFeedback);

  void initialiseResidual(const FeedbackBuffer & filteredFeedback);

	static const int TEXTURE_DIMENSION;
    static const int MAX_VERTICES;
    static const int NODE_TEXTURE_DIMENSION;
    static const int MAX_NODES;


    EFUSION_API void renderPointCloud(pangolin::OpenGlMatrix mvp,
                          const float threshold,
                          const bool drawUnstable,
                          const bool drawNormals,
                          const bool drawColors,
                          const bool drawPoints,
                          const bool drawWindow,
                          const bool drawTimes,
                          const int time,
                          const int timeDelta);

    EFUSION_API const std::pair<GLuint, GLuint> & model();

    void fuse(const Eigen::Matrix4f & pose,
              const int & time,
              GPUTexture * rgb,
              GPUTexture * depthRaw,
              GPUTexture * depthFiltered,
              GPUTexture * indexMap,
              GPUTexture * vertConfMap,
              GPUTexture * colorTimeMap,
              GPUTexture * normRadMap,
              const float depthCutoff,
              const float confThreshold,
              const float weighting);

    void clean(const Eigen::Matrix4f & pose,
               const int & time,
               GPUTexture * indexMap,
               GPUTexture * vertConfMap,
               GPUTexture * colorTimeMap,
               GPUTexture * normRadMap,
               GPUTexture * depthMap,
               const float confThreshold,
               std::vector<float> & graph,
               const int timeDelta,
               const float maxDepth,
               const bool isFern);

    EFUSION_API unsigned int lastCount();

    Eigen::Vector4f * downloadMap();

    std::vector<float> psdf(const std::vector<Eigen::Vector3f>& warped, DeviceArray2D<unsigned short>& cur_depth);
	

private:
    //First is the vbo, second is the fid
    std::pair<GLuint, GLuint> * vbos;
    int target, renderSource;

    const int bufferSize;

    GLuint countQuery;
    unsigned int count;

    std::shared_ptr<Shader> initProgram;
    std::shared_ptr<Shader> drawProgram;
    std::shared_ptr<Shader> drawSurfelProgram;

    //For supersample fusing
    std::shared_ptr<Shader> dataProgram;
    std::shared_ptr<Shader> updateProgram;
    std::shared_ptr<Shader> unstableProgram;
    pangolin::GlRenderBuffer renderBuffer;

    //We render updated vertices vec3 + confidences to one texture
    GPUTexture updateMapVertsConfs;

    //We render updated colors vec3 + timestamps to another
    GPUTexture updateMapColorsTime;

    //We render updated normals vec3 + radii to another
    GPUTexture updateMapNormsRadii;

    //16 floats stored column-major yo'
    GPUTexture deformationNodes;

    GLuint newUnstableVbo, newUnstableFid;

    pangolin::GlFramebuffer frameBuffer;
    GLuint uvo;
    int uvSize;
};





#endif